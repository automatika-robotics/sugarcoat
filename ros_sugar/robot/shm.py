"""Shared-memory ring buffer for large robot-plugin feedback payloads.

When a plugin feedback (a camera frame, a large custom struct) has to cross the process
boundary between the plugin HOST and a component under multiprocess launch,
this module carries the raw payload through POSIX shared memory instead. The
HOST writes the frame into a slot of a per-channel ring buffer and publishes
only a tiny descriptor (segment name, slot, sequence, length) over the existing
bus socket. The consumer maps the segment and copies the frame out of the slot.
The big buffer never gets CDR-encoded and never streams through the socket.

Design Elements:

* Single writer, many readers, per feedback channel. One `ShmRingWriter`
  owns each segment and is the sole party that ever ``unlink``s it; readers only
  attach and ``close``.
* **Torn-read safety** is a per-slot seqlock (a ``state`` counter bumped to odd
  before a write and to even after) combined with a ring depth of a few slots and
  a reader that copies the slot out immediately. A reader that loses the race
  gets ``None`` (a dropped frame). It never gets corrupt bytes or a stall.
"""

import os
import re
import struct
from multiprocessing import resource_tracker, shared_memory
from typing import Dict, List, Optional

import msgpack
from attrs import define, field

try:
    from _posixshmem import shm_unlink as _shm_unlink  # POSIX low-level shm removal
except ImportError:  # pragma: no cover - non-POSIX
    _shm_unlink = None

# --- segment layout ---------------------------------------------------------
#
# [ control header | slot table (N entries) | data region (N slots) ]
#
# Header (packed little-endian, padded out to HEADER_SIZE):
#   magic(4s) version(H) flags(H) slot_count(I) slot_size(I)
#   epoch(I) data_off(I) write_seq(Q)
# Slot-table entry (SLOT_ENTRY_SIZE each):
#   state(Q, seqlock) frame_seq(Q) length(I) pad(4x)
MAGIC = b"EMOS"  # 4-byte segment signature; checked on attach
VERSION = 1

_HEADER_FMT = "<4sHHIIIIQ"
HEADER_SIZE = 64  # struct is 32 bytes; padded to a cache line
_WRITE_SEQ_OFF = 24  # offset of write_seq within the header

_SLOT_ENTRY_FMT = "<QQI4x"
SLOT_ENTRY_SIZE = struct.calcsize(_SLOT_ENTRY_FMT)  # 24
_STATE_OFF = 0  # within a slot entry
_FRAME_SEQ_OFF = 8

PAGE = 4096
DEFAULT_SLOT_COUNT = (
    4  # tradeoff between ring depth vs memory footprint (overwriteable)
)
DEFAULT_MIN_SLOT_SIZE = PAGE
_READ_RETRIES = 4


def _align(n: int, a: int) -> int:
    return (n + a - 1) // a * a


def _short(value: object, limit: int = 40) -> str:
    """Sanitize an id fragment for use in a shm segment name (``[A-Za-z0-9_]``)."""
    cleaned = re.sub(r"[^A-Za-z0-9]+", "_", str(value)).strip("_")
    return cleaned[:limit] or "x"


def segment_name_base(launcher_pid: int, plugin_id: str, feedback_key: str) -> str:
    """Base name for a channel's segments; the epoch is appended per allocation.

    Namespaced by launcher pid and plugin id + feedback key (unique per channel).
    The final name stays well under ``NAME_MAX`` for ``/dev/shm`` entries.
    """
    return f"sc_{int(launcher_pid)}_{_short(plugin_id)}_{_short(feedback_key)}"


def _open_segment(name: str, create: bool, size: int = 0) -> shared_memory.SharedMemory:
    """Create/attach a ``SharedMemory`` under fully manual lifetime management.

    The multiprocessing resource_tracker (bpo-38119) registers every segment a
    process opens — a *reader* that only attaches included — and unlinks them at
    interpreter exit. Left alone that would delete a writer's live segment out
    from under it and print a "leaked shared_memory" warning per reader. We opt
    the segment out of the tracker for **both** roles and take ownership ourselves.
    Readers only ``close`` (munmap), and the writer removes the name with `_destroy`.

    The trade is no tracker safety-net if the writer is hard-killed before
    ``close()`` resulting in a bounded, pid+epoch-named leak.
    """
    try:
        # for python3.13+
        return shared_memory.SharedMemory(
            name=name, create=create, size=size, track=False
        )
    except TypeError:
        # for python3.8-3.12
        shm = shared_memory.SharedMemory(name=name, create=create, size=size)
        try:
            resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
        except Exception:
            pass
        return shm


def _destroy(shm: shared_memory.SharedMemory) -> None:
    """Unmap and remove a segment we own, bypassing the resource_tracker.

    Unlink the name with the low-level ``shm_unlink`` rather than
    ``SharedMemory.unlink``. Falls back to ``SharedMemory.unlink`` off POSIX.
    """
    try:
        shm.close()
    except Exception:
        pass
    try:
        if _shm_unlink is not None:
            _shm_unlink(shm._name)  # type: ignore[attr-defined]
        else:  # pragma: no cover - non-POSIX fallback
            shm.unlink()
    except FileNotFoundError:
        pass
    except Exception:
        pass


@define(frozen=True)
class ShmDescriptor:
    """Everything a reader needs to locate one frame in shared memory."""

    name: str = field()
    slot: int = field()
    seq: int = field()
    length: int = field()
    slot_count: int = field()

    def pack(self) -> bytes:
        """Serialize to a compact msgpack blob for the bus."""
        return msgpack.packb(  # pyright: ignore[reportReturnType]
            {
                "n": self.name,
                "sl": self.slot,
                "sq": self.seq,
                "ln": self.length,
                "N": self.slot_count,
            },
            use_bin_type=True,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "ShmDescriptor":
        d = msgpack.unpackb(data, raw=False)
        return cls(
            name=d["n"], slot=d["sl"], seq=d["sq"], length=d["ln"], slot_count=d["N"]
        )


class ShmRingWriter:
    """The HOST side of one channel's ring. Writes frames and hands back descriptors.

    Single-writer (each feedback channel is dispatched from one thread).
    The segment is created lazily on the first ``write``, sized from the first
    frame and recreated if any subsequent frame outgrows the slow (should be rare).
    """

    def __init__(
        self,
        name_base: str,
        slot_count: int = DEFAULT_SLOT_COUNT,
        min_slot_size: int = DEFAULT_MIN_SLOT_SIZE,
    ) -> None:
        if slot_count < 2:
            raise ValueError("slot_count must be >= 2 for torn-read margin")
        self._base = name_base
        self._slot_count = slot_count
        self._min_slot_size = min_slot_size
        self._epoch = 0
        self._write_seq = 0
        self._slot_size = 0
        self._data_off = 0
        self._name = ""
        self._shm: Optional[shared_memory.SharedMemory] = None

        # Segments outgrown by a resize are kept mapped (and named) so any reader
        # still holding an old descriptor can finish, unlinked at close().
        self._retired: List[shared_memory.SharedMemory] = []

    def _create_segment(self, slot_size: int) -> None:
        name = f"{self._base}_{self._epoch}"
        # align header size to cache line
        data_off = _align(HEADER_SIZE + self._slot_count * SLOT_ENTRY_SIZE, HEADER_SIZE)
        size = data_off + self._slot_count * slot_size
        # create shared memory segment
        try:
            shm = _open_segment(name, create=True, size=size)
        except FileExistsError:
            # stale segment from a hard-killed prior run at this pid+epoch.
            _destroy(_open_segment(name, create=False))
            shm = _open_segment(name, create=True, size=size)
        struct.pack_into(
            _HEADER_FMT,
            shm.buf,
            0,  # buf write position
            MAGIC,
            VERSION,
            0,  # flags (unused, might use in a later version)
            self._slot_count,
            slot_size,
            self._epoch,
            data_off,
            self._write_seq,
        )
        # Slot table is zero-initialized by the OS. state=0 (even), frame_seq=0.
        self._shm = shm
        self._name = name
        self._slot_size = slot_size
        self._data_off = data_off

    def write(self, buffer) -> ShmDescriptor:
        """Copy ``buffer`` into the next slot and return its descriptor."""
        mv = memoryview(buffer)
        if mv.format != "B" or mv.ndim != 1:
            mv = mv.cast("B")
        n = mv.nbytes

        if self._shm is None:
            self._create_segment(_align(max(n, self._min_slot_size), PAGE))
        elif n > self._slot_size:
            self._retired.append(self._shm)
            self._epoch += 1
            self._create_segment(_align(max(n, self._min_slot_size), PAGE))

        assert self._shm is not None
        buf = self._shm.buf
        slot = self._write_seq % self._slot_count
        entry = HEADER_SIZE + slot * SLOT_ENTRY_SIZE

        # Seqlock write: state odd = write in progress, even = committed.
        # NOTE: These are plain, non-atomic stores. The reader tolerates the absence
        # of memory fences by copying out and re-checking in ShmReaderCache.read
        # Thus this is best-effort rather than a hardware guarantee.
        state = struct.unpack_from("<Q", buf, entry + _STATE_OFF)[0]  # read seqlock
        struct.pack_into("<Q", buf, entry + _STATE_OFF, state + 1)  # begin (odd)
        data_start = self._data_off + slot * self._slot_size  # this slot's data
        buf[data_start : data_start + n] = mv  # copy the frame into the slot
        # stamp this frame's seq + byte length
        struct.pack_into("<QI", buf, entry + _FRAME_SEQ_OFF, self._write_seq, n)
        struct.pack_into("<Q", buf, entry + _STATE_OFF, state + 2)  # commit (even)

        # create the descriptor for readers
        desc = ShmDescriptor(
            name=self._name,
            slot=slot,
            seq=self._write_seq,
            length=n,
            slot_count=self._slot_count,
        )
        self._write_seq += 1
        struct.pack_into("<Q", buf, _WRITE_SEQ_OFF, self._write_seq)
        return desc

    def close(self) -> None:
        """Unmap and unlink the current and all retired segments. Idempotent."""
        for shm in [self._shm, *self._retired]:
            if shm is not None:
                _destroy(shm)
        self._shm = None
        self._retired.clear()


class ShmReaderCache:
    """Consumer side. Attach segments by name (cached) and copy frames out."""

    def __init__(self, retries: int = _READ_RETRIES) -> None:
        self._retries = retries
        # name -> (shm, slot_count, slot_size, data_off)
        self._segs: Dict[str, tuple] = {}

    def _attach(self, name: str) -> Optional[tuple]:
        # if already gotten, return cached
        info = self._segs.get(name)
        if info is not None:
            return info
        try:
            shm = _open_segment(name, create=False)
        except FileNotFoundError:
            return None  # unlinked before we attached -> drop
        magic, _version, _flags, slot_count, slot_size, _epoch, data_off, _wseq = (
            struct.unpack_from(_HEADER_FMT, shm.buf, 0)
        )
        # gate on magic
        if magic != MAGIC:
            shm.close()
            return None
        # cache and return
        info = (shm, slot_count, slot_size, data_off)
        self._segs[name] = info
        return info

    def read(self, desc: ShmDescriptor) -> Optional[bytes]:
        """Return the frame ``desc`` points at, or ``None`` if it was missed.

        ``None`` means the slot was recycled, the writer was mid-write across
        every retry, or the segment is already gone. All such conditions are
        considered dropped frames.
        """
        info = self._attach(desc.name)
        if info is None:
            return None
        shm, _slot_count, slot_size, data_off = info
        buf = shm.buf
        entry = HEADER_SIZE + desc.slot * SLOT_ENTRY_SIZE
        data_start = data_off + desc.slot * slot_size
        end = data_start + desc.length

        # Seqlock validation. Read state before and after copying and trust the
        # frame only if it stayed even, unchanged, and still tagged with frame_seq
        # across the whole copy.

        # NOTE: This is best-effort. A sufficiently adversarial interleaving could
        # pass the checks below over torn bytes. In practice the reader copies
        # the slot out immediately, so for a tear to slip through, the writer would
        # have to lap the entire ``slot_count``-deep ring mid-copy. Worst case is
        # a single bad frame acceptable for a streaming sensor.
        for _ in range(self._retries):
            s0 = struct.unpack_from("<Q", buf, entry + _STATE_OFF)[0]
            # check for even
            if s0 & 1:
                continue  # writer mid-write on this slot
            # check for frame_seq
            fs0 = struct.unpack_from("<Q", buf, entry + _FRAME_SEQ_OFF)[0]
            if fs0 != desc.seq:
                return None  # slot already holds a different frame (recycled)
            # copy out
            data = bytes(buf[data_start:end])
            # check for still even and frame_seq to remain the same
            s1 = struct.unpack_from("<Q", buf, entry + _STATE_OFF)[0]
            fs1 = struct.unpack_from("<Q", buf, entry + _FRAME_SEQ_OFF)[0]
            if s0 == s1 and fs1 == desc.seq:
                return data
        return None

    def close(self) -> None:
        """Unmap every attached segment. Idempotent."""
        for info in self._segs.values():
            try:
                info[0].close()
            except Exception:
                pass
        self._segs.clear()


class PluginShmManager:
    """Owns every channel's `ShmRingWriter` for one launch.

    Created by the launcher alongside the socket feedback bus and injected into
    each plugin HOST, which requests a per-channel writer via `writer_for`.
    Closing the manager tears down every segment.
    """

    def __init__(
        self,
        launcher_pid: Optional[int] = None,
        slot_count: int = DEFAULT_SLOT_COUNT,
        min_slot_size: int = DEFAULT_MIN_SLOT_SIZE,
    ) -> None:
        self._pid = os.getpid() if launcher_pid is None else launcher_pid
        self._slot_count = slot_count
        self._min_slot_size = min_slot_size
        self._writers: Dict[str, ShmRingWriter] = {}

    def writer_for(self, plugin_id: str, feedback_key: str) -> ShmRingWriter:
        """Create shm writer certain feedback out of certain plugin"""
        key = f"{plugin_id}/{feedback_key}"
        writer = self._writers.get(key)
        if writer is None:
            writer = ShmRingWriter(
                segment_name_base(self._pid, plugin_id, feedback_key),
                slot_count=self._slot_count,
                min_slot_size=self._min_slot_size,
            )
            self._writers[key] = writer
        return writer

    def close(self) -> None:
        """Close and unlink every channel's segments. Idempotent."""
        for writer in self._writers.values():
            writer.close()
        self._writers.clear()


__all__ = [
    "ShmDescriptor",
    "ShmRingWriter",
    "ShmReaderCache",
    "PluginShmManager",
    "segment_name_base",
]
