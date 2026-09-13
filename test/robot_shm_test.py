"""Unit tests for the shared-memory feedback ring (`ros_sugar.robot.shm`).

Pure Python -- no ROS graph, no rclpy. Exercises the writer/reader round-trip,
slot recycling and wraparound, the torn-read seqlock guard, segment growth,
writer-owns-unlink lifecycle, cross-process reads, and the manager.
"""

import multiprocessing as mp
import os
import struct

import pytest

from ros_sugar.robot.shm import (
    HEADER_SIZE,
    SLOT_ENTRY_SIZE,
    PluginShmManager,
    ShmDescriptor,
    ShmReaderCache,
    ShmRingWriter,
)


def _slot_entry_off(slot: int) -> int:
    return HEADER_SIZE + slot * SLOT_ENTRY_SIZE


def test_descriptor_pack_roundtrip():
    d = ShmDescriptor(name="sc_1_cam_visible_0", slot=2, seq=17, length=1234, slot_count=4)
    assert ShmDescriptor.unpack(d.pack()) == d


def test_single_reader_roundtrip():
    writer = ShmRingWriter("sc_test_roundtrip", slot_count=4)
    reader = ShmReaderCache()
    try:
        payloads = [os.urandom(size) for size in (300, 1, 2048, 777)]
        for i, payload in enumerate(payloads):
            desc = writer.write(payload)
            assert desc.slot == i % 4
            assert desc.seq == i
            assert reader.read(desc) == payload
    finally:
        reader.close()
        writer.close()


def test_wraparound_current_frame_always_valid():
    writer = ShmRingWriter("sc_test_wrap", slot_count=4)
    reader = ShmReaderCache()
    try:
        for i in range(10):  # more than one full lap of the 4-slot ring
            payload = os.urandom(300)
            desc = writer.write(payload)
            assert desc.slot == i % 4
            assert reader.read(desc) == payload
    finally:
        reader.close()
        writer.close()


def test_recycled_slot_returns_none():
    writer = ShmRingWriter("sc_test_recycle", slot_count=4)
    reader = ShmReaderCache()
    try:
        stale = writer.write(os.urandom(500))  # seq 0, slot 0
        for _ in range(4):  # seq 1..4; seq 4 overwrites slot 0
            writer.write(os.urandom(500))
        assert reader.read(stale) is None  # slot 0 now holds seq 4, not 0
    finally:
        reader.close()
        writer.close()


def test_torn_read_writer_mid_write_returns_none():
    writer = ShmRingWriter("sc_test_torn_state", slot_count=4)
    reader = ShmReaderCache()
    try:
        payload = os.urandom(800)
        desc = writer.write(payload)  # slot 0, state committed even
        buf = writer._shm.buf
        entry = _slot_entry_off(desc.slot)
        committed = struct.unpack_from("<Q", buf, entry)[0]

        # Simulate the writer being mid-write: state odd across every retry.
        struct.pack_into("<Q", buf, entry, committed + 1)
        assert reader.read(desc) is None

        # Restore the committed (even) state -> the frame reads cleanly again.
        struct.pack_into("<Q", buf, entry, committed)
        assert reader.read(desc) == payload
    finally:
        reader.close()
        writer.close()


def test_torn_read_frame_seq_mismatch_returns_none():
    writer = ShmRingWriter("sc_test_torn_seq", slot_count=4)
    reader = ShmReaderCache()
    try:
        payload = os.urandom(800)
        desc = writer.write(payload)
        buf = writer._shm.buf
        entry = _slot_entry_off(desc.slot)

        struct.pack_into("<Q", buf, entry + 8, desc.seq + 999)  # bump frame_seq
        assert reader.read(desc) is None

        struct.pack_into("<Q", buf, entry + 8, desc.seq)  # restore
        assert reader.read(desc) == payload
    finally:
        reader.close()
        writer.close()


def test_grow_to_new_epoch_keeps_old_readable():
    writer = ShmRingWriter("sc_test_grow", slot_count=4, min_slot_size=4096)
    reader = ShmReaderCache()
    try:
        small = os.urandom(1000)
        d_small = writer.write(small)  # slot_size 4096, epoch 0

        big = os.urandom(9000)  # > slot_size -> grow to a new epoch segment
        d_big = writer.write(big)

        assert d_big.name != d_small.name
        assert reader.read(d_big) == big
        assert reader.read(d_small) == small  # retired segment kept alive
    finally:
        reader.close()
        writer.close()


def test_reader_never_unlinks_writer_owns():
    writer = ShmRingWriter("sc_test_owner", slot_count=4)
    payload = os.urandom(2000)
    try:
        desc = writer.write(payload)

        r1 = ShmReaderCache()
        assert r1.read(desc) == payload
        r1.close()  # closing a reader must NOT unlink the segment

        r2 = ShmReaderCache()
        assert r2.read(desc) == payload  # still there
        r2.close()
    finally:
        writer.close()  # the writer is the sole unlinker

    r3 = ShmReaderCache()
    try:
        assert r3.read(desc) is None  # segment gone after the writer closed
    finally:
        r3.close()


def _child_read(desc_bytes, result_q):
    from ros_sugar.robot.shm import ShmDescriptor, ShmReaderCache

    desc = ShmDescriptor.unpack(desc_bytes)
    reader = ShmReaderCache()
    try:
        result_q.put(reader.read(desc))
    finally:
        reader.close()


# fork is intentional here: the child only attaches shared memory and reads,
# touching none of the parent's locks, so the multi-threaded-fork warning is moot.
# (spawn is not viable -- a pytest test module isn't importable by name in the child.)
@pytest.mark.filterwarnings("ignore:This process .* is multi-threaded")
def test_cross_process_read():
    writer = ShmRingWriter("sc_test_xproc", slot_count=4)
    ctx = mp.get_context("fork")
    result_q = ctx.Queue()
    try:
        payload = os.urandom(50_000)
        desc = writer.write(payload)
        proc = ctx.Process(target=_child_read, args=(desc.pack(), result_q))
        proc.start()
        got = result_q.get(timeout=10)
        proc.join(timeout=10)
        assert got == payload  # a reader in another process copies the frame out
    finally:
        writer.close()


def test_manager_writer_reuse_and_teardown():
    mgr = PluginShmManager(launcher_pid=12345, slot_count=4)
    payload = os.urandom(1000)
    try:
        w1 = mgr.writer_for("cam", "visible_image")
        assert mgr.writer_for("cam", "visible_image") is w1  # cached per channel
        w2 = mgr.writer_for("cam", "thermal_image")
        assert w2 is not w1

        desc = w1.write(payload)
        assert desc.name.startswith("sc_12345_")
        reader = ShmReaderCache()
        assert reader.read(desc) == payload
        reader.close()
    finally:
        mgr.close()

    r = ShmReaderCache()
    try:
        assert r.read(desc) is None  # manager.close() unlinked every segment
    finally:
        r.close()


def test_slot_count_floor():
    with pytest.raises(ValueError):
        ShmRingWriter("sc_test_bad", slot_count=1)


def test_writer_lifecycle_no_resource_tracker_error():
    """A writer + reader on one segment, then teardown, must leave a clean
    stderr. The reader and the writer share a resource_tracker here, so a
    double-unregister (reader drops the name, then the writer's unlink drops it
    again) would make the tracker raise KeyError. Runs in a fresh subprocess
    loading shm.py standalone so the tracker's output is visible."""
    import subprocess
    import sys

    from ros_sugar.robot import shm as shm_mod

    script = (
        "import importlib.util\n"
        f"spec = importlib.util.spec_from_file_location('shm_standalone', {shm_mod.__file__!r})\n"
        "m = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(m)\n"
        "w = m.ShmRingWriter('sc_test_rt_lifecycle', slot_count=4)\n"
        "r = m.ShmReaderCache()\n"
        "d = w.write(bytes(4096))\n"
        "assert r.read(d) is not None\n"
        "r.close()\n"
        "w.close()\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True
    )
    assert proc.returncode == 0, proc.stderr
    assert "resource_tracker" not in proc.stderr, proc.stderr
    assert "Traceback" not in proc.stderr, proc.stderr
