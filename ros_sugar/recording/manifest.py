"""Recording manifest writer.

Produces ``<output_dir>/<recording_id>/manifest.json`` describing a recording:
what recipe/stack/runtime produced it, which sinks it wrote, and the episodes
captured within it. The recipe/stack/runtime/system_info fields are filled by
the Launcher at bringup; the Recorder fills timestamps, sink URIs, and episodes
as it runs.
"""

import json
import os
from typing import Any, Dict, List, Optional

from ..utils import logger

SCHEMA_VERSION = "1.0"


class RecordingManifest:
    """Mutable manifest accumulated over a recording's lifetime."""

    def __init__(
        self,
        recording_id: str,
        output_dir: str,
        *,
        recipe: Optional[dict] = None,
        stack: Optional[dict] = None,
        system_info: Any = None,
        runtime: Optional[dict] = None,
        modalities: Optional[list] = None,
    ) -> None:
        """Initialize an empty manifest; the Launcher fills recipe/stack/runtime."""
        self.recording_id = recording_id
        self.output_dir = os.path.expanduser(output_dir)
        self.recipe = recipe or {}
        self.stack = stack or {}
        self.system_info = system_info  # JSON string (from _build_system_info) or dict
        self.runtime = runtime or {}
        self.modalities = modalities or []
        self.sinks: Dict[str, Dict[str, Any]] = {}
        self.episodes: List[Dict[str, Any]] = []
        self.started_at_ns: Optional[int] = None
        self.closed_at_ns: Optional[int] = None

    def set_started(self, t_ns: int) -> None:
        """Stamp the recording start time (ns)."""
        self.started_at_ns = int(t_ns)

    def set_closed(self, t_ns: int) -> None:
        """Stamp the recording close time (ns)."""
        self.closed_at_ns = int(t_ns)

    def add_sink(self, name: str, uri: Optional[str]) -> None:
        """Record a sink's name and output URI."""
        self.sinks[name] = {"uri": uri}

    def add_episode(self, episode: Dict[str, Any]) -> None:
        """Append a captured episode."""
        self.episodes.append(episode)

    @property
    def path(self) -> str:
        """Path to the manifest.json on disk."""
        return os.path.join(self.output_dir, self.recording_id, "manifest.json")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize the manifest to a plain dict."""
        out: Dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "recording_id": self.recording_id,
            "started_at_ns": self.started_at_ns,
            "closed_at_ns": self.closed_at_ns,
            "recipe": self.recipe,
            "stack": self.stack,
            "runtime": self.runtime,
            "modalities": self.modalities,
            "sinks": self.sinks,
            "episodes": self.episodes,
        }
        if self.system_info is not None:
            if isinstance(self.system_info, str):
                try:
                    out["system_info"] = json.loads(self.system_info)
                except (ValueError, TypeError):
                    out["system_info"] = self.system_info
            else:
                out["system_info"] = self.system_info
        return out

    def write(self) -> str:
        """Write manifest.json to disk and return its path."""
        p = self.path
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2, default=str)
        logger.info(f"[recorder] wrote manifest {p}")
        return p
