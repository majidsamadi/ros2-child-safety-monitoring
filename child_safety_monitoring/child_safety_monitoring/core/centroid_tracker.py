from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from .geometry import Point2, distance


@dataclass
class Track:
    track_id: str
    center: Point2
    bbox_height: float
    missed: int = 0


class CentroidTracker:
    def __init__(self, max_distance_px: float = 120.0, max_missed: int = 8) -> None:
        self.max_distance_px = max_distance_px
        self.max_missed = max_missed
        self._next_id = 1
        self._tracks: Dict[str, Track] = {}

    def update(self, detections: List[Tuple[Point2, float]]) -> Dict[int, str]:
        assignment: Dict[int, str] = {}
        unused_tracks = set(self._tracks.keys())
        pairs = []
        for det_idx, (center, _) in enumerate(detections):
            for tid, track in self._tracks.items():
                pairs.append((distance(center, track.center), det_idx, tid))
        pairs.sort(key=lambda x: x[0])
        used_dets = set()
        for dist, det_idx, tid in pairs:
            if dist > self.max_distance_px or det_idx in used_dets or tid not in unused_tracks:
                continue
            center, height = detections[det_idx]
            self._tracks[tid] = Track(tid, center, height, missed=0)
            assignment[det_idx] = tid
            used_dets.add(det_idx)
            unused_tracks.remove(tid)
        for det_idx, (center, height) in enumerate(detections):
            if det_idx in used_dets:
                continue
            tid = f'track_{self._next_id}'
            self._next_id += 1
            self._tracks[tid] = Track(tid, center, height, missed=0)
            assignment[det_idx] = tid
        for tid in list(unused_tracks):
            track = self._tracks[tid]
            track.missed += 1
            if track.missed > self.max_missed:
                del self._tracks[tid]
        return assignment
