from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Point2:
    x: float
    y: float


def distance(a: Point2, b: Point2) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def midpoint(points: Iterable[Point2]) -> Point2:
    pts = list(points)
    if not pts:
        return Point2(0.0, 0.0)
    return Point2(sum(p.x for p in pts) / len(pts), sum(p.y for p in pts) / len(pts))


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def ramp_score(value: float, low: float, high: float) -> float:
    if high <= low:
        return 1.0 if value >= high else 0.0
    return clamp01((value - low) / (high - low))


def bbox_center(x: int, y: int, width: int, height: int) -> Point2:
    return Point2(float(x + width / 2.0), float(y + height / 2.0))


def safe_mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0
