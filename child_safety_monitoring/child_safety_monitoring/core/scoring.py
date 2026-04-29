from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FeatureScores:
    contact: float = 0.0
    wrap: float = 0.0
    lift: float = 0.0
    feet_off_ground: float = 0.0
    limb_speed: float = 0.0
    limb_accel: float = 0.0
    co_motion: float = 0.0


def calculate_suspicion_score(features: FeatureScores) -> float:
    struggle = 0.60 * features.limb_speed + 0.40 * features.limb_accel
    score = (
        0.20 * features.contact
        + 0.15 * features.wrap
        + 0.25 * features.lift
        + 0.15 * features.feet_off_ground
        + 0.15 * struggle
        + 0.10 * features.co_motion
    )
    return max(0.0, min(1.0, score))


def state_from_score(score: float, warning_threshold: float = 0.55, high_threshold: float = 0.75) -> str:
    if score >= high_threshold:
        return 'high_alert'
    if score >= warning_threshold:
        return 'warning'
    if score > 0.25:
        return 'watch'
    return 'observing'
