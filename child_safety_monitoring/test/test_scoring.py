from child_safety_monitoring.core.scoring import FeatureScores, calculate_suspicion_score, state_from_score


def test_low_score_observing():
    score = calculate_suspicion_score(FeatureScores())
    assert score == 0.0
    assert state_from_score(score) == 'observing'


def test_high_score_alert():
    score = calculate_suspicion_score(
        FeatureScores(
            contact=1.0,
            wrap=1.0,
            lift=1.0,
            feet_off_ground=1.0,
            limb_speed=1.0,
            limb_accel=1.0,
            co_motion=1.0,
        )
    )
    assert score == 1.0
    assert state_from_score(score) == 'high_alert'
