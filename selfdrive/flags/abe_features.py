# Abe feature flags - central place for feature gating

class AbeFeatures:
    # Traffic lights detection / behavior (feature-flagged)
    TRAFFIC_LIGHTS = False

    # Tesla-like UI theme (visuals only)
    TESLA_THEME = False

    # Lane-change on blinker (SunnyPilot feature)
    SUNNYPILOT_LANECHANGE = False
