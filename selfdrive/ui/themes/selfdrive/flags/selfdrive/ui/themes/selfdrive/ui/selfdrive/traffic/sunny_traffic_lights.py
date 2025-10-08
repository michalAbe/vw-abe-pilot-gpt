from selfdrive.flags.abe_features import AbeFeatures

class SunnyTrafficLights:
    def __init__(self):
        pass

    def process_frame(self, frame):
        if AbeFeatures.TRAFFIC_LIGHTS:
            # Just log detection, no control yet
            print("Traffic light detected (stub)")
        return None
