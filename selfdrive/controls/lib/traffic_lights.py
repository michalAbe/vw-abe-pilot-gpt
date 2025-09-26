# selfdrive/controls/lib/traffic_lights.py

class TrafficLightController:
"""
Minimal controller: keď je RED s dostatočnou istotou, znížime cieľovú rýchlosť.
Reálne vstupy (stav semafora, confidence, vzdialenosť) zapojíme neskôr.
"""
def __init__(self):
self.red_conf_thresh = 0.65 # istota pre RED
self.lookahead_s = 4.0 # koľko s dopredu uvažujeme (rezerva)
self.min_decel = -2.0 # m/s^2 komfortné dobrzdenie
self.stop_buffer_m = 6.0 # koľko metrov pred čiarou zastaviť

def update(self, v_ego, model_distance_m, tl_state, tl_confidence):
"""
v_ego: aktuálna rýchlosť [m/s]
model_distance_m: vzdialenosť k semaforu/stop line (ak vieme; inak None)
tl_state: "RED" | "YELLOW" | "GREEN" | "UNKNOWN"
tl_confidence: 0..1
return: {"should_stop": bool, "target_v": float}
"""
if tl_state == "RED" and tl_confidence is not None and tl_confidence >= self.red_conf_thresh:
if model_distance_m is None or model_distance_m <= self.stop_buffer_m:
# bez vzdialenosti → aspoň jemne ubrať
return {"should_stop": True, "target_v": max(0.0, v_ego - 1.0)}

# v^2 = v0^2 + 2*a*s → a = -v0^2 / (2*s)
req_a = -(v_ego * v_ego) / (2.0 * max(1.0, model_distance_m - self.stop_buffer_m))
# neprekroč komfort
a_cmd = max(req_a, self.min_decel)
# krátky horizont na zníženie cieľovky
target_v = max(0.0, v_ego + a_cmd * 0.5) # ~0.5 s dopredu
return {"should_stop": True, "target_v": target_v}

return {"should_stop": False, "target_v": v_ego}
