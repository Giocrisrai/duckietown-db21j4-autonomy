"""Lazo de marcha integrado: deteccion de linea -> duty de ruedas.

Une las piezas portadas en un solo paso testeable:
    bbox (hsv_lane) -> cross_track_error -> LanePID(omega) -> vw_to_duty -> safe_duty

Es agnostico al hardware: recibe una deteccion (lo que devuelve
`object_detection.hsv_lane.detect_yellow_line`) y entrega (u_l, u_r) en [-max_duty,
max_duty], listo para enviarse a un driver de motor (HUT sana o externo via
hw_profile). La politica sin-linea es DETENERSE (fail-safe).
"""
from lane_follow_pid import cross_track_error, LanePID
from kinematics import vw_to_duty, DEFAULTS
from motor_safety import safe_duty


class LaneFollowLoop:
    def __init__(self, v_nominal=0.2, kin=None, max_duty=0.5, pid=None,
                 lane_offset_px=0):
        self.v_nominal = v_nominal
        self.kin = kin or DEFAULTS
        self.max_duty = max_duty
        self.pid = pid or LanePID()
        self.lane_offset_px = lane_offset_px

    def reset(self):
        self.pid.reset()

    def step(self, det, dt):
        """(u_l, u_r) en [-max_duty, max_duty]. Sin linea -> (0,0) (fail-safe)."""
        if not getattr(det, "found", False):
            return (0.0, 0.0)
        center_x = det.frame_w // 2
        diff = cross_track_error(det.x, det.y, det.h, center_x,
                                 lane_offset_px=self.lane_offset_px)
        omega = self.pid.update(diff, dt)
        u_l, u_r = vw_to_duty(self.v_nominal, omega, self.kin)
        return safe_duty(u_l, self.max_duty), safe_duty(u_r, self.max_duty)
