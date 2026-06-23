#!/usr/bin/env python3
"""Demo en vivo: camara + EL CEREBRO del Duckiebot, sobre el video (por VNC).

Une nuestra percepcion y control sobre frames reales de la IMX219:
  camara -> hsv_lane.detect_yellow_line -> LaneFollowLoop.step -> (u_left, u_right)
y dibuja: el bbox de la linea, el error, el duty por rueda, y la NITIDEZ (para
enfocar el lente). Asi se "ve pensar" al robot. OpenCV 3.2 (Jetson). Tecla ESC sale.
"""
import sys

REPO = "/home/prueba/duckietown_new_version"
sys.path.insert(0, REPO + "/perception/packages/object_detection/src")
sys.path.insert(0, REPO + "/control")

import cv2

from object_detection.hsv_lane import detect_yellow_line, YELLOW_LOWER, YELLOW_UPPER
try:
    from lane_follow_loop import LaneFollowLoop
    LOOP = LaneFollowLoop(v_nominal=0.2)
except Exception as e:          # si el control no importa, seguimos solo con deteccion
    LOOP = None
    print("aviso: lane_follow_loop no disponible (%s); solo deteccion" % e)

PIPE = ("nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 "
        "! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR "
        "! appsink drop=1 max-buffers=1")


def main():
    cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
    cv2.namedWindow("Cerebro Duckiebot (VNC)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Cerebro Duckiebot (VNC)", 640, 380)
    mx = 1.0
    while True:
        ok, f = cap.read()
        if not ok:
            continue
        det = detect_yellow_line(f, YELLOW_LOWER, YELLOW_UPPER)
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        sharp = cv2.Laplacian(g, cv2.CV_64F).var()
        mx = max(mx, sharp)
        disp = cv2.resize(f, (640, 360))
        sx, sy = 640.0 / f.shape[1], 360.0 / f.shape[0]
        # bbox de la linea (en coords del crop -> al frame completo)
        if det.found:
            x = int(det.x * sx); y = int((det.y + det.crop_top) * sy)
            w = int(det.w * sx); h = int(det.h * sy)
            cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # duty del cerebro
        txt = "SIN LINEA"
        if LOOP is not None:
            ul, ur = LOOP.step(det, dt=0.033)
            txt = "L:%+.2f  R:%+.2f" % (ul, ur)
        cv2.putText(disp, "linea: %s  duty %s" % ("SI" if det.found else "no", txt),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(disp, "nitidez %.0f (max %.0f) - gira el lente" % (sharp, mx),
                    (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("Cerebro Duckiebot (VNC)", disp)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
