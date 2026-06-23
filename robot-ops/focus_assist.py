#!/usr/bin/env python3
"""Asistente de enfoque de la camara IMX219 (robot-side, por VNC).

Muestra la camara en vivo + un numero de NITIDEZ (varianza del Laplaciano) y una
barra: gira el anillo del lente hasta MAXIMIZAR el numero. OpenCV 3.2 (Jetson).
Uso (en el robot, con DISPLAY):  python3 robot-ops/focus_assist.py
"""
import cv2

PIPE = ("nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 "
        "! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR "
        "! appsink drop=1 max-buffers=1")


def main():
    cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
    mx = 1.0
    cv2.namedWindow("Enfoque Duckiebot", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Enfoque Duckiebot", 600, 360)
    while True:
        ok, f = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        s = cv2.Laplacian(g, cv2.CV_64F).var()
        mx = max(mx, s)
        pct = int(min(100, 100.0 * s / mx))
        disp = cv2.resize(f, (600, 338))
        cv2.putText(disp, "NITIDEZ: %.0f" % s, (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(disp, "max %.0f  -> gira el lente para SUBIR" % mx, (15, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.rectangle(disp, (15, 90), (15 + pct * 5, 105), (0, 255, 0), -1)
        cv2.rectangle(disp, (15, 90), (515, 105), (255, 255, 255), 1)
        cv2.imshow("Enfoque Duckiebot", disp)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
