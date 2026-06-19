#!/usr/bin/env python3
"""Captura un frame de la camara (CSI o USB) y lo valida (end-to-end).

Usa la autodeteccion + el pipeline correcto y nuestro chequeo de salud de frame.
Uso (en el robot, con camara conectada):  python3 tools/capture_test.py [imx219|usb]
"""
import sys
from pathlib import Path

P = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P / "perception/packages/object_detection/src/object_detection"))
import camera_support as cs
from camera_health import check_frame


def main(modelo=None):
    import cv2
    if modelo is None:
        # autodeteccion: dmesg + /dev/video*
        import glob
        try:
            dmesg = Path("/var/log/dmesg").read_text()
        except Exception:
            dmesg = ""
        modelo = cs.detect_camera(dmesg=dmesg, devices=glob.glob("/dev/video*")) or "usb"
    print(f"Camara: {modelo}")
    pipe = cs.gst_pipeline(modelo)
    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER) if cs.camera_info(modelo)["tipo"] != "v4l2" \
        else cv2.VideoCapture(0)
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        print("  [FALLA] no se capturo frame (revisar cinta/permisos)")
        return 1
    sano, motivo = check_frame(frame)
    print(f"  frame {frame.shape} -> salud: {motivo}")
    cv2.imwrite("/tmp/capture_test.jpg", frame)
    print("  guardado en /tmp/capture_test.jpg")
    return 0 if sano else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else None))
