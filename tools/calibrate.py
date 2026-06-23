#!/usr/bin/env python3
"""Calibracion del Duckiebot: genera y valida las calibraciones base.

Reune en una sola herramienta la calibracion CINEMATICA (gain/trim/baseline/
radius, formato Duckietown) y la de CAMARA (intrinsecos CameraInfo). Sin
hardware se generan los valores POR DEFECTO validos (los que trae el DB21 de
fabrica) para que el robot arranque; la calibracion fina (rueda con regla,
camara con tablero) se corre luego en el robot y sobre-escribe estos archivos.

Layout (igual al stack oficial, replicado en el robot en /data/config):
    <out>/calibrations/kinematics/<robot>.yaml
    <out>/calibrations/camera_intrinsic/<robot>.yaml

Uso:
    python3 tools/calibrate.py generate --robot prueba --camera pi-m12
    python3 tools/calibrate.py validate config/calibrations/kinematics/prueba.yaml
    python3 tools/calibrate.py show --camera pi-m12
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "planning" / "packages" / "duckietown_planning" / "src"))
sys.path.insert(0, str(ROOT / "perception" / "packages" / "object_detection" / "src"))

from duckietown_planning.calibration import (  # noqa: E402
    validate_kinematics, save_kinematics, CalibrationError,
)
from object_detection.camera_calibration import (  # noqa: E402
    validate_intrinsics, save_intrinsics, CameraCalibrationError,
)
from object_detection.camera_support import default_intrinsics, supported_cameras  # noqa: E402

DEFAULT_OUT = ROOT / "config"


def _paths(out_dir, robot):
    base = Path(out_dir) / "calibrations"
    return (
        base / "kinematics" / f"{robot}.yaml",
        base / "camera_intrinsic" / f"{robot}.yaml",
    )


def generate(robot, camera, out_dir):
    """Genera las calibraciones base (cinematica + camara) para un robot."""
    kin_path, cam_path = _paths(out_dir, robot)
    kin = validate_kinematics({})                       # DB21 de fabrica
    save_kinematics(kin_path, kin)
    intr = default_intrinsics(camera)                   # focal~ancho, centro al medio
    save_intrinsics(cam_path, intr)
    print(f"  cinematica -> {kin_path}")
    print(f"    {kin}")
    print(f"  camara ({camera}) -> {cam_path}")
    print(f"    {intr['image_width']}x{intr['image_height']}, fx={intr['camera_matrix']['data'][0]}")
    print("  OK. Calibracion fina: rueda con regla / camara con tablero en el robot.")
    return kin_path, cam_path


def validate(path):
    """Valida un YAML de calibracion (auto-detecta cinematica vs camara)."""
    import yaml
    data = yaml.safe_load(Path(path).read_text()) or {}
    if "camera_matrix" in data or "image_width" in data:
        validate_intrinsics(data)
        print(f"  camara OK: {data['image_width']}x{data['image_height']}")
    else:
        full = validate_kinematics(data)
        print(f"  cinematica OK: {full}")
    return True


def show(camera):
    """Muestra los valores por defecto sin escribir nada."""
    print(f"  cinematica (DB21 fabrica): {validate_kinematics({})}")
    intr = default_intrinsics(camera)
    print(f"  camara ({camera}): {intr['image_width']}x{intr['image_height']}, fx={intr['camera_matrix']['data'][0]}")


def main(argv=None):
    p = argparse.ArgumentParser(description="Calibracion del Duckiebot (cinematica + camara)")
    sub = p.add_subparsers(dest="cmd")
    sub.required = True                                 # compat Python 3.6 (Jetson)

    g = sub.add_parser("generate", help="genera calibraciones base")
    g.add_argument("--robot", default="prueba")
    g.add_argument("--camera", default="pi-m12", choices=supported_cameras())
    g.add_argument("--out-dir", default=str(DEFAULT_OUT))

    v = sub.add_parser("validate", help="valida un YAML de calibracion")
    v.add_argument("path")

    s = sub.add_parser("show", help="muestra los valores por defecto")
    s.add_argument("--camera", default="pi-m12", choices=supported_cameras())

    a = p.parse_args(argv)
    try:
        if a.cmd == "generate":
            generate(a.robot, a.camera, a.out_dir)
        elif a.cmd == "validate":
            validate(a.path)
        elif a.cmd == "show":
            show(a.camera)
    except (CalibrationError, CameraCalibrationError) as e:
        print(f"  CALIBRACION INVALIDA: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
