"""Validacion ESTATICA del deploy: caza fallas de despliegue sin el build arm64.

Cubre la clase de fallos que vimos en `prueba`:
- el CMD del Dockerfile no pasaba `veh:=` (el launch lo exige sin default) -> `docker
  run` fallaba al instante;
- launches incluidos que no existen en el repo;
- imagen del compose distinta a la que construye build_image.sh;
- nodos ROS que importan paquetes NO copiados a la imagen;
- requirements-robot.txt referenciado pero ausente / sin el tope de numpy.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Paquetes de launch que SON nuestros (estan en el repo). El resto (p.ej.
# duckietown_demos) vienen de la imagen base dt-core y no se validan aqui.
PKG_DIRS = {
    "bringup": ROOT / "bringup",
    "object_detection": ROOT / "perception/packages/object_detection",
    "duckietown_planning": ROOT / "planning/packages/duckietown_planning",
}
# Paquetes de nivel superior que el Dockerfile NO copia (no deben importarse en runtime).
COPIED = {"perception", "planning", "bringup"}
NOT_COPIED = {"control", "diagnostics", "simulation", "platform_support", "rl", "training"}

_INCLUDE = re.compile(r'file="\$\(find ([^)]+)\)([^"]*)"')


def _read(rel):
    return (ROOT / rel).read_text()


def test_launches_incluidos_existen():
    """Todo <include file="$(find PKG)/..."> de un PKG nuestro debe existir."""
    faltan = []
    for lf in ["bringup/full_stack.launch", "bringup/lane_following.launch"]:
        for pkg, rest in _INCLUDE.findall(_read(lf)):
            if pkg in PKG_DIRS:
                target = PKG_DIRS[pkg] / rest.lstrip("/")
                if not target.exists():
                    faltan.append(f"{lf}: {pkg}{rest}")
    assert not faltan, f"launches referenciados inexistentes: {faltan}"


def test_dockerfile_cmd_pasa_veh():
    """El CMD debe pasar veh:= (el launch lo exige sin default) -> no falla docker run."""
    cmd = [l for l in _read("Dockerfile").splitlines()
           if l.strip().startswith("CMD")]
    assert cmd, "Dockerfile sin CMD"
    assert "veh:=" in cmd[-1], f"CMD no pasa veh:= -> docker run fallaria: {cmd[-1]}"


def test_compose_command_pasa_veh():
    assert "veh:=" in _read("bringup/docker-compose.yaml")


def test_compose_image_igual_a_build_default():
    """La imagen del compose debe coincidir con el TAG por defecto de build_image.sh."""
    compose = _read("bringup/docker-compose.yaml")
    img = re.search(r"image:\s*(\S+)", compose).group(1)
    build = _read("tools/build_image.sh")
    default_tag = re.search(r'TAG="\$\{1:-([^}]+)\}"', build).group(1)
    assert img == default_tag, f"compose usa {img} pero build crea {default_tag}"


def test_dockerfile_requirements_robot():
    """Dockerfile referencia requirements-robot.txt, que existe y topa numpy<2 (ABI ROS)."""
    df = _read("Dockerfile")
    assert "requirements-robot.txt" in df
    req = ROOT / "requirements-robot.txt"
    assert req.exists()
    assert "numpy<2" in req.read_text()


def test_dockerfile_copia_paquetes_ros():
    df = _read("Dockerfile")
    for pkg in ("perception", "planning", "bringup"):
        assert re.search(rf"COPY\s+\./{pkg}\b", df), f"Dockerfile no copia {pkg}"


def test_nodos_runtime_no_importan_paquetes_no_copiados():
    """Ningun .py de runtime (perception/planning, sin tests) importa un paquete de
    nivel superior que el Dockerfile NO copia -> evita ImportError en el robot."""
    ofensores = []
    pat = re.compile(r"^\s*(?:from|import)\s+(" + "|".join(NOT_COPIED) + r")\b", re.M)
    for base in ("perception", "planning"):
        for py in (ROOT / base).rglob("*.py"):
            if "/tests/" in py.as_posix() or "__pycache__" in py.as_posix():
                continue
            for m in pat.findall(py.read_text()):
                ofensores.append(f"{py.relative_to(ROOT)}: importa {m}")
    assert not ofensores, f"imports no copiados al deploy: {ofensores}"
