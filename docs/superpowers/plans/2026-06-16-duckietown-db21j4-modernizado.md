# Stack modernizado DB21-J4 — Plan de implementación

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Construir un stack de autonomía moderno y robusto para el Duckiebot DB21-J4 sobre la distro Duckietown `ente`, con detección de objetos YOLO26→TensorRT, seguir línea por visión clásica, y planificación de ruta reactiva + global cargable.

**Architecture:** Repo modular que *extiende* a Duckietown vía paquetes ROS propios y Docker. La lógica pura (entrenamiento, pre/post-proceso de visión, grafos y planificación) vive en módulos Python testeables en la Mac M1; la inferencia en el robot es solo TensorRT; el carril reutiliza el `line_detector` clásico de `dt-core`.

**Tech Stack:** Python 3.13 (dev/Mac), Ultralytics 8.4.x (YOLO26), ONNX, TensorRT 8.2 (Jetson), ROS 1 Noetic, Docker, `dts` (duckietown-shell v6.x), pytest, numpy, OpenCV, PyYAML.

## Global Constraints

- **SO del robot congelado:** JetPack 4.6.6 / Ubuntu 18.04 / CUDA 10.2 / Python 3.6.9. No se actualiza. Verbatim de la spec.
- **Inferencia en el robot: solo TensorRT.** Nada de PyTorch/Ultralytics en la Nana.
- **Una sola red en GPU** (YOLO26); el carril va por visión clásica en CPU.
- **Entrenamiento siempre fuera del robot:** Colab (CUDA) o MacBook M1 (MPS). Mismo código en ambos.
- **Middleware: ROS 1 Noetic**, Docker, jerarquía `ente`. No ROS2.
- **Distro: `ente`.** Config de flasheo: `DB21J`. Credenciales por defecto `duckie`/`quackquack`.
- **Tamaño de imagen del modelo: 416×416** (múltiplo de 32, ligero para la Nana).
- **Clases del dataset (orden fijo):** `0=duckie`, `1=cone`, `2=truck`, `3=bus`.
- **Idioma:** comentarios y mensajes en español (proyecto personal/DUOC).
- **Commits frecuentes**, terminando cada mensaje con la línea `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

---

## Estructura de archivos

```
duckietown_new_version/
├── pyproject.toml                                    # config pytest + pythonpath (dev en Mac)
├── training/
│   ├── yolo26/
│   │   ├── __init__.py
│   │   ├── config.py                                 # clases, imgsz, rutas
│   │   ├── device.py                                 # seleccion cuda/mps/cpu
│   │   ├── prepare_dataset.py                         # escribe duckietown.yaml
│   │   ├── export_onnx.py                             # best.pt -> model.onnx
│   │   └── train.py                                   # entrena YOLO26n (Colab/M1)
│   ├── requirements.txt
│   └── tests/
│       ├── test_config.py
│       ├── test_device.py
│       ├── test_prepare_dataset.py
│       └── test_export_onnx.py
├── perception/                                        # dtproject ROS
│   ├── packages/object_detection/
│   │   ├── src/object_detection/
│   │   │   ├── __init__.py
│   │   │   ├── letterbox.py                           # preprocesamiento (pure-math + cv2)
│   │   │   ├── postprocess.py                         # decode/escala/filtra detecciones
│   │   │   └── trt_inference.py                       # wrapper TensorRT (corre en la Nana)
│   │   ├── launch/object_detection_node.launch
│   │   └── src/object_detection_node.py               # nodo ROS
│   └── tests/
│       ├── test_letterbox.py
│       └── test_postprocess.py
├── planning/                                          # dtproject ROS
│   ├── packages/duckietown_planning/
│   │   ├── src/duckietown_planning/
│   │   │   ├── __init__.py
│   │   │   ├── intersection_policy.py                 # giro reactivo
│   │   │   ├── map_graph.py                           # grafo de intersecciones
│   │   │   ├── route_search.py                        # BFS/Dijkstra -> secuencia de giros
│   │   │   └── localization.py                        # localizacion por conteo
│   │   ├── launch/planning.launch
│   │   ├── src/intersection_navigator_node.py         # nodo ROS reactivo
│   │   └── src/route_planner_node.py                  # nodo ROS global
│   ├── maps/example_loop.yaml                          # mapa de ejemplo
│   └── tests/
│       ├── test_intersection_policy.py
│       ├── test_map_graph.py
│       ├── test_route_search.py
│       └── test_localization.py
├── bringup/
│   └── docker-compose.yaml
├── tools/
│   ├── install_dts.sh
│   ├── check_robot.sh
│   ├── flash_sd.sh
│   └── build_trt_engine.sh                            # corre en la Nana
└── docs/
    └── runbook-competencia.md
```

---

## FASE P0 — Verificar y preparar

### Task 1: Scaffolding del repo y configuración de pruebas

**Files:**
- Create: `pyproject.toml`
- Create: `training/yolo26/__init__.py`
- Create: `perception/packages/object_detection/src/object_detection/__init__.py`
- Create: `planning/packages/duckietown_planning/src/duckietown_planning/__init__.py`
- Test: `training/tests/test_config.py` (mínimo de humo en este task)

**Interfaces:**
- Produces: layout de paquetes importables (`yolo26`, `object_detection`, `duckietown_planning`) y `pytest` configurado con `pythonpath`.

- [ ] **Step 1: Escribir el test de humo que falla**

```python
# training/tests/test_config.py
def test_import_paquetes():
    import yolo26  # noqa: F401
    import object_detection  # noqa: F401
    import duckietown_planning  # noqa: F401
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `python -m pytest training/tests/test_config.py -v`
Expected: FAIL con `ModuleNotFoundError: No module named 'yolo26'` (o falta pytest).

- [ ] **Step 3: Crear configuración y `__init__.py`**

```toml
# pyproject.toml
[project]
name = "duckietown-new-version"
version = "0.1.0"
requires-python = ">=3.9"

[tool.pytest.ini_options]
pythonpath = [
  "training",
  "perception/packages/object_detection/src",
  "planning/packages/duckietown_planning/src",
]
testpaths = ["training/tests", "perception/tests", "planning/tests"]
```

```python
# training/yolo26/__init__.py
"""Pipeline de entrenamiento YOLO26 (Colab + M1)."""
```

```python
# perception/packages/object_detection/src/object_detection/__init__.py
"""Percepcion: pre/post-proceso e inferencia TensorRT."""
```

```python
# planning/packages/duckietown_planning/src/duckietown_planning/__init__.py
"""Planificacion: politica reactiva, grafo de mapa y busqueda de ruta."""
```

- [ ] **Step 4: Instalar pytest y correr el test**

Run: `pip install pytest pyyaml numpy && python -m pytest training/tests/test_config.py -v`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml training perception planning
git commit -m "chore: scaffolding del repo y config de pytest

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Instalación de `dts` y verificación del estado del robot

**Files:**
- Create: `tools/install_dts.sh`
- Create: `tools/check_robot.sh`

**Interfaces:**
- Produces: `tools/check_robot.sh <hostname>` que reporta conectividad y estado de contenedores Duckietown; `tools/install_dts.sh` que deja `dts` operativo.

*Tarea de infraestructura: sin unit test, se valida por comandos y salida observada.*

- [ ] **Step 1: Escribir `tools/install_dts.sh`**

```bash
#!/usr/bin/env bash
# Instala/actualiza la Duckietown Shell (dts) y verifica la version.
set -euo pipefail

echo "==> Instalando duckietown-shell..."
pip3 install --user --upgrade duckietown-shell

echo "==> Version instalada:"
dts version || { echo "ERROR: dts no quedo en PATH. Agrega ~/.local/bin al PATH"; exit 1; }

echo "==> Configurando distro ente..."
dts --set-version ente 2>/dev/null || true

echo "OK: dts listo. Si es la primera vez, ejecuta: dts tok set"
```

- [ ] **Step 2: Escribir `tools/check_robot.sh`**

```bash
#!/usr/bin/env bash
# Diagnostico del Duckiebot: conectividad, SSH, contenedores y camara.
# Uso: tools/check_robot.sh <hostname>   (ej: tools/check_robot.sh quackbot)
set -uo pipefail
ROBOT="${1:?Uso: check_robot.sh <hostname>}"

echo "==> 1) Ping mDNS (${ROBOT}.local)"
ping -c 2 "${ROBOT}.local" && echo "  [OK] responde" || echo "  [FALLA] sin respuesta mDNS"

echo "==> 2) Estado general dts"
dts fleet discover || echo "  [WARN] no se pudo listar la flota"

echo "==> 3) Contenedores en el robot"
dts duckiebot diagnose "${ROBOT}" 2>/dev/null || \
  echo "  [WARN] diagnose no disponible; revisa el dashboard http://${ROBOT}.local"

echo "==> 4) Dashboard"
echo "  Abre: http://${ROBOT}.local/  (camara, motores, bateria)"
echo "FIN del diagnostico."
```

- [ ] **Step 3: Dar permisos y ejecutar la instalación**

Run: `chmod +x tools/install_dts.sh tools/check_robot.sh && ./tools/install_dts.sh`
Expected: imprime la versión de `dts` (ej. `6.2.x`) y "OK: dts listo".

- [ ] **Step 4: Verificar el estado del robot**

Run: `./tools/check_robot.sh <hostname-de-tu-robot>`
Expected: reporta `[OK] responde` si el robot está encendido y en la red; si no, anota qué falla (esto define si hay que reflashear en Task 3).

- [ ] **Step 5: Commit**

```bash
git add tools/install_dts.sh tools/check_robot.sh
git commit -m "feat(tools): scripts de instalacion de dts y diagnostico del robot

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Flasheo de la SD (solo si el diagnóstico lo requiere)

**Files:**
- Create: `tools/flash_sd.sh`

**Interfaces:**
- Produces: `tools/flash_sd.sh <hostname>` que envuelve `dts init_sd_card` con la configuración DB21J/ente.

*Tarea de infraestructura. Ejecutar SOLO si Task 2 mostró que el robot no arranca o no está en `ente`.*

- [ ] **Step 1: Escribir `tools/flash_sd.sh`**

```bash
#!/usr/bin/env bash
# Flashea la microSD para un DB21-J4 con la distro ente.
# Uso: tools/flash_sd.sh <hostname> <wifi_ssid> <wifi_pass>
set -euo pipefail
HOST="${1:?Uso: flash_sd.sh <hostname> <wifi_ssid> <wifi_pass>}"
SSID="${2:?falta wifi_ssid}"
PASS="${3:?falta wifi_pass}"

echo "==> ADVERTENCIA: esto BORRA la microSD seleccionada."
echo "    Robot: ${HOST}  |  Config: DB21J  |  Distro: ente"
read -r -p "Continuar? (escribe SI): " ok
[ "$ok" = "SI" ] || { echo "Cancelado."; exit 1; }

dts init_sd_card \
  --hostname "${HOST}" \
  --type duckiebot \
  --configuration DB21J \
  --wifi "${SSID}:${PASS}" \
  --country CL

echo "OK: SD flasheada. Inserta en el robot, enciende y espera ~5 min."
echo "    Luego: tools/check_robot.sh ${HOST}"
```

- [ ] **Step 2: Validar la sintaxis del script**

Run: `chmod +x tools/flash_sd.sh && bash -n tools/flash_sd.sh && echo "sintaxis OK"`
Expected: `sintaxis OK`.

- [ ] **Step 3: (Condicional) Ejecutar el flasheo**

Run: `./tools/flash_sd.sh <hostname> <ssid> <pass>` (solo si hace falta)
Expected: `dts` descarga la imagen JetPack 4.6.6, escribe la SD y termina sin error. Tras encender el robot, `tools/check_robot.sh` debe dar `[OK] responde`.

- [ ] **Step 4: Commit**

```bash
git add tools/flash_sd.sh
git commit -m "feat(tools): wrapper de flasheo dts init_sd_card para DB21J/ente

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## FASE P1 — Percepción YOLO26

### Task 4: Configuración del dataset y selección de dispositivo

**Files:**
- Create: `training/yolo26/config.py`
- Create: `training/yolo26/device.py`
- Test: `training/tests/test_config.py` (extiende), `training/tests/test_device.py`

**Interfaces:**
- Produces: `CLASSES`, `IMG_SIZE`, `DATASET_DIR`, `dataset_yaml_dict()`; `select_device(prefer="auto") -> str`.

- [ ] **Step 1: Escribir los tests que fallan**

```python
# training/tests/test_config.py  (reemplaza el contenido del Task 1)
from yolo26.config import CLASSES, IMG_SIZE, dataset_yaml_dict

def test_import_paquetes():
    import yolo26, object_detection, duckietown_planning  # noqa: F401

def test_clases_en_orden_fijo():
    assert CLASSES == ["duckie", "cone", "truck", "bus"]

def test_imgsz_multiplo_de_32():
    assert IMG_SIZE % 32 == 0

def test_dataset_yaml_dict():
    d = dataset_yaml_dict()
    assert d["names"] == {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
    assert d["train"] == "images/train"
    assert d["val"] == "images/val"
```

```python
# training/tests/test_device.py
import yolo26.device as dev

def test_prefer_explicito_se_respeta():
    assert dev.select_device("cpu") == "cpu"

def test_auto_cuda(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: True)
    monkeypatch.setattr(dev, "_mps_ok", lambda: False)
    assert dev.select_device("auto") == "cuda"

def test_auto_mps(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: False)
    monkeypatch.setattr(dev, "_mps_ok", lambda: True)
    assert dev.select_device("auto") == "mps"

def test_auto_cpu(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: False)
    monkeypatch.setattr(dev, "_mps_ok", lambda: False)
    assert dev.select_device("auto") == "cpu"
```

- [ ] **Step 2: Correr y verificar que fallan**

Run: `python -m pytest training/tests/test_config.py training/tests/test_device.py -v`
Expected: FAIL (`ImportError` / atributos inexistentes).

- [ ] **Step 3: Implementar `config.py` y `device.py`**

```python
# training/yolo26/config.py
from pathlib import Path

CLASSES = ["duckie", "cone", "truck", "bus"]   # orden fijo (Global Constraints)
IMG_SIZE = 416                                  # multiplo de 32, ligero para la Nana
DATASET_DIR = Path(__file__).resolve().parents[2] / "datasets" / "duckietown"

def dataset_yaml_dict(dataset_dir: Path = DATASET_DIR) -> dict:
    """Diccionario de dataset en formato Ultralytics."""
    return {
        "path": str(dataset_dir),
        "train": "images/train",
        "val": "images/val",
        "names": {i: n for i, n in enumerate(CLASSES)},
    }
```

```python
# training/yolo26/device.py
"""Selección de dispositivo de cómputo: cuda (Colab) / mps (M1) / cpu."""

def _cuda_ok() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False

def _mps_ok() -> bool:
    try:
        import torch
        return torch.backends.mps.is_available()
    except Exception:
        return False

def select_device(prefer: str = "auto") -> str:
    if prefer != "auto":
        return prefer
    if _cuda_ok():
        return "cuda"
    if _mps_ok():
        return "mps"
    return "cpu"
```

- [ ] **Step 4: Correr y verificar que pasan**

Run: `python -m pytest training/tests/test_config.py training/tests/test_device.py -v`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add training/yolo26/config.py training/yolo26/device.py training/tests/test_config.py training/tests/test_device.py
git commit -m "feat(training): config de dataset y seleccion de dispositivo cuda/mps/cpu

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Generación de `duckietown.yaml` para Ultralytics

**Files:**
- Create: `training/yolo26/prepare_dataset.py`
- Test: `training/tests/test_prepare_dataset.py`

**Interfaces:**
- Consumes: `dataset_yaml_dict()` (Task 4).
- Produces: `write_dataset_yaml(out_path, dataset_dir=DATASET_DIR) -> Path`.

- [ ] **Step 1: Escribir el test que falla**

```python
# training/tests/test_prepare_dataset.py
import yaml
from pathlib import Path
from yolo26.prepare_dataset import write_dataset_yaml

def test_escribe_yaml_valido(tmp_path):
    out = write_dataset_yaml(tmp_path / "duckietown.yaml", dataset_dir=tmp_path / "ds")
    assert out.exists()
    data = yaml.safe_load(Path(out).read_text())
    assert data["names"] == {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
    assert data["path"].endswith("ds")
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest training/tests/test_prepare_dataset.py -v`
Expected: FAIL (`ModuleNotFoundError` / función inexistente).

- [ ] **Step 3: Implementar `prepare_dataset.py`**

```python
# training/yolo26/prepare_dataset.py
"""Prepara el dataset oficial de Duckietown para YOLO26.

El dataset oficial (clases duckie/cone/truck/bus) se descarga desde el
ejercicio object-detection del curso (duckietown-lx, branch mooc2022) o
desde Roboflow. Este modulo solo genera el archivo de configuracion que
Ultralytics necesita; la descarga/colocacion de imagenes se documenta en
el docstring de train.py.
"""
from pathlib import Path
import yaml
from .config import dataset_yaml_dict, DATASET_DIR

def write_dataset_yaml(out_path, dataset_dir: Path = DATASET_DIR) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        yaml.safe_dump(dataset_yaml_dict(Path(dataset_dir)), f, sort_keys=False)
    return out_path
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest training/tests/test_prepare_dataset.py -v`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add training/yolo26/prepare_dataset.py training/tests/test_prepare_dataset.py
git commit -m "feat(training): generacion de duckietown.yaml para Ultralytics

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: Parámetros de exportación a ONNX

**Files:**
- Create: `training/yolo26/export_onnx.py`
- Test: `training/tests/test_export_onnx.py`

**Interfaces:**
- Produces: `build_export_kwargs(imgsz=416, opset=12) -> dict`; `export(weights, **overrides)` (envuelve Ultralytics, no se testea su I/O).

- [ ] **Step 1: Escribir el test que falla**

```python
# training/tests/test_export_onnx.py
import pytest
from yolo26.export_onnx import build_export_kwargs

def test_kwargs_por_defecto():
    k = build_export_kwargs()
    assert k["format"] == "onnx"
    assert k["imgsz"] == 416
    assert k["opset"] == 12
    assert k["simplify"] is True
    assert k["dynamic"] is False

def test_imgsz_debe_ser_multiplo_de_32():
    with pytest.raises(AssertionError):
        build_export_kwargs(imgsz=400)

def test_opset_fuera_de_rango_falla():
    with pytest.raises(AssertionError):
        build_export_kwargs(opset=20)
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest training/tests/test_export_onnx.py -v`
Expected: FAIL (función inexistente).

- [ ] **Step 3: Implementar `export_onnx.py`**

```python
# training/yolo26/export_onnx.py
"""Exporta los pesos YOLO26 entrenados a ONNX, listo para TensorRT 8.2 (Jetson)."""

def build_export_kwargs(imgsz: int = 416, opset: int = 12) -> dict:
    assert imgsz % 32 == 0, "imgsz debe ser multiplo de 32"
    assert 9 <= opset <= 17, "opset fuera de rango seguro para TensorRT 8.2"
    return dict(format="onnx", imgsz=imgsz, opset=opset, simplify=True, dynamic=False)

def export(weights: str, imgsz: int = 416, opset: int = 12) -> str:
    """Carga los pesos y exporta a ONNX. Devuelve la ruta del .onnx generado.

    Requiere ultralytics instalado (Colab/M1). No se ejecuta en el robot.
    """
    from ultralytics import YOLO
    kwargs = build_export_kwargs(imgsz=imgsz, opset=opset)
    return YOLO(weights).export(**kwargs)
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest training/tests/test_export_onnx.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add training/yolo26/export_onnx.py training/tests/test_export_onnx.py
git commit -m "feat(training): parametros de exportacion ONNX para TensorRT 8.2

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: Script de entrenamiento y requirements

**Files:**
- Create: `training/yolo26/train.py`
- Create: `training/requirements.txt`

**Interfaces:**
- Consumes: `select_device` (Task 4), `write_dataset_yaml` (Task 5).
- Produces: `runs/.../weights/best.pt` al ejecutarse en Colab/M1.

*El entrenamiento real es un comando largo (GPU); aquí se valida import y `--help`.*

- [ ] **Step 1: Escribir `training/requirements.txt`**

```text
ultralytics>=8.4.0
onnx>=1.16.0
onnxslim>=0.1.34
pyyaml>=6.0
```

- [ ] **Step 2: Escribir `training/yolo26/train.py`**

```python
# training/yolo26/train.py
"""Entrena YOLO26n con el dataset oficial de Duckietown.

Corre identico en Colab (CUDA) y en MacBook M1 (MPS):
    python -m yolo26.train --data datasets/duckietown.yaml --epochs 100

Pasos previos (documentados, no automatizados aqui):
  1. Colocar imagenes/labels YOLO en datasets/duckietown/{images,labels}/{train,val}
  2. Generar el yaml:  python -c "from yolo26.prepare_dataset import write_dataset_yaml; \
     write_dataset_yaml('datasets/duckietown.yaml')"
"""
import argparse
from .config import IMG_SIZE
from .device import select_device

def main():
    p = argparse.ArgumentParser(description="Entrena YOLO26n para Duckietown")
    p.add_argument("--data", required=True, help="ruta a duckietown.yaml")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--imgsz", type=int, default=IMG_SIZE)
    p.add_argument("--device", default="auto")
    p.add_argument("--model", default="yolo26n.pt")
    args = p.parse_args()

    device = select_device(args.device)
    print(f"==> Entrenando en device={device}")
    from ultralytics import YOLO
    model = YOLO(args.model)
    model.train(data=args.data, epochs=args.epochs, batch=args.batch,
                imgsz=args.imgsz, device=device)
    print("OK: entrenamiento terminado. Pesos en runs/detect/train/weights/best.pt")

if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Verificar que el CLI parsea (sin entrenar)**

Run: `pip install -r training/requirements.txt && python -m yolo26.train --help`
Expected: imprime el `usage:` con las opciones `--data --epochs ...` sin error de import.

- [ ] **Step 4: Entrenar (Colab o M1) — ejecución real**

Run (M1): `python -m yolo26.train --data datasets/duckietown.yaml --epochs 100 --device mps`
Expected: termina e imprime "OK: entrenamiento terminado"; existe `runs/detect/train/weights/best.pt`. Revisar mAP en la salida de Ultralytics (objetivo razonable: mAP50 > 0.7 en val).

- [ ] **Step 5: Exportar a ONNX**

Run: `python -c "from yolo26.export_onnx import export; print(export('runs/detect/train/weights/best.pt'))"`
Expected: imprime la ruta de `best.onnx`. Copiar como `model.onnx` para el robot.

- [ ] **Step 6: Commit**

```bash
git add training/yolo26/train.py training/requirements.txt
git commit -m "feat(training): script de entrenamiento YOLO26n (Colab/M1) + requirements

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: Preprocesamiento (letterbox)

**Files:**
- Create: `perception/packages/object_detection/src/object_detection/letterbox.py`
- Test: `perception/tests/test_letterbox.py`

**Interfaces:**
- Produces: `compute_letterbox(h, w, new_shape) -> (r, new_unpad, (dw, dh))` (pura, testeable); `letterbox(img, new_shape=416) -> (img_out, r, (left, top))` (usa cv2, en el robot).

- [ ] **Step 1: Escribir el test que falla**

```python
# perception/tests/test_letterbox.py
import pytest
from object_detection.letterbox import compute_letterbox

def test_imagen_cuadrada_sin_padding():
    r, new_unpad, (dw, dh) = compute_letterbox(416, 416, 416)
    assert r == 1.0
    assert new_unpad == (416, 416)
    assert (dw, dh) == (0.0, 0.0)

def test_imagen_horizontal_escala_por_ancho():
    # 480x640 -> escala por el lado mas grande (640) a 416
    r, new_unpad, (dw, dh) = compute_letterbox(480, 640, 416)
    assert r == pytest.approx(416/640)
    assert new_unpad == (416, 312)   # (w, h) escalados
    assert dh == pytest.approx((416 - 312) / 2)
    assert dw == pytest.approx(0.0)
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest perception/tests/test_letterbox.py -v`
Expected: FAIL (módulo/función inexistente).

- [ ] **Step 3: Implementar `letterbox.py`**

```python
# perception/packages/object_detection/src/object_detection/letterbox.py
"""Letterbox: redimensiona manteniendo aspecto y rellena hasta new_shape."""

def compute_letterbox(h: int, w: int, new_shape):
    """Devuelve (r, (new_w, new_h), (dw, dh)) sin tocar pixeles (testeable)."""
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)   # (H, W)
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))   # (w, h)
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    return r, new_unpad, (dw, dh)

def letterbox(img, new_shape=416, color=(114, 114, 114)):
    """Aplica letterbox a una imagen BGR (numpy). Devuelve (img_out, r, (left, top))."""
    import cv2
    h, w = img.shape[:2]
    r, new_unpad, (dw, dh) = compute_letterbox(h, w, new_shape)
    resized = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    out = cv2.copyMakeBorder(resized, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=color)
    return out, r, (left, top)
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest perception/tests/test_letterbox.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add perception/packages/object_detection/src/object_detection/letterbox.py perception/tests/test_letterbox.py
git commit -m "feat(perception): preprocesamiento letterbox con calculo testeable

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: Postprocesamiento de detecciones

**Files:**
- Create: `perception/packages/object_detection/src/object_detection/postprocess.py`
- Test: `perception/tests/test_postprocess.py`

**Interfaces:**
- Consumes: `r`, `(dw, dh)` del letterbox (Task 8).
- Produces: `scale_boxes(boxes_xyxy, r, pad, orig_shape) -> np.ndarray`; `filter_detections(dets, conf_thres=0.25) -> np.ndarray`. Formato de `dets`: `(N, 6)` = `[x1, y1, x2, y2, score, cls]` (salida end-to-end de YOLO26 en espacio letterbox 416).

- [ ] **Step 1: Escribir el test que falla**

```python
# perception/tests/test_postprocess.py
import numpy as np
from object_detection.postprocess import scale_boxes, filter_detections

def test_scale_boxes_revierte_letterbox():
    # box en espacio 416 con padding dh=52 (imagen 480x640, r=0.65)
    boxes = np.array([[100.0, 152.0, 200.0, 252.0]])  # x1,y1,x2,y2
    r = 416 / 640
    out = scale_boxes(boxes, r, (0.0, 52.0), orig_shape=(480, 640))
    # y se corrige restando dh y dividiendo por r
    assert out[0, 1] == np.float64((152.0 - 52.0) / r)
    assert out[0, 0] == np.float64(100.0 / r)

def test_scale_boxes_clipa_a_la_imagen():
    boxes = np.array([[-10.0, -10.0, 9999.0, 9999.0]])
    out = scale_boxes(boxes, 1.0, (0.0, 0.0), orig_shape=(480, 640))
    assert out[0, 0] == 0.0 and out[0, 1] == 0.0
    assert out[0, 2] == 640 and out[0, 3] == 480

def test_filter_detections_por_confianza():
    dets = np.array([
        [0, 0, 10, 10, 0.9, 0],
        [0, 0, 10, 10, 0.1, 1],
    ])
    out = filter_detections(dets, conf_thres=0.25)
    assert out.shape == (1, 6)
    assert out[0, 5] == 0
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest perception/tests/test_postprocess.py -v`
Expected: FAIL (módulo inexistente).

- [ ] **Step 3: Implementar `postprocess.py`**

```python
# perception/packages/object_detection/src/object_detection/postprocess.py
"""Postproceso de la salida YOLO26 (end-to-end, NMS-free).

La salida del engine TensorRT son cajas ya decodificadas en el espacio
letterbox 416: (N, 6) = [x1, y1, x2, y2, score, cls]. Aqui solo revertimos
el letterbox a coordenadas de la imagen original y filtramos por confianza.
"""
import numpy as np

def scale_boxes(boxes_xyxy, r, pad, orig_shape):
    """Mapea cajas del espacio letterbox a la imagen original."""
    dw, dh = pad
    out = np.asarray(boxes_xyxy, dtype=np.float64).copy()
    out[:, [0, 2]] -= dw
    out[:, [1, 3]] -= dh
    out[:, :4] /= r
    h, w = orig_shape
    out[:, [0, 2]] = out[:, [0, 2]].clip(0, w)
    out[:, [1, 3]] = out[:, [1, 3]].clip(0, h)
    return out

def filter_detections(dets, conf_thres: float = 0.25):
    """Filtra detecciones (N,6) por score >= conf_thres."""
    dets = np.asarray(dets, dtype=np.float64)
    if dets.size == 0:
        return dets.reshape(0, 6)
    return dets[dets[:, 4] >= conf_thres]
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest perception/tests/test_postprocess.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add perception/packages/object_detection/src/object_detection/postprocess.py perception/tests/test_postprocess.py
git commit -m "feat(perception): postproceso de detecciones YOLO26 (escala + filtro)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 10: Build del engine TensorRT en la Nana + wrapper de inferencia

**Files:**
- Create: `tools/build_trt_engine.sh`
- Create: `perception/packages/object_detection/src/object_detection/trt_inference.py`

**Interfaces:**
- Consumes: `model.onnx` (Task 7), `letterbox` (Task 8), `scale_boxes`/`filter_detections` (Task 9).
- Produces: `model.engine` en la Nana; clase `TRTDetector(engine_path, imgsz=416)` con método `infer(img_bgr) -> np.ndarray (N,6)`.

*El build y la inferencia corren en la Nana (TensorRT/pycuda); se validan por comando en el robot.*

- [ ] **Step 1: Escribir `tools/build_trt_engine.sh`**

```bash
#!/usr/bin/env bash
# Construye el engine TensorRT desde un ONNX. EJECUTAR EN LA JETSON NANO.
# Uso: build_trt_engine.sh model.onnx model.engine
set -euo pipefail
ONNX="${1:?Uso: build_trt_engine.sh <model.onnx> <model.engine>}"
ENGINE="${2:?falta ruta de salida .engine}"

TRTEXEC="/usr/src/tensorrt/bin/trtexec"
[ -x "$TRTEXEC" ] || { echo "ERROR: no se encontro trtexec en $TRTEXEC"; exit 1; }

echo "==> Construyendo engine FP16 (puede tardar varios minutos)..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --workspace=2048

echo "OK: engine guardado en $ENGINE"
```

- [ ] **Step 2: Validar sintaxis del script**

Run: `chmod +x tools/build_trt_engine.sh && bash -n tools/build_trt_engine.sh && echo "sintaxis OK"`
Expected: `sintaxis OK`.

- [ ] **Step 3: Escribir `trt_inference.py`**

```python
# perception/packages/object_detection/src/object_detection/trt_inference.py
"""Inferencia TensorRT para YOLO26 en la Jetson Nano (CUDA 10.2, TensorRT 8.2).

Solo se ejecuta en la Nana (requiere tensorrt + pycuda). El pre/post-proceso
reutiliza modulos testeados en la Mac (letterbox, postprocess).
"""
import numpy as np
from .letterbox import letterbox
from .postprocess import scale_boxes, filter_detections

class TRTDetector:
    def __init__(self, engine_path: str, imgsz: int = 416, conf_thres: float = 0.25):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401  (inicializa el contexto CUDA)
        import pycuda.driver as cuda

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self._cuda = cuda
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

    def _preprocess(self, img_bgr):
        img, r, (left, top) = letterbox(img_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)            # BGR->RGB, HWC->CHW
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob[None], r, (left, top)

    def infer(self, img_bgr) -> np.ndarray:
        """Devuelve detecciones (N,6)=[x1,y1,x2,y2,score,cls] en coords de img_bgr."""
        blob, r, pad = self._preprocess(img_bgr)
        raw = self._run(blob)                                # (N,6) espacio letterbox
        raw = filter_detections(raw, self.conf_thres)
        if raw.size == 0:
            return raw
        boxes = scale_boxes(raw[:, :4], r, pad, img_bgr.shape[:2])
        return np.concatenate([boxes, raw[:, 4:6]], axis=1)

    def _run(self, blob) -> np.ndarray:
        """Ejecuta el engine. Implementacion con buffers pycuda."""
        cuda = self._cuda
        inp = self.engine.get_binding_shape(0)
        out_idx = 1
        out_shape = tuple(self.context.get_binding_shape(out_idx))
        out_host = np.empty(out_shape, dtype=np.float32)
        d_in = cuda.mem_alloc(blob.nbytes)
        d_out = cuda.mem_alloc(out_host.nbytes)
        cuda.memcpy_htod(d_in, np.ascontiguousarray(blob))
        self.context.execute_v2([int(d_in), int(d_out)])
        cuda.memcpy_dtoh(out_host, d_out)
        return out_host.reshape(-1, 6)
```

- [ ] **Step 4: Construir el engine en la Nana (ejecución real en el robot)**

Run (en la Nana, vía `dts duckiebot ... ` o ssh): `./tools/build_trt_engine.sh model.onnx model.engine`
Expected: `trtexec` termina con `OK: engine guardado en model.engine` e imprime throughput (qps). Anotar los FPS (objetivo ~10–25).

- [ ] **Step 5: Commit**

```bash
git add tools/build_trt_engine.sh perception/packages/object_detection/src/object_detection/trt_inference.py
git commit -m "feat(perception): build de engine TensorRT y wrapper de inferencia YOLO26

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 11: Nodo ROS de detección de objetos

**Files:**
- Create: `perception/packages/object_detection/src/object_detection_node.py`
- Create: `perception/packages/object_detection/launch/object_detection_node.launch`

**Interfaces:**
- Consumes: `TRTDetector` (Task 10), imagen comprimida de `dt-duckiebot-interface` (`~/camera_node/image/compressed`).
- Produces: tópico `~/detections` con marcadores/lista de detecciones.

*Nodo ROS; se valida levantándolo en el robot.*

- [ ] **Step 1: Escribir el nodo**

```python
#!/usr/bin/env python3
# perception/packages/object_detection/src/object_detection_node.py
"""Nodo ROS: detecta objetos con YOLO26 (TensorRT) y publica detecciones."""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from object_detection.trt_inference import TRTDetector

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node("object_detection_node")
        engine = rospy.get_param("~engine_path", "/data/model.engine")
        self.detector = TRTDetector(engine,
                                    imgsz=rospy.get_param("~imgsz", 416),
                                    conf_thres=rospy.get_param("~conf_thres", 0.25))
        self.pub = rospy.Publisher("~detections", Float32MultiArray, queue_size=1)
        self.sub = rospy.Subscriber("~image/compressed", CompressedImage,
                                    self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("object_detection_node listo")

    def cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        dets = self.detector.infer(img)              # (N,6)
        out = Float32MultiArray()
        out.layout.dim = [MultiArrayDimension(label="n", size=dets.shape[0], stride=dets.size),
                          MultiArrayDimension(label="f", size=6, stride=6)]
        out.data = dets.astype(np.float32).flatten().tolist()
        self.pub.publish(out)

if __name__ == "__main__":
    ObjectDetectionNode()
    rospy.spin()
```

- [ ] **Step 2: Escribir el launch**

```xml
<!-- perception/packages/object_detection/launch/object_detection_node.launch -->
<launch>
  <arg name="veh" doc="hostname del robot"/>
  <group ns="$(arg veh)">
    <node name="object_detection_node" pkg="object_detection"
          type="object_detection_node.py" output="screen">
      <param name="engine_path" value="/data/model.engine"/>
      <param name="imgsz" value="416"/>
      <param name="conf_thres" value="0.25"/>
      <remap from="~image/compressed" to="camera_node/image/compressed"/>
    </node>
  </group>
</launch>
```

- [ ] **Step 3: Verificar sintaxis Python del nodo**

Run: `python -m py_compile perception/packages/object_detection/src/object_detection_node.py && echo "compila OK"`
Expected: `compila OK`.

- [ ] **Step 4: Levantar el nodo en el robot (ejecución real)**

Run (en el robot): `roslaunch object_detection object_detection_node.launch veh:=<hostname>` y luego `rostopic hz /<hostname>/object_detection_node/detections`
Expected: el tópico publica a la frecuencia de la cámara/engine (objetivo ~10–25 Hz). Al poner un patito frente a la cámara, aparecen detecciones (cls=0).

- [ ] **Step 5: Commit**

```bash
git add perception/packages/object_detection/src/object_detection_node.py perception/packages/object_detection/launch/object_detection_node.launch
git commit -m "feat(perception): nodo ROS de deteccion YOLO26 sobre TensorRT

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## FASE P2 — Seguir línea sólido

### Task 12: Configuración de seguir línea (visión clásica reutilizada)

**Files:**
- Create: `bringup/lane_following.launch`
- Create: `docs/lane-following.md`

**Interfaces:**
- Consumes: `line_detector` / `lane_controller` de `dt-core` (no se reescriben).
- Produces: un launch que levanta el pipeline clásico de seguir línea con parámetros afinados para tu pista.

*Reutiliza componentes oficiales; se valida en el robot.*

- [ ] **Step 1: Escribir el launch de seguir línea**

```xml
<!-- bringup/lane_following.launch -->
<launch>
  <arg name="veh" doc="hostname del robot"/>
  <!-- Incluye el pipeline clasico de dt-core: line_detector + ground_projection
       + lane_filter + lane_controller. No reescribimos nada; solo orquestamos. -->
  <include file="$(find duckietown_demos)/launch/lane_following.launch">
    <arg name="veh" value="$(arg veh)"/>
  </include>
</launch>
```

- [ ] **Step 2: Documentar el ajuste de color (HSV) de la pista**

```markdown
<!-- docs/lane-following.md -->
# Seguir línea (visión clásica)

Reutilizamos el `line_detector` de `dt-core` (umbral HSV de blanco/amarillo/rojo)
y el `lane_controller` (PID). No reescribimos código; afinamos parámetros.

## Ajuste para tu pista
1. Lanzar: `roslaunch bringup lane_following.launch veh:=<hostname>`
2. Abrir el dashboard y el visor de segmentos (`line_detector_node`).
3. Si la pista tiene iluminación distinta, ajustar los rangos HSV en el
   `line_detector_node` (parámetros `hsv_white`, `hsv_yellow`, `hsv_red`).
4. Validar que el robot complete una vuelta al loop sin salirse.
```

- [ ] **Step 3: Verificar XML bien formado**

Run: `python -c "import xml.dom.minidom,sys; xml.dom.minidom.parse('bringup/lane_following.launch'); print('XML OK')"`
Expected: `XML OK`.

- [ ] **Step 4: Probar seguir línea en el robot (ejecución real)**

Run (en el robot): `roslaunch bringup lane_following.launch veh:=<hostname>`
Expected: el robot sigue el carril y completa una vuelta al loop sin salirse. Ajustar HSV si hace falta (ver docs).

- [ ] **Step 5: Commit**

```bash
git add bringup/lane_following.launch docs/lane-following.md
git commit -m "feat(bringup): launch de seguir linea reutilizando dt-core + guia de ajuste HSV

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## FASE P3 — Intersecciones reactivas

### Task 13: Política reactiva de intersección

**Files:**
- Create: `planning/packages/duckietown_planning/src/duckietown_planning/intersection_policy.py`
- Test: `planning/tests/test_intersection_policy.py`

**Interfaces:**
- Produces: `choose_turn(available, planned=None, default_priority=("straight","right","left")) -> str`. `available` es lista de giros válidos (`"left"|"straight"|"right"`); `planned` es la sugerencia del planeador global o `None`.

- [ ] **Step 1: Escribir el test que falla**

```python
# planning/tests/test_intersection_policy.py
import pytest
from duckietown_planning.intersection_policy import choose_turn

def test_respeta_el_plan_si_es_valido():
    assert choose_turn(["left", "straight"], planned="left") == "left"

def test_ignora_plan_invalido_y_usa_prioridad():
    # planned=right no esta disponible -> cae a prioridad (straight primero)
    assert choose_turn(["left", "straight"], planned="right") == "straight"

def test_sin_plan_usa_prioridad_por_defecto():
    assert choose_turn(["left", "right"]) == "right"   # straight no esta, right antes que left

def test_sin_salidas_lanza_error():
    with pytest.raises(ValueError):
        choose_turn([])
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest planning/tests/test_intersection_policy.py -v`
Expected: FAIL (módulo inexistente).

- [ ] **Step 3: Implementar `intersection_policy.py`**

```python
# planning/packages/duckietown_planning/src/duckietown_planning/intersection_policy.py
"""Politica reactiva: elige un giro valido en una interseccion."""

TURNS = ("left", "straight", "right")

def choose_turn(available, planned=None,
                default_priority=("straight", "right", "left")) -> str:
    """Devuelve un giro dentro de `available`.

    - Si hay `planned` y es valido, se respeta (lo usa el planeador global).
    - Si no, se elige por `default_priority`.
    """
    if not available:
        raise ValueError("no hay salidas disponibles en la interseccion")
    if planned is not None and planned in available:
        return planned
    for t in default_priority:
        if t in available:
            return t
    return available[0]
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest planning/tests/test_intersection_policy.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add planning/packages/duckietown_planning/src/duckietown_planning/intersection_policy.py planning/tests/test_intersection_policy.py
git commit -m "feat(planning): politica reactiva de seleccion de giro en interseccion

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 14: Nodo ROS navegador de intersecciones (reactivo)

**Files:**
- Create: `planning/packages/duckietown_planning/src/intersection_navigator_node.py`
- Create: `planning/packages/duckietown_planning/launch/planning.launch`

**Interfaces:**
- Consumes: `choose_turn` (Task 13); detecta intersección (línea roja) vía `dt-core` FSM/`stop_line_filter`.
- Produces: comando de giro hacia `dt-car-interface`. Suscribe opcionalmente a `~/planned_turn` (lo publica el planeador global, Task 18).

*Nodo ROS; se valida en el robot.*

- [ ] **Step 1: Escribir el nodo**

```python
#!/usr/bin/env python3
# planning/packages/duckietown_planning/src/intersection_navigator_node.py
"""Nodo reactivo: en cada interseccion elige y ejecuta un giro."""
import rospy
from std_msgs.msg import String, Bool
from duckietown_planning.intersection_policy import choose_turn

class IntersectionNavigatorNode:
    def __init__(self):
        rospy.init_node("intersection_navigator_node")
        self.planned_turn = None
        self.pub_turn = rospy.Publisher("~turn_cmd", String, queue_size=1)
        rospy.Subscriber("~at_stop_line", Bool, self.cb_stop_line, queue_size=1)
        rospy.Subscriber("~planned_turn", String, self.cb_planned, queue_size=1)
        rospy.loginfo("intersection_navigator_node listo")

    def cb_planned(self, msg):
        self.planned_turn = msg.data or None

    def cb_stop_line(self, msg):
        if not msg.data:
            return
        # Salidas disponibles: en una interseccion estandar Duckietown asumimos
        # las tres; el FSM real puede filtrarlas. Reactivo robusto por defecto.
        available = rospy.get_param("~available_turns", ["left", "straight", "right"])
        turn = choose_turn(available, planned=self.planned_turn)
        rospy.loginfo("Interseccion -> giro: %s (plan=%s)", turn, self.planned_turn)
        self.pub_turn.publish(String(data=turn))
        self.planned_turn = None   # se consume el plan

if __name__ == "__main__":
    IntersectionNavigatorNode()
    rospy.spin()
```

- [ ] **Step 2: Escribir el launch**

```xml
<!-- planning/packages/duckietown_planning/launch/planning.launch -->
<launch>
  <arg name="veh" doc="hostname del robot"/>
  <arg name="use_global_planner" default="false"/>
  <group ns="$(arg veh)">
    <node name="intersection_navigator_node" pkg="duckietown_planning"
          type="intersection_navigator_node.py" output="screen">
      <remap from="~at_stop_line" to="stop_line_filter_node/at_stop_line"/>
    </node>
  </group>
</launch>
```

- [ ] **Step 3: Verificar sintaxis**

Run: `python -m py_compile planning/packages/duckietown_planning/src/intersection_navigator_node.py && python -c "import xml.dom.minidom; xml.dom.minidom.parse('planning/packages/duckietown_planning/launch/planning.launch'); print('OK')"`
Expected: imprime `OK` sin errores de compilación.

- [ ] **Step 4: Probar en el robot (ejecución real)**

Run (en el robot): `roslaunch duckietown_planning planning.launch veh:=<hostname>`
Expected: al llegar a una intersección (línea roja), el log imprime `Interseccion -> giro: ...` y el robot ejecuta el giro. Repetir varias intersecciones.

- [ ] **Step 5: Commit**

```bash
git add planning/packages/duckietown_planning/src/intersection_navigator_node.py planning/packages/duckietown_planning/launch/planning.launch
git commit -m "feat(planning): nodo reactivo de navegacion de intersecciones

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## FASE P4 — Planeador global (stretch)

### Task 15: Grafo del mapa de intersecciones

**Files:**
- Create: `planning/packages/duckietown_planning/src/duckietown_planning/map_graph.py`
- Create: `planning/maps/example_loop.yaml`
- Test: `planning/tests/test_map_graph.py`

**Interfaces:**
- Produces: `class MapGraph(nodes, edges)` con `neighbors(node) -> [(turn, to)]` y `turns_from(node) -> [turn]`; `load_map(path) -> MapGraph`. Esquema YAML: `nodes: [...]`, `edges: [{from, to, turn}]`.

- [ ] **Step 1: Escribir el mapa de ejemplo y el test que falla**

```yaml
# planning/maps/example_loop.yaml
# Grafo de intersecciones (no tiles): nodos = intersecciones, aristas = giros.
nodes: [A, B, C, D]
edges:
  - {from: A, to: B, turn: straight}
  - {from: B, to: C, turn: right}
  - {from: C, to: D, turn: right}
  - {from: D, to: A, turn: right}
  - {from: A, to: C, turn: right}
```

```python
# planning/tests/test_map_graph.py
from pathlib import Path
from duckietown_planning.map_graph import MapGraph, load_map

def test_neighbors_y_turns():
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "straight"}])
    assert g.neighbors("A") == [("straight", "B")]
    assert g.turns_from("A") == ["straight"]
    assert g.neighbors("B") == []

def test_load_map_del_yaml():
    g = load_map(Path("planning/maps/example_loop.yaml"))
    assert set(g.nodes) == {"A", "B", "C", "D"}
    assert ("straight", "B") in g.neighbors("A")
    assert ("right", "C") in g.neighbors("A")
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest planning/tests/test_map_graph.py -v`
Expected: FAIL (módulo inexistente).

- [ ] **Step 3: Implementar `map_graph.py`**

```python
# planning/packages/duckietown_planning/src/duckietown_planning/map_graph.py
"""Grafo dirigido de intersecciones para el planeador global."""
from pathlib import Path
import yaml

class MapGraph:
    def __init__(self, nodes, edges):
        self.nodes = list(nodes)
        self.adj = {n: [] for n in self.nodes}
        for e in edges:
            self.adj[e["from"]].append((e["turn"], e["to"]))

    def neighbors(self, node):
        return self.adj.get(node, [])

    def turns_from(self, node):
        return [turn for turn, _ in self.neighbors(node)]

def load_map(path) -> "MapGraph":
    data = yaml.safe_load(Path(path).read_text())
    return MapGraph(data["nodes"], data["edges"])
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest planning/tests/test_map_graph.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add planning/packages/duckietown_planning/src/duckietown_planning/map_graph.py planning/maps/example_loop.yaml planning/tests/test_map_graph.py
git commit -m "feat(planning): grafo de intersecciones + mapa de ejemplo

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 16: Búsqueda de ruta (secuencia de giros)

**Files:**
- Create: `planning/packages/duckietown_planning/src/duckietown_planning/route_search.py`
- Test: `planning/tests/test_route_search.py`

**Interfaces:**
- Consumes: `MapGraph` (Task 15).
- Produces: `plan_route(graph, start, goal) -> list[str] | None` (lista de giros del camino más corto, o `None` si no hay ruta; `[]` si `start == goal`).

- [ ] **Step 1: Escribir el test que falla**

```python
# planning/tests/test_route_search.py
from duckietown_planning.map_graph import MapGraph
from duckietown_planning.route_search import plan_route

def _grafo():
    return MapGraph(
        ["A", "B", "C", "D"],
        [{"from": "A", "to": "B", "turn": "straight"},
         {"from": "B", "to": "C", "turn": "right"},
         {"from": "C", "to": "D", "turn": "right"},
         {"from": "A", "to": "C", "turn": "right"}],
    )

def test_ruta_directa_mas_corta():
    # A->C directo (right) es mas corto que A->B->C
    assert plan_route(_grafo(), "A", "C") == ["right"]

def test_ruta_multietapa():
    assert plan_route(_grafo(), "A", "D") == ["right", "right"]  # A->C->D

def test_mismo_nodo():
    assert plan_route(_grafo(), "A", "A") == []

def test_sin_ruta():
    assert plan_route(_grafo(), "D", "A") is None
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest planning/tests/test_route_search.py -v`
Expected: FAIL (módulo inexistente).

- [ ] **Step 3: Implementar `route_search.py`**

```python
# planning/packages/duckietown_planning/src/duckietown_planning/route_search.py
"""Busqueda BFS del camino mas corto -> secuencia de giros."""
from collections import deque

def plan_route(graph, start, goal):
    if start == goal:
        return []
    prev = {start: None}                       # nodo -> (nodo_previo, giro)
    q = deque([start])
    while q:
        cur = q.popleft()
        for turn, nxt in graph.neighbors(cur):
            if nxt not in prev:
                prev[nxt] = (cur, turn)
                if nxt == goal:
                    return _reconstruct(prev, goal)
                q.append(nxt)
    return None

def _reconstruct(prev, goal):
    turns = []
    node = goal
    while prev[node] is not None:
        p, turn = prev[node]
        turns.append(turn)
        node = p
    turns.reverse()
    return turns
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest planning/tests/test_route_search.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add planning/packages/duckietown_planning/src/duckietown_planning/route_search.py planning/tests/test_route_search.py
git commit -m "feat(planning): busqueda BFS de ruta como secuencia de giros

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 17: Localización por conteo de intersecciones

**Files:**
- Create: `planning/packages/duckietown_planning/src/duckietown_planning/localization.py`
- Test: `planning/tests/test_localization.py`

**Interfaces:**
- Consumes: `MapGraph` (Task 15).
- Produces: `class IntersectionLocalizer(graph, start)` con `update(turn_taken) -> nodo_actual` y atributo `current`.

- [ ] **Step 1: Escribir el test que falla**

```python
# planning/tests/test_localization.py
import pytest
from duckietown_planning.map_graph import MapGraph
from duckietown_planning.localization import IntersectionLocalizer

def _grafo():
    return MapGraph(["A", "B", "C"],
                    [{"from": "A", "to": "B", "turn": "straight"},
                     {"from": "B", "to": "C", "turn": "right"}])

def test_avanza_con_giro_valido():
    loc = IntersectionLocalizer(_grafo(), "A")
    assert loc.update("straight") == "B"
    assert loc.current == "B"

def test_giro_invalido_lanza_error():
    loc = IntersectionLocalizer(_grafo(), "A")
    with pytest.raises(ValueError):
        loc.update("right")
```

- [ ] **Step 2: Correr y verificar que falla**

Run: `python -m pytest planning/tests/test_localization.py -v`
Expected: FAIL (módulo inexistente).

- [ ] **Step 3: Implementar `localization.py`**

```python
# planning/packages/duckietown_planning/src/duckietown_planning/localization.py
"""Localizacion simple: rastrea el nodo actual contando giros ejecutados."""

class IntersectionLocalizer:
    def __init__(self, graph, start):
        self.graph = graph
        self.current = start

    def update(self, turn_taken):
        for turn, nxt in self.graph.neighbors(self.current):
            if turn == turn_taken:
                self.current = nxt
                return self.current
        raise ValueError(f"giro '{turn_taken}' no valido desde '{self.current}'")
```

- [ ] **Step 4: Correr y verificar que pasa**

Run: `python -m pytest planning/tests/test_localization.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add planning/packages/duckietown_planning/src/duckietown_planning/localization.py planning/tests/test_localization.py
git commit -m "feat(planning): localizacion por conteo de intersecciones

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 18: Nodo ROS del planeador global

**Files:**
- Create: `planning/packages/duckietown_planning/src/route_planner_node.py`

**Interfaces:**
- Consumes: `load_map` (Task 15), `plan_route` (Task 16), `IntersectionLocalizer` (Task 17).
- Produces: publica `~/planned_turn` (String) que consume el navegador reactivo (Task 14). Si no hay mapa/destino, no publica → el reactivo decide solo (fallback automático).

*Nodo ROS; se valida con el reactivo en el robot/simulador.*

- [ ] **Step 1: Escribir el nodo**

```python
#!/usr/bin/env python3
# planning/packages/duckietown_planning/src/route_planner_node.py
"""Nodo global: dado un mapa + destino, publica el proximo giro planeado.

Si no hay mapa o destino, NO publica nada -> el navegador reactivo (Task 14)
decide por su cuenta. Asi el global es puramente aditivo y con fallback seguro.
"""
import rospy
from std_msgs.msg import String, Bool
from duckietown_planning.map_graph import load_map
from duckietown_planning.route_search import plan_route
from duckietown_planning.localization import IntersectionLocalizer

class RoutePlannerNode:
    def __init__(self):
        rospy.init_node("route_planner_node")
        map_path = rospy.get_param("~map_path", "")
        start = rospy.get_param("~start", "")
        goal = rospy.get_param("~goal", "")
        self.route = []
        self.loc = None
        if map_path and start and goal:
            graph = load_map(map_path)
            self.route = plan_route(graph, start, goal) or []
            self.loc = IntersectionLocalizer(graph, start)
            rospy.loginfo("Ruta planeada %s->%s: %s", start, goal, self.route)
        else:
            rospy.logwarn("Sin mapa/destino: el reactivo decidira solo.")
        self.pub = rospy.Publisher("~planned_turn", String, queue_size=1, latch=True)
        rospy.Subscriber("~at_stop_line", Bool, self.cb_stop_line, queue_size=1)
        self._publish_next()

    def _publish_next(self):
        if self.route:
            self.pub.publish(String(data=self.route[0]))

    def cb_stop_line(self, msg):
        if not msg.data or not self.route:
            return
        turn = self.route.pop(0)
        if self.loc:
            try:
                self.loc.update(turn)
            except ValueError as e:
                rospy.logwarn("Localizacion desincronizada: %s", e)
        self._publish_next()

if __name__ == "__main__":
    RoutePlannerNode()
    rospy.spin()
```

- [ ] **Step 2: Verificar sintaxis**

Run: `python -m py_compile planning/packages/duckietown_planning/src/route_planner_node.py && echo "compila OK"`
Expected: `compila OK`.

- [ ] **Step 3: Probar con mapa (ejecución real, robot/simulador)**

Run: `roslaunch duckietown_planning planning.launch veh:=<hostname> use_global_planner:=true` con params `map_path:=planning/maps/example_loop.yaml start:=A goal:=D`
Expected: el log imprime `Ruta planeada A->D: ['right', 'right']`; en cada línea roja el reactivo recibe `planned_turn` y gira según la ruta. Sin params de mapa, el reactivo decide solo (fallback).

- [ ] **Step 4: Commit**

```bash
git add planning/packages/duckietown_planning/src/route_planner_node.py
git commit -m "feat(planning): nodo del planeador global con fallback al reactivo

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## FASE P5 — Integración y runbook

### Task 19: Orquestación completa (docker-compose / bringup)

**Files:**
- Create: `bringup/docker-compose.yaml`
- Create: `bringup/full_stack.launch`

**Interfaces:**
- Consumes: nodos de `perception` (Task 11) y `planning` (Tasks 14, 18) + seguir línea (Task 12).
- Produces: arranque único del stack completo en el robot.

*Integración; se valida levantando todo en el robot.*

- [ ] **Step 1: Escribir el launch integrador**

```xml
<!-- bringup/full_stack.launch -->
<launch>
  <arg name="veh" doc="hostname del robot"/>
  <arg name="use_global_planner" default="false"/>

  <include file="$(find bringup)/lane_following.launch">
    <arg name="veh" value="$(arg veh)"/>
  </include>

  <include file="$(find object_detection)/launch/object_detection_node.launch">
    <arg name="veh" value="$(arg veh)"/>
  </include>

  <include file="$(find duckietown_planning)/launch/planning.launch">
    <arg name="veh" value="$(arg veh)"/>
    <arg name="use_global_planner" value="$(arg use_global_planner)"/>
  </include>
</launch>
```

- [ ] **Step 2: Escribir `docker-compose.yaml`**

```yaml
# bringup/docker-compose.yaml
# Levanta el stack propio junto a los drivers oficiales en el Duckiebot.
version: "3.7"
services:
  new-stack:
    image: duckietown-new-version:latest        # build arm64 con dts devel build
    network_mode: host
    privileged: true
    volumes:
      - /data:/data                              # contiene /data/model.engine
    environment:
      - VEHICLE_NAME=${VEHICLE_NAME}
    command: roslaunch bringup full_stack.launch veh:=${VEHICLE_NAME}
```

- [ ] **Step 3: Verificar formato**

Run: `python -c "import xml.dom.minidom; xml.dom.minidom.parse('bringup/full_stack.launch'); print('XML OK')" && python -c "import yaml; yaml.safe_load(open('bringup/docker-compose.yaml')); print('YAML OK')"`
Expected: imprime `XML OK` y `YAML OK`.

- [ ] **Step 4: Build arm64 y arranque completo (ejecución real)**

Run: `dts devel build -H <hostname>` y luego `roslaunch bringup full_stack.launch veh:=<hostname>`
Expected: arrancan seguir línea + detección + planificación. El robot da una vuelta completa siguiendo línea, detecta patos y resuelve intersecciones.

- [ ] **Step 5: Commit**

```bash
git add bringup/full_stack.launch bringup/docker-compose.yaml
git commit -m "feat(bringup): orquestacion completa del stack (lane+deteccion+planificacion)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 20: Suite de pruebas completa + runbook de competencia

**Files:**
- Create: `docs/runbook-competencia.md`

**Interfaces:**
- Produces: documento operativo con fallbacks en cascada y checklist del día.

- [ ] **Step 1: Correr toda la suite de unit tests**

Run: `python -m pytest -v`
Expected: PASS de todos los tests de `training/`, `perception/` y `planning/` (≈ 22 tests).

- [ ] **Step 2: Escribir el runbook**

```markdown
<!-- docs/runbook-competencia.md -->
# Runbook de competencia — DB21-J4

## Antes (en casa)
- [ ] `python -m pytest -v` todo verde.
- [ ] Engine `model.engine` construido y copiado a `/data/` del robot.
- [ ] Vuelta completa de seguir línea OK (Task 12).
- [ ] Detección de patos OK a ~10–25 FPS (Task 11).
- [ ] Intersecciones reactivas OK (Task 14).
- [ ] Batería cargada; SD respaldada.

## En el venue
1. Encender robot, `tools/check_robot.sh <hostname>` → `[OK]`.
2. Ajustar HSV a la luz del lugar si el carril falla (docs/lane-following.md).
3. Si conoces el trazado: armar `planning/maps/<pista>.yaml` y lanzar con
   `use_global_planner:=true start:=<...> goal:=<...>`.
4. Si NO lo conoces: dejar `use_global_planner:=false` (reactivo).

## Fallbacks en cascada (de más a menos funcionalidad)
- Falla el planeador global → `use_global_planner:=false` (reactivo siempre funciona).
- Falla YOLO/TensorRT → comentar el include de `object_detection` en `full_stack.launch`
  y correr solo seguir línea + intersecciones.
- Falla una intersección → bajar velocidad (`lane_controller` `v_bar`) y reintentar.
- Todo lo demás falla → `lane_following.launch` solo (mínimo: dar vueltas al loop).

## Comando de arranque
roslaunch bringup full_stack.launch veh:=<hostname> use_global_planner:=<true|false>
```

- [ ] **Step 3: Verificar el runbook**

Run: `test -f docs/runbook-competencia.md && grep -q "Fallbacks en cascada" docs/runbook-competencia.md && echo "runbook OK"`
Expected: `runbook OK`.

- [ ] **Step 4: Commit final**

```bash
git add docs/runbook-competencia.md
git commit -m "docs: runbook de competencia con fallbacks en cascada y checklist

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Notas de extensibilidad (futuro, post-competencia)

Conforme indicó el usuario, el stack es modular para ir incorporando más de Duckietown:
- **AprilTags** (`dt-core` `apriltag_detector`) → semáforos/señales e intersecciones señalizadas.
- **ToF VL53L0X** → freno de emergencia ante obstáculos cercanos (nodo nuevo en `perception/`).
- **Encoders + IMU** → mejor localización para el planeador global.
- **Duckiematrix** → más pruebas en simulador antes del robot.
- Cada uno entra como nodo/módulo nuevo + sus tests, sin tocar lo existente.
