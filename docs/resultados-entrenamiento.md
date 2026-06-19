# Resultados de entrenamiento y validación (pipeline real)

Este documento registra una **ejecución real** del pipeline completo en la
MacBook M1 (MPS), para demostrar que toda la cadena funciona de punta a punta
con artefactos reales: **entrenar → validar → exportar ONNX → detectar**.

> ⚠️ Se entrenó con un **dataset sintético** (`tools/make_synthetic_dataset.py`,
> formas de colores para las 4 clases) porque aún no se dispone del dataset real
> de la pista. Los números abajo validan el pipeline, **no** son un modelo de
> competencia. Para competir, reentrena con tus imágenes reales (mismo flujo;
> Colab GPU recomendado para 100 épocas).

## Entorno

- MacBook Pro M1 Pro · `torch 2.12.1` · **MPS** · `ultralytics 8.4.71`
- Modelo base: **`yolo26n.pt`** (descargado de los assets oficiales de Ultralytics, v8.4.0)
- Dataset sintético: 120 train + 30 val, `imgsz=416`, clases `0=duckie,1=cone,2=truck,3=bus`

## Entrenamiento

```bash
.venv/bin/python tools/make_synthetic_dataset.py --root datasets/duckietown --train 120 --val 30
PYTHONPATH=training .venv/bin/python -m yolo26.train \
  --data datasets/duckietown.yaml --epochs 40 --batch 16 --imgsz 416 --device mps --model yolo26n.pt
```

- 40 épocas en **~4 min** (0.066 h) en M1 (MPS).
- Inferencia: **43.9 ms/img (~22 FPS)** en M1 (en la Jetson Nano vía TensorRT se espera ~10–25 FPS).

## Validación — mAP (set de val)

Medido con nuestro `yolo26.validate.validate(...)`:

| Clase | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| **todas** | 0.648 | 0.806 | **0.833** | **0.766** |
| duckie | 0.574 | 0.500 | 0.599 | 0.415 |
| cone | 0.572 | 0.974 | 0.917 | 0.864 |
| truck | 0.606 | 0.857 | 0.867 | 0.839 |
| bus | 0.840 | 0.895 | 0.947 | 0.947 |

`yolo26.validate.passes_threshold(0.8325)` → **True** (umbral 0.70). ✅

## Export ONNX y detección end-to-end

```bash
PYTHONPATH=training .venv/bin/python -c "from yolo26.export_onnx import export; export('runs/detect/train/weights/best.pt')"
```

- Export OK (opset 12, simplify) → `best.onnx`.
- **Salida ONNX: `[1, 300, 6]`** = formato end2end `[x1,y1,x2,y2,score,cls]`. YOLO26 es
  NMS-free nativo, así que **encaja sin cambios** con `object_detection.onnx_inference.ONNXDetector`
  y con la ruta TensorRT del robot (`trt_inference.TRTDetector`), que comparten pre/postproceso.

Detección real con nuestro `ONNXDetector` sobre imágenes de val (umbral 0.25): produjo
detecciones correctas, p. ej. `cone score=0.77`, `duckie score=0.35` — confirmando la cadena
completa modelo entrenado → ONNX → detector del proyecto.

## Reproducir con TU dataset real (competencia)

1. Coloca tus imágenes/labels YOLO en `datasets/duckietown/{images,labels}/{train,val}`
   (clases en el orden fijo `duckie,cone,truck,bus`), o ajusta `datasets/duckietown.yaml`.
2. `make dataset-check DATA=datasets/duckietown` para validar la estructura.
3. Entrena (M1: `make train DATA=datasets/duckietown.yaml`; **Colab GPU recomendado** con
   `training/notebooks/colab_yolo26.ipynb` para 100 épocas).
4. `make validate W=runs/detect/train/weights/best.pt DATA=datasets/duckietown.yaml`.
5. Exporta a ONNX y llévalo al robot: `tools/build_trt_engine.sh model.onnx model.engine`.

> Los pesos (`*.pt`), el ONNX (`*.onnx`) y `datasets/` están en `.gitignore` (binarios/datos
> no se versionan). Se regeneran con los comandos de arriba.

---

## Entrenamiento con DATASET REAL de Duckietown (competencia)

Se descargó el **dataset oficial** (público, S3, 671 MB) y se convirtió a YOLO con
`tools/convert_dtod_to_yolo.py`. Clases reales: **duckie, cone, duckiebot** (1956 frames
640×480). Mapeo a ranuras del proyecto: `duckie→0, cone→1, duckiebot→2 (truck), bus(3) sin datos`.

- Dataset: **1663 train + 293 val**.
- Entrenamiento real en M1 (MPS): YOLO26n, 25 épocas, imgsz 416, ~1 min/época.

### mAP (modelo real, validación)

| Clase | P | R | **mAP50** | mAP50-95 |
|---|---|---|---|---|
| **todas** | 0.836 | 0.837 | **0.871** | 0.604 |
| duckie | 0.731 | 0.739 | 0.762 | 0.458 |
| truck (=duckiebot) | 0.940 | 0.934 | 0.979 | 0.750 |

`validate.py` → **mAP50 = 0.8707**, pasa umbral 0.70. Inferencia 9.5 ms/img en M1.
Detección real confirmada con `ONNXDetector` sobre frames reales (multi-objeto correcto).

> **Limitaciones honestas del split actual:** `cone` (clase rara, 372 ej.) quedó sin instancias
> en val con el split secuencial; `bus` no tiene datos reales (solo simulador). Mejora pendiente:
> split **estratificado** por clase y/o mezclar datos sintéticos de truck/bus del simulador.

---

## Modelo MEJORADO (split estratificado + 50 épocas + cache)

Con el split estratificado (cone ahora en val), 50 épocas y `cache=ram`:

| Clase | P | R | **mAP50** | mAP50-95 |
|---|---|---|---|---|
| **todas** | 0.914 | 0.877 | **0.937** | 0.648 |
| duckie | 0.888 | 0.840 | 0.935 | 0.621 |
| **cone** | 0.914 | 0.837 | 0.891 | 0.537 |
| truck (=duckiebot) | 0.942 | 0.954 | 0.987 | 0.786 |

**Mejora: mAP50 0.871 → 0.937**; `cone` ahora se mide (0.891). Pesos: `runs/detect/train-3/`.

## Transfer learning — VALIDADO empíricamente

Comparación con `tools/compare_transfer.py` (dataset real, 12 épocas cada uno):

| Modelo | mAP50 |
|---|---|
| Preentrenado (yolo26n COCO → fine-tuning) | **0.857** |
| Desde cero (pesos aleatorios) | 0.584 |
| **Ventaja transfer learning** | **+0.273 (CONFIRMADO)** |

El transfer learning aporta **+27 pts de mAP** con las mismas épocas. Por eso el
pipeline usa siempre `yolo26n.pt` preentrenado como punto de partida.
