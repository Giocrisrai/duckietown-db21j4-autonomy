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
