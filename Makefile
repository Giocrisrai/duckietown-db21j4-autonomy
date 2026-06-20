# Makefile — flujos del stack modernizado DB21-J4
.PHONY: setup test sim sim-auto dataset-check train export validate

setup:           ## instala dependencias de desarrollo (Mac/Colab)
	pip install -r requirements-dev.txt

test:            ## corre toda la suite de tests
	python -m pytest -q

sim:             ## simulacion de lazo cerrado del planeador
	python -m pytest simulation/tests -q

sim-auto:        ## demo: mision autonoma end-to-end (cerebro completo, sin robot)
	PYTHONPATH=simulation:planning/packages/duckietown_planning/src:perception/packages/object_detection/src:control \
	  python -m sim_autonomy

dataset-check:   ## valida el dataset (uso: make dataset-check DATA=datasets/duckietown)
	PYTHONPATH=training python -c "from yolo26.dataset_check import check_dataset; print(check_dataset('$(DATA)'))"

train:           ## entrena YOLO26n (uso: make train DATA=datasets/duckietown.yaml)
	cd training && python -m yolo26.train --data $(DATA)

export:          ## exporta a ONNX (uso: make export W=runs/detect/train/weights/best.pt)
	cd training && python -c "from yolo26.export_onnx import export; print(export('$(W)'))"

validate:        ## valida el modelo (uso: make validate W=best.pt DATA=duckietown.yaml)
	cd training && python -c "from yolo26.validate import validate, passes_threshold; m=validate('$(W)','$(DATA)'); print('mAP50=', m, 'OK' if passes_threshold(m) else 'BAJO')"
