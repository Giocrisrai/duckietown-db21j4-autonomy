#!/usr/bin/env bash
# Despliegue y verificacion END-TO-END en la Jetson (o cualquier placa) por SSH.
# Transfiere el repo, construye el contenedor en el dispositivo, corre la suite
# para VERIFICAR, y levanta el monitor de salud accesible por red. Sin dts.
# Uso: tools/deploy_jetson.sh user@host        (requiere llave SSH ya autorizada)
set -euo pipefail
T="${1:?Uso: deploy_jetson.sh user@host}"
echo "==> 1/4 Transfiriendo repo a $T"
rsync -az --exclude '.git' --exclude '.venv' --exclude datasets --exclude runs \
  --exclude __pycache__ --exclude '*.pyc' --exclude '.pytest_cache' \
  ./ "$T:~/duckietown_new_version/"
echo "==> 2/4 Build del contenedor (arm64 nativo en el dispositivo)"
ssh "$T" 'cd ~/duckietown_new_version && sudo docker build -f Dockerfile.dev -t dt-newver:dev .'
echo "==> 3/4 VERIFICAR: suite de tests en el dispositivo real"
ssh "$T" 'sudo docker run --rm dt-newver:dev'
echo "==> 4/4 Monitor de salud por red (puerto 8080)"
ssh "$T" 'sudo docker rm -f dt-monitor 2>/dev/null || true; sudo docker run -d --name dt-monitor -p 8080:8080 -w /work -e PYTHONPATH=diagnostics dt-newver:dev python -m run_monitor'
echo "OK: stack desplegado y VERIFICADO. Monitor: http://<host>:8080/"
