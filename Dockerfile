# Dockerfile — imagen arm64 para el Duckiebot DB21-J4 (distro ente).
# Construir con: dts devel build -H <hostname>
# NOTA: alinear BASE_IMAGE/BASE_TAG con el template-ros de tu version de dts.
ARG ARCH=arm64v8
ARG DISTRO=ente
ARG BASE_IMAGE=dt-ros-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
# El robot es arm64: la plataforma de la imagen base se declara para que el build
# (y el chequeo en CI, que corre en amd64) no dependan de la maquina que construye.
ARG BASE_PLATFORM=linux/arm64
FROM --platform=${BASE_PLATFORM} duckietown/${BASE_IMAGE}:${BASE_TAG}

ARG REPO_PATH="${CATKIN_WS_DIR}/src/duckietown_new_version"
RUN mkdir -p "${REPO_PATH}"
COPY ./perception "${REPO_PATH}/perception"
COPY ./planning "${REPO_PATH}/planning"
COPY ./bringup "${REPO_PATH}/bringup"

COPY ./requirements-robot.txt "${REPO_PATH}/requirements-robot.txt"
RUN pip3 install -r "${REPO_PATH}/requirements-robot.txt"

# Compila el catkin_ws con nuestros paquetes
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace "${CATKIN_WS_DIR}"

# Exec-form con `bash -c`: maneja senales correctamente (JSON args) Y expande
# VEHICLE_NAME. El launch exige `veh`, asi que SIEMPRE se lo pasamos (fallback al
# hostname) para que `docker run` no falle aunque se ejecute sin docker-compose.
# El compose tambien pasa veh:=${VEHICLE_NAME}.
CMD ["/bin/bash", "-c", "roslaunch bringup full_stack.launch veh:=${VEHICLE_NAME:-$(hostname)}"]
