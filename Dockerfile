# Dockerfile — imagen arm64 para el Duckiebot DB21-J4 (distro ente).
# Construir con: dts devel build -H <hostname>
# NOTA: alinear BASE_IMAGE/BASE_TAG con el template-ros de tu version de dts.
ARG ARCH=arm64v8
ARG DISTRO=ente
ARG BASE_IMAGE=dt-ros-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
FROM duckietown/${BASE_IMAGE}:${BASE_TAG}

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

CMD ["roslaunch", "bringup", "full_stack.launch"]
