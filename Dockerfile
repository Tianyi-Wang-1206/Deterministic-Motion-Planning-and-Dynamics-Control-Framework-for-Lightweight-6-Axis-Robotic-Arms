# 1. Base Image: ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop

# 2. Install basic system tools, PyQt5, and rosdep via apt
RUN apt-get update && apt-get install -y \
    curl git build-essential \
    python3-pip python3-pyqt5 python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 3. Inject the local MuJoCo directory into the Docker container
COPY mujoco-3.9.0 /opt/mujoco/mujoco-3.9.0

# Set MuJoCo Environment Variables inside the container
ENV MUJOCO_DIR=/opt/mujoco/mujoco-3.9.0
ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH:-}

# 4. Install Python dependencies using pip
# [CRITICAL UPDATE FOR UBUNTU 24.04]: Added --break-system-packages to bypass PEP 668 restriction.
# Since this is an isolated Docker container, modifying system Python packages is safe.
RUN pip3 install --break-system-packages -i https://pypi.mirrors.ustc.edu.cn/simple "numpy<2.0.0" scipy

# 5. Set up the ROS 2 workspace
ENV WS_DIR=/root/lite6_ws
RUN mkdir -p ${WS_DIR}/src
WORKDIR ${WS_DIR}

# 6. Copy the project's source code into the container
COPY src ${WS_DIR}/src

# Install ROS 2 dependencies automatically via rosdep
RUN apt-get update && \
    (rosdep init || true) && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 7. Automatically source ROS 2 and the workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f ${WS_DIR}/install/setup.bash ]; then source ${WS_DIR}/install/setup.bash; fi" >> ~/.bashrc

# 8. Start with a bash shell
CMD ["bash"]