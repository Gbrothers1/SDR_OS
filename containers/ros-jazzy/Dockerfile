FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-server \
    ros-jazzy-rclpy \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    python3-opengl \
    python3-pip \
    wget \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Install Python deps (filter out ROS-provided packages).
COPY requirements/requirements-base.txt /tmp/requirements-base.txt
RUN if [ -s /tmp/requirements-base.txt ]; then \
      grep -Ev '^(#|rclpy|geometry_msgs|sensor_msgs|python3-opengl)' /tmp/requirements-base.txt \
        | grep -v '^$' > /tmp/requirements.filtered.txt || true; \
      if [ -s /tmp/requirements.filtered.txt ]; then \
        pip3 install --no-cache-dir -r /tmp/requirements.filtered.txt; \
      fi; \
    fi

COPY . /workspace

CMD ["/workspace/scripts/ros/run_nodes.sh"]
