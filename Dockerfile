ARG BASE_IMAGE=go2-control:arm64-v4
FROM ${BASE_IMAGE}
ARG BUILD_JOBS=2
ARG SOURCE_REVISION=unreleased
ENV DEBIAN_FRONTEND=noninteractive P9_HOME=/opt/p9 ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LABEL org.opencontainers.image.title="Go2 P9 Point-LIO localization" \
      org.opencontainers.image.source="https://github.com/stfhssssy/go2-p9-localization" \
      org.opencontainers.image.revision="${SOURCE_REVISION}"
SHELL ["/bin/bash", "-c"]
RUN test "$(dpkg --print-architecture)" = arm64 && \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake python3-dev python3-colcon-common-extensions python3-numpy python3-yaml python3-pytest \
      libeigen3-dev libpcl-dev libomp-dev \
      ros-jazzy-ament-cmake-python ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
      ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-sensor-msgs \
      ros-jazzy-tf2-ros ros-jazzy-diagnostic-msgs ros-jazzy-std-srvs ros-jazzy-rviz2 \
      ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-default-plugins \
      ros-jazzy-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
COPY src /opt/p9_ws/src
COPY config /opt/p9/config
COPY provenance /opt/p9/provenance
COPY docs /opt/p9/docs
COPY tests /opt/p9/tests
COPY LICENSE THIRD_PARTY_NOTICES.md /opt/p9/
COPY docker/entrypoint.sh /usr/local/bin/p9-entrypoint
RUN ln -s /opt/p9_ws/src /opt/p9/src && chmod 755 /usr/local/bin/p9-entrypoint && \
    mkdir -p /opt/p9_ws/src/point_lio_unilidar/Log /opt/p9_ws/src/point_lio_unilidar/PCD && \
    source /opt/ros/jazzy/setup.bash && cd /opt/p9_ws && \
    MAKEFLAGS="-j${BUILD_JOBS}" CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}" \
      colcon build --executor sequential --packages-select point_lio_unilidar go2_p9 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    ln -s /opt/p9_ws/install/go2_p9/lib/go2_p9/p9 /usr/local/bin/p9 && \
    source /opt/p9_ws/install/setup.bash && \
    python3 -m pytest -q /opt/p9/tests/test_sensor_core.py /opt/p9/tests/test_mapping_core.py && \
    python3 -c 'import rclpy, go2_p9.adapter_node, go2_p9.mapping_node; from ament_index_python.packages import get_package_prefix; print(get_package_prefix("point_lio_unilidar"))' && \
    sha256sum install/point_lio_unilidar/lib/point_lio_unilidar/pointlio_mapping > /opt/p9/provenance/binary.sha256 && \
    dpkg-query -W > /opt/p9/provenance/packages.txt && \
    rm -rf build log
WORKDIR /work
ENTRYPOINT ["/usr/local/bin/p9-entrypoint"]
CMD ["p9", "diagnose"]
