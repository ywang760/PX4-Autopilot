ARG ROS_IMAGE=docker.io/library/ros:jazzy-ros-base-noble@sha256:bab7e640bf79cd84957e4e18fcba7d87efc3385b4e3f36a32eeca01638e43206
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_IMAGE
ARG XRCE_AGENT_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
ARG FASTCDR_COMMIT=4611cea8cf805befc6cac8fadf0e8f75fc92405a
ARG FASTDDS_COMMIT=f052f239538ebc0df56835fdb57f7628628010bc
ARG PX4_MSGS_COMMIT=598c7aad7b2386f9406ebd2a2f841619fddc3c78

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
        build-essential \
        ca-certificates \
        cmake \
        git \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --filter=blob:none https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && git -C /tmp/Micro-XRCE-DDS-Agent checkout --detach "${XRCE_AGENT_COMMIT}" \
    && test "$(git -C /tmp/Micro-XRCE-DDS-Agent rev-parse HEAD)" = "${XRCE_AGENT_COMMIT}" \
    && sed -i \
        -e "s/set(_fastcdr_tag 2.2.x)/set(_fastcdr_tag ${FASTCDR_COMMIT})/" \
        -e "s/set(_fastdds_tag 2.14.x)/set(_fastdds_tag ${FASTDDS_COMMIT})/" \
        /tmp/Micro-XRCE-DDS-Agent/CMakeLists.txt \
    && grep -q "_fastcdr_tag ${FASTCDR_COMMIT}" /tmp/Micro-XRCE-DDS-Agent/CMakeLists.txt \
    && grep -q "_fastdds_tag ${FASTDDS_COMMIT}" /tmp/Micro-XRCE-DDS-Agent/CMakeLists.txt \
    && cmake -S /tmp/Micro-XRCE-DDS-Agent -B /tmp/Micro-XRCE-DDS-Agent/build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/Micro-XRCE-DDS-Agent/build --parallel 2 \
    && cmake --install /tmp/Micro-XRCE-DDS-Agent/build \
    && ldconfig \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

RUN mkdir -p /opt/px4_msgs_ws/src \
    && git clone --filter=blob:none https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && git -C /opt/px4_msgs_ws/src/px4_msgs checkout --detach "${PX4_MSGS_COMMIT}" \
    && test "$(git -C /opt/px4_msgs_ws/src/px4_msgs rev-parse HEAD)" = "${PX4_MSGS_COMMIT}" \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/px4_msgs_ws \
    && colcon build --merge-install --packages-select px4_msgs \
    && rm -rf build log

COPY entrypoint.sh /opt/am_fw02/entrypoint.sh
COPY shadow_monitor.py /opt/am_fw02/shadow_monitor.py
RUN chmod 0555 /opt/am_fw02/entrypoint.sh /opt/am_fw02/shadow_monitor.py

LABEL org.opencontainers.image.base.name="${ROS_IMAGE}" \
      io.lecar.xrce-agent.commit="${XRCE_AGENT_COMMIT}" \
      io.lecar.fastcdr.commit="${FASTCDR_COMMIT}" \
      io.lecar.fastdds.commit="${FASTDDS_COMMIT}" \
      io.lecar.px4-msgs.commit="${PX4_MSGS_COMMIT}" \
      io.lecar.authority="subscribe-only-lab02a"

ENV ROS_DOMAIN_ID=0
ENTRYPOINT ["/opt/am_fw02/entrypoint.sh"]
CMD ["MicroXRCEAgent", "--help"]
