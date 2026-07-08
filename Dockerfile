# syntax=docker/dockerfile:1
# aruco_follow — Gazebo Harmonic + ArduPilot SITL + ArUco follower in one image.
# Build:  docker compose build      Run:  docker compose up
# (multi-stage: heavy sources/toolchain stay out of the runtime image)

# ── common base: Ubuntu 22.04 + OSRF Gazebo repo ───────────────────────────
FROM ubuntu:22.04 AS gz-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends curl gnupg ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update

# ── build stage: ArduPilot SITL + ardupilot_gazebo plugin ──────────────────
FROM gz-base AS build
RUN apt-get install -y --no-install-recommends \
      git build-essential cmake pkg-config rapidjson-dev libgz-sim8-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      python3 python3-dev python3-pip python3-setuptools \
 && pip3 install --no-cache-dir empy==3.3.4 pexpect future pymavlink

# ArduPilot SITL (stable Copter tag; shallow clone with full-fallback submodules)
RUN git clone --depth 1 --branch Copter-4.5.7 \
      https://github.com/ArduPilot/ardupilot /ardupilot \
 && cd /ardupilot \
 && (git submodule update --init --recursive --depth 1 \
     || git submodule update --init --recursive)
RUN cd /ardupilot && ./waf configure --board sitl && ./waf copter

# ardupilot_gazebo plugin (ArduPilotPlugin + gimbal camera plugins)
# (separate layer so the long ArduPilot compile above stays cached)
ENV GZ_VERSION=harmonic
RUN apt-get install -y --no-install-recommends libopencv-dev
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo \
      /ardupilot_gazebo \
 && cmake -S /ardupilot_gazebo -B /ardupilot_gazebo/build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 && cmake --build /ardupilot_gazebo/build -j"$(nproc)"

# ── runtime stage ───────────────────────────────────────────────────────────
FROM gz-base AS runtime
RUN apt-get install -y --no-install-recommends \
      gz-harmonic python3-gz-transport13 python3-gz-msgs10 \
      python3-pip python3-yaml iproute2 \
      libopencv-imgproc4.5d libopencv-imgcodecs4.5d libopencv-videoio4.5d \
      libopencv-calib3d4.5d \
      libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir pymavlink MAVProxy opencv-python-headless

# MAVProxy imports these but no longer declares them as pip dependencies
RUN pip3 install --no-cache-dir future lxml

COPY --from=build /ardupilot/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY --from=build /ardupilot/Tools/autotest/default_params \
      /ardupilot/Tools/autotest/default_params
COPY --from=build /ardupilot_gazebo/build /ardupilot_gazebo/build
COPY --from=build /ardupilot_gazebo/models /ardupilot_gazebo/models
COPY --from=build /ardupilot_gazebo/worlds /ardupilot_gazebo/worlds

COPY . /aruco_follow
RUN chmod +x /aruco_follow/docker/*.sh

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/ardupilot_gazebo/models:/aruco_follow/models \
    GZ_IP=127.0.0.1

WORKDIR /aruco_follow
CMD ["/aruco_follow/docker/start_sim.sh"]
