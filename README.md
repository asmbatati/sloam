
---

# Steps to launch in container

> **Note:** 1st, ensure to look at [docker setup](DOCKER_SETUP.md) or [local setup](LOCAL_SETUP.md) to setup the workspace. Then, ensure to follow [segmenation model setup](MODEL.md) to make the model file.

---

## Dependency Installation

Install the required system packages and ROS dependencies (for ROS Noetic on Ubuntu 20.04):

```bash
sudo apt-get update
sudo apt-get install -y \
    g++ \
    libeigen3-dev \
    git \
    python3-catkin-tools \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    build-essential \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    cmake
```

Install tf:

```bash
sudo apt-get install ros-noetic-rqt*
```

Then, update rosdep and install additional ROS dependencies:

```bash
rosdep update
rosdep install --from-paths . --ignore-src -y -r --as-root apt:false
```

---

## Building External Libraries

The GitHub Actions build file for SLOAM installs some dependencies from source. Follow these steps in a temporary directory (e.g., `/tmp`):

### glog

```bash
cd /tmp
git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake -S .. -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17
cmake --build build
sudo cmake --build build --target install
```

### fmt (with PIC enabled)

To avoid linking issues, build fmt with position-independent code:

```bash
cd /tmp
git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git
cd fmt
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE \
      -DCMAKE_CXX_STANDARD=17 \
      -DFMT_TEST=False ..
make -j$(nproc)
sudo make install
```

*Tip:* If a non‑PIC version is already installed in `/usr/local/lib`, consider renaming it to avoid conflicts.
`sudo mv /usr/local/lib/libfmt.a /usr/local/lib/libfmt.a.bak`

### Abseil

```bash
cd /tmp
git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake -DABSL_BUILD_TESTING=OFF \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
sudo cmake --build . --target install
```

Verify that the configuration file is installed at:  
`/usr/lib/x86_64-linux-gnu/cmake/absl/abslConfig.cmake`

### Sophus

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 785fef3
mkdir build && cd build
cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=17 ..
sudo make install
```

---

## Build the Workspace

Clean any previous builds and build with the proper dependency paths:

```bash
cd ~/sloam_ws
rm -rf build devel
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -Dabsl_DIR=/usr/lib/x86_64-linux-gnu/cmake/absl \
  -Dbenchmark_DIR=/usr/lib/x86_64-linux-gnu/cmake/benchmark \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
  -DBUILD_BENCHMARK=OFF
```

*Note:* The flag `-DBUILD_BENCHMARK=OFF` disables benchmark executables that may lack a `main()` function.

## Source the Workspace

   ```bash
   source ~/sloam_ws/devel/setup.bash
   ```

1. **Run the Ouster Driver**

   ```bash
   roslaunch ouster_decoder driver.launch
   ```

2. **Run the Ouster Decoder**

   ```bash
   roslaunch ouster_decoder decoder.launch
   ```

3. **Run LLOL**

   ```bash
   roslaunch llol llol.launch
   ```
   
4. **Launch SLOAM**

   ```bash
   roslaunch sloam run_sim.launch  # Use run.launch for real sensor data
   ```

---
