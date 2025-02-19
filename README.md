
---

# SLOAM & LLOL

**SLOAM** (Semantic Lidar Odometry and Mapping in Forests) is a ROS framework that fuses LiDAR data with semantic segmentation for forest mapping. **LLOL** (Low-Latency Odometry for Spinning Lidars) provides a low-latency odometry backbone for spinning LiDAR sensors.

> **Note:** We recommend running SLOAM inside Docker. However, a local installation is also supported.

---

## Workspace Structure

Create a workspace with the following directory structure:

```
sloam_ws/
 ├── src/
 │    ├── sloam           (clone from https://github.com/KumarRobotics/sloam)
 │    ├── sloam_msgs      (clone from https://github.com/KumarRobotics/sloam)
 │    ├── ouster_decoder  (clone from https://github.com/KumarRobotics/ouster_decoder)
 │    ├── ouster_ros      (clone from https://github.com/ouster-lidar/ouster-ros)
 │    ├── llol            (clone from https://github.com/versatran01/llol)
 │    └── models/         (create this folder and add segmentation models in ONNX format)
```

---

## Installation & Dependencies

### System & ROS Packages (Ubuntu 20.04 / ROS Noetic)

```bash
sudo apt-get update
sudo apt-get install -y g++ libeigen3-dev git python3-catkin-tools \
    ros-noetic-pcl-ros ros-noetic-rviz build-essential \
    libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev libpcap-dev cmake
```

Run rosdep:

```bash
rosdep update
rosdep install --from-paths . --ignore-src -y -r --as-root apt:false
```

### External Libraries

Install these libraries either via apt or build from source as follows:

#### glog

```bash
cd /tmp
git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake -S .. -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17
cmake --build build
sudo cmake --build build --target install
```

#### fmt (Ensure PIC is enabled)

Either install via apt:
```bash
sudo apt-get install libfmt-dev
```
*or* build from source:

```bash
cd /tmp
git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git
cd fmt
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE \
      -DCMAKE_CXX_STANDARD=17 -DFMT_TEST=False ..
make -j$(nproc)
sudo make install
```

#### Abseil

```bash
cd /tmp
git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake -DABSL_BUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
sudo cmake --build . --target install
```

Verify that `abslConfig.cmake` is located (e.g., `/usr/lib/x86_64-linux-gnu/cmake/absl/abslConfig.cmake`).

#### Sophus

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

## Building the Workspace

1. **Clone Repositories:**

   ```bash
   mkdir -p ~/sloam_ws/src && cd ~/sloam_ws/src
   git clone https://github.com/KumarRobotics/sloam.git
   git clone https://github.com/KumarRobotics/ouster_decoder.git
   git clone https://github.com/ouster-lidar/ouster-ros.git
   git clone https://github.com/versatran01/llol.git
   mkdir models
   ```

2. **Update llol CMakeLists.txt:**

   In `llol/CMakeLists.txt`, add:
   ```cmake
   include_directories(${CMAKE_CURRENT_SOURCE_DIR})
   ```
   This ensures that headers (e.g., `sv/util/nlls.h`) are found.

3. **Build with catkin_make:**

   Clean and build the workspace, specifying dependency paths:
   ```bash
   cd ~/sloam_ws
   rm -rf build devel
   catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release \
     -Dabsl_DIR=/usr/lib/x86_64-linux-gnu/cmake/absl \
     -Dbenchmark_DIR=/usr/lib/x86_64-linux-gnu/cmake/benchmark \
     -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
     -DBUILD_BENCHMARK=OFF
   ```

   *Note:* `-DBUILD_BENCHMARK=OFF` disables benchmark executables that lack a `main()`.

---

## Exporting Pretrained Segmentation Models to ONNX

SLOAM uses a segmentation network (e.g., RangeNet++) for tree segmentation. Follow these steps (e.g., in Google Colab):

1. **Install ONNX:**

   ```python
   !pip install onnx
   ```

2. **Download and Extract a Pretrained Model:**

   ```bash
   !wget http://www.ipb.uni-bonn.de/html/projects/bonnetal/lidar/semantic/models/squeezesegV2.tar.gz
   !tar -xvzf squeezesegV2.tar.gz
   ```

3. **Export the Model to ONNX:**

   Use a script like the following (adjust paths as needed):

   ```python
   import torch, yaml
   from tasks.semantic.modules.segmentator import Segmentator

   model_path = "/content/squeezesegV2/squeezesegV2"
   with open(model_path + "/arch_cfg.yaml", "r") as f:
       ARCH = yaml.safe_load(f)
   with open(model_path + "/data_cfg.yaml", "r") as f:
       dataset_cfg = yaml.safe_load(f)

   learning_map = dataset_cfg["learning_map"]
   n_classes = len(set(learning_map.values()))
   print(f"Number of classes: {n_classes}")

   model = Segmentator(ARCH, n_classes, path=model_path)
   model.eval()

   height = ARCH["dataset"]["sensor"]["img_prop"]["height"]
   width = ARCH["dataset"]["sensor"]["img_prop"]["width"]
   dummy_input = torch.randn(1, 5, height, width)
   dummy_mask = torch.ones(1, height, width)

   onnx_path = "/content/squeezesegV2_segmentator.onnx"
   torch.onnx.export(
       model,
       (dummy_input, dummy_mask),
       onnx_path,
       export_params=True,
       opset_version=11,
       do_constant_folding=True,
       input_names=["input", "mask"],
       output_names=["output"],
       dynamic_axes={"input": {0: "batch_size"}, "mask": {0: "batch_size"}, "output": {0: "batch_size"}}
   )
   print(f"Model exported to {onnx_path}")
   ```

4. **Integrate the ONNX Model:**

   Move the exported ONNX file to the `models` folder and update the SLOAM parameter file (`sloam/params/sloam.yaml`) with the new model path.

---

## Running SLOAM

### Docker-Based Run

1. **Build the Docker Image:**

   ```bash
   ./docker/build_sloam_image.sh
   ```

2. **Configure the Run Script:**

   Edit `sloam/docker/run_sloam_container.sh` and set:
   ```bash
   SLOAMWS="$HOME/ros/sloam_ws"
   BAGS_DIR="$HOME/bags"
   ```

3. **Run the Container:**

   ```bash
   ./docker/run_sloam_container.sh
   ```

4. **Inside the Container:**

   Verify volume mapping:
   ```bash
   cd /opt/sloam_ws && ls src
   ```
   Launch SLOAM (for simulated data):
   ```bash
   tmux
   source devel/setup.bash
   roslaunch sloam run_sim.launch
   ```
   In another tmux pane, play a bag file:
   ```bash
   cd ../bags
   rosbag play example.bag
   ```

### Local Installation

1. **Source the Workspace:**

   ```bash
   source ~/sloam_ws/devel/setup.bash
   ```

2. **Launch SLOAM:**

   ```bash
   roslaunch sloam run_sim.launch   # Use run.launch for live sensor data.
   ```

---

## Running LLOL for Odometry

LLOL provides the odometry backbone.

1. **Run the Ouster Driver:**

   ```bash
   roslaunch ouster_decoder driver.launch
   ```

2. **Run the Ouster Decoder:**

   ```bash
   roslaunch ouster_decoder decoder.launch
   ```

3. **Run LLOL:**

   ```bash
   roslaunch llol llol.launch
   ```
   For multithreaded mode with timing output every 5 seconds:
   ```bash
   roslaunch llol llol.launch tbb:=1 log:=5
   ```

4. **Replay a Bag File (if needed):**

   ```bash
   rosbag play <path_to_bag_file>
   ```

   Visualize odometry using the provided RViz config (`llol/launch/llol.rviz`).

---

## Parameter Tuning & Development

- SLOAM parameters are primarily defined in the launch files (e.g., `sloam/launch/sloam.launch`, `run.launch`, `run_sim.launch`).
- Adjust topics, sensor settings, and mapping parameters as needed.
- For development, use VSCode with the following recommended extensions:
  - ROS
  - Docker
  - Remote-Containers

A sample VSCode debug configuration:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Node SLOAM",
      "request": "launch",
      "target": "/opt/sloam_ws/src/sloam/launch/run_sim.launch",
      "type": "ros"
    }
  ]
}
```

---

## Citations

If you use this work in your academic publications, please cite:

### SLOAM

```bibtex
@inproceedings{chen2019,
  title={SLOAM: Semantic lidar odometry and mapping for forest inventory},
  author={Chen, Steven W and Nardari, Guilherme V and Lee, Elijah S and Qu, Chao and Liu, Xu and Romero, Roseli Ap Francelin and Kumar, Vijay},
  year={2020}
}
```

```bibtex
@inproceedings{liu2022large,
  title={Large-scale Autonomous Flight with Real-time Semantic SLAM under Dense Forest Canopy},
  author={Liu, Xu and Nardari, Guilherme V and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W and Romero, Roseli AF and Taylor, Camillo J and others},
  year={2022}
}
```

### LLOL

```bibtex
@misc{qu2021llol,
  title={LLOL: Low-Latency Odometry for Spinning Lidars},
  author={Chao Qu and Shreyas S. Shivakumar and Wenxin Liu and Camillo J. Taylor},
  note={https://arxiv.org/abs/2110.01725},
  year={2021}
}
```

---

This README provides a complete guide for setting up, building, and running the SLOAM and LLOL systems on Ubuntu 20.04 with ROS Noetic. Adjust file paths and parameters as necessary for your environment.

---
