## Exporting Pretrained Segmentation Models to ONNX

SLOAM uses a segmentation network (e.g., RangeNet++) for tree segmentation. To export a pretrained model to ONNX, follow these steps (e.g., in a [Google Colab notebook](sloam_pkgs/models/onnx_seg_export.ipynb)):

### a. Install ONNX

```python
pip install onnx
```

### b. Download and Extract Pretrained Models

Download pretrained model archives (e.g., squeezesegV2) from the provided links and extract them:

```bash
cd /opt/sloam_ws/src/sloam/sloam_pkgs/models/
wget http://www.ipb.uni-bonn.de/html/projects/bonnetal/lidar/semantic/models/squeezesegV2.tar.gz
tar -xvzf squeezesegV2.tar.gz
```

### c. Clone the Model Repository (if needed)

If required, clone the RangeNet++/LiDAR-Bonnetal repository:

```bash
cd /opt/sloam_ws/src/sloam/sloam_pkgs/models/
git clone https://github.com/PRBonn/lidar-bonnetal.git
```

### d. Export the Model to ONNX

Use a script similar to the following (adjust paths as needed):

```python
import torch, yaml
from tasks.semantic.modules.segmentator import Segmentator

model_path = "/content/squeezesegV2/squeezesegV2"

with open(model_path + "/arch_cfg.yaml", "r") as f:
    ARCH = yaml.safe_load(f)
with open(model_path + "/data_cfg.yaml", "r") as f:
    dataset_cfg = yaml.safe_load(f)

# Determine the number of classes from the learning map
learning_map = dataset_cfg["learning_map"]
n_classes = len(set(learning_map.values()))
print(f"Number of classes: {n_classes}")

# Initialize and evaluate the model
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
---

### e. Integrate the ONNX Model

Move the exported ONNX model into the `models` folder of your workspace and update the SLOAM parameter file (`sloam/params/sloam.yaml`) to point to this model.

---
