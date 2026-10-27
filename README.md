# NSG motion-planning dataset

Dataset snapshot **26.05.21** and C++ dataset-generation programs for neural skeleton-guided manipulator motion planning.

## Download the dataset

The raw CSV files are distributed as losslessly compressed **GitHub Release assets**:

- [Dataset snapshot 26.05.21](https://github.com/2799063570/NSG-dataset/releases/tag/dataset-26.05.21)
- [training_envs.csv.gz](https://github.com/2799063570/NSG-dataset/releases/download/dataset-26.05.21/training_envs.csv.gz)
- Path data: download the ten `training_paths.csv.gz.part01`–`.part10` files from the Release, or use the script below.

After cloning this repository, download, decompress and verify both CSV files with Python 3.8 or later:

```bash
python3 scripts/download_dataset.py
```

The script downloads the parts, verifies each one, reassembles the compressed path archive, and restores both CSV files to `data/dataset/26.05.21/`. Alternatively, download the assets manually, concatenate the ten path parts in numerical order into `training_paths.csv.gz`, and decompress both `.gz` files with a gzip-compatible tool. Raw, compressed and per-part SHA-256 checksums are in [data/SHA256SUMS.txt](data/SHA256SUMS.txt); exact sizes and audited row counts are in [data/dataset-manifest.json](data/dataset-manifest.json).

The raw dataset is approximately **1.21 GB**. Keep enough free disk space for both the compressed downloads and the restored CSVs. The Git repository stores programs, metadata and checksums; the large CSV files are in Releases.

| Snapshot statistic | Value |
|---|---:|
| Unique scene IDs in each CSV | 48,015 |
| Unique `(scene_id, path_id)` paths | 48,015 |
| Obstacle rows | 1,296,405 |
| Path waypoint rows | 18,539,735 |
| Waypoints per raw path | 100–956 |

This is the explicitly supplied `26.05.21` snapshot. Its audited count is 48,015 paths; it should not be represented as a complete 67,725-sample manuscript dataset without identifying and publishing the additional samples.

## CSV format

Both CSV files have **eight columns and no header row**. Scene and path identifiers are zero-based integers.

### `training_envs.csv`

```text
scene_id,shape_type,x,y,z,dimension_1,dimension_2,dimension_3
```

Each row is one obstacle. Positions and dimensions are in metres. The generator uses the following shape encoding:

| Shape type | Meaning | Dimensions |
|---:|---|---|
| 0 | Box | x, y and z size |
| 1 | Sphere | radius; remaining entries are unused |
| 2 | Cylinder | height, radius; remaining entry is unused |

### `training_paths.csv`

```text
scene_id,path_id,q1,q2,q3,q4,q5,q6
```

Rows sharing `(scene_id, path_id)` form an ordered six-joint configuration-space path. Joint positions are in radians. Preserve file order within each path. Join a path to its obstacle rows using `scene_id`; take its first and last configurations as the start and goal.

These files contain primitive obstacle parameters and raw waypoint sequences. They do not directly contain the model's fixed-length waypoint tensors, sampled 2048-point obstacle clouds, trained weights, or benchmark train/validation/test splits. The manifest describes this uploaded snapshot; do not infer a manuscript's sample counts or splits from the folder name alone.

## Generation programs

The three supplied generator sources are copied **without changes**. Their SHA-256 hashes, together with the copied launch resources, are in [docs/source-files.sha256](docs/source-files.sha256).

| Source | Purpose |
|---|---|
| `src/data_generator_path_obs.cpp` | Generate a reference path, then add path-constrained obstacles while retaining path validity |
| `src/data_generator_node.cpp` | Generate random obstacle scenes and plan paths |
| `src/data_generator_app_scene.cpp` | Generate structured application scenes and corresponding paths |

The supplied CMake configuration builds only these generators. It does not require the original project's neural planner, LibTorch model or ROS service plugin.

### Requirements

- A Linux ROS 1 / catkin environment with MoveIt 1, MoveIt Visual Tools, RViz Visual Tools, Eigen3, geometric_shapes and OMPL.
- A C++17 compiler and CMake 3.18 or later.
- An AUBO i5 robot description and semantic robot description with a MoveIt planning group named `arm`.
- The copied launch files expect an external ROS package named `robot_moveit_config`, providing `launch/planning_context.launch`. Supply that package or adapt the launch include to your own robot configuration.

Robot URDF/SRDF/mesh assets are **not included** in this dataset repository. No new ROS build or generation run was performed as part of this publication from the Windows upload environment.

### Build

The ROS package name remains `motion_planning` so that the supplied launch files keep working. Do not put it alongside another package with the same name in one catkin workspace.

```bash
mkdir -p ~/nsg_catkin_ws/src
cd ~/nsg_catkin_ws/src
git clone https://github.com/2799063570/NSG-dataset.git motion_planning
cd ~/nsg_catkin_ws
catkin_make
source devel/setup.bash
```

Ensure the required robot configuration package is also available in the ROS environment.

### Generate path-constrained scenes

Use a small run first; this writes a new dataset rather than modifying the downloaded release:

```bash
roslaunch motion_planning run_generator_path.launch \
  debug:=false num_base_paths:=10 variants_per_path:=5 \
  output_dir:="$HOME/nsg_generated/path_obs" overwrite_output:=false
```

The source validates paths after smoothing and after adding obstacles. It can append to an existing output dataset and continue scene IDs. The supplied launch defaults are in `launch/run_generator_path.launch` and differ from some fallback defaults compiled into the C++ node.

### Generate structured application scenes

```bash
roslaunch motion_planning run_generator_app_scene.launch \
  debug:=false total_scenes:=10 paths_per_scene:=2 \
  output_dir:="$HOME/nsg_generated/application" overwrite_output:=false
```

### Legacy random-scene generator

```bash
mkdir -p "$HOME/aubo/arm_ws/src/motion_planning/data"
roslaunch motion_planning run_generator.launch debug:=false
```

The unchanged legacy `data_generator_node.cpp` uses that hard-coded output directory and opens `training_envs.csv` and `training_paths.csv` in truncation mode. Run it with a separate empty output location or edit its output path before use. It does not read the newer generators' `output_dir` parameter.

## Reproducibility and scope

The release preserves the author-supplied CSV bytes: gzip decompression was checked against the raw SHA-256 hashes. The CSV audit scans all row widths and integer scene/path keys. This verifies packaging and structure, not geometric collision validity.

The generator files are the supplied source snapshot at publication time. This does not establish that this exact code revision and current launch defaults generated the archived CSV files. Random seeds, robot configuration, dependency versions and post-processing can affect regenerated output; bit-for-bit regeneration of the archived dataset is not claimed.

No repository-wide code or data licence has been selected by the authors in this release. Package-maintainer metadata was inherited from the original package and remains a placeholder. Do not infer a particular reuse licence solely from public accessibility.

## 中文说明

本仓库提供指定 `26.05.21` 快照中的两个原始 CSV 文件，以及三种数据集生成程序。大文件位于上方 Release 下载区，Git 仓库内提供程序、启动文件、数据格式说明和校验值。

运行 `python scripts/download_dataset.py` 可下载、解压并验证两个文件。数据是原始路径点和障碍物参数，不等同于已经生成点云、固定长度骨架、训练划分或模型权重的完整训练缓存。生成程序依赖 ROS 1 / MoveIt 和外部机器人描述；公开源文件未改动。
