# Contributing to SIL

Thanks for your interest in improving SIL! This document explains how to get a
development environment running and the conventions we follow.

## Development setup

```bash
# 1. Clone into a catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/linusnep/SIL.git sil_ros

# 2. Python dependencies
cd sil_ros
pip install -r requirements.txt

# 3. Build
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## Project conventions

- Every tunable parameter are in
  `config/sil_config.yaml` and is read through `sil_ros/config.py`
  (`SILConfig` / `get_param`). If you add a tunable quantity, add a key to the
  YAML with an inline comment.
- **Keep ROS optional.** The neural / co-adaptation classes accept
  explicit constructor arguments so they can be unit-tested without a ROS master
  (`SILConfig.load()` falls back to read the YAML directly).

## Before opening a pull request

```bash
# Byte-compile everything
python -m compileall src scripts

# Verify every get_param key exists in the YAML
python tools/check_config_keys.py
```

Please describe what you changed and, for algorithmic changes, point to the
relevant section/equation of the paper.

## Reporting issues

Open a GitHub issue with: ROS distribution, Python version, the command you ran,
and the full error output.
