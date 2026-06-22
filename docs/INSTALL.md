# Installation Guide

## Prerequisites

- **Docker** with NVIDIA Container Toolkit
- **NVIDIA drivers** installed on the host
- **Git**
- **xhost** (for GUI forwarding - usually pre-installed on Ubuntu)

---

## 1. Clone the repository

```bash
git clone <repo-url>
cd rbsf-racing-irobocity
```

---

## 2. Initialize submodules

The workspace uses nested git submodules. Run the recursive init to populate all of them
(including `ackermann_mux`, `vesc`, and `teleop_tools` inside `f1tenth_system`):

```bash
git submodule update --init --recursive
```

> **Why recursive?** `f1tenth_system` is itself a collection of submodules. A plain
> `git submodule update --init` only initializes the top-level entry and leaves its
> children empty, so packages like `ackermann_mux` won't be found by colcon.

---

## 3. Build the Docker image

```bash
make build
```

This builds the image tagged `rbsf_racing_irobocity:v0.0.1` with your current user's
UID/GID baked in to avoid volume permission issues.

---

## 4. Run the container

```bash
make run
```

The entrypoint (`entrypoint.sh`) runs automatically and:

1. Sources `/opt/ros/humble/setup.bash`
2. Runs `colcon build` to compile the workspace
3. Sources `install/local_setup.bash`
4. Checks for a `.venv` and runs `uv sync` if needed

The workspace directory (`./`) is mounted at `/ros2_ws` inside the container, so any
changes you make on the host are immediately visible without rebuilding the image.

---

## Attaching a second terminal

To open a new shell in a running container:

```bash
make attach
```
