# ROS2 PCA9685

A modular ROS 2 package that contains a node for interacting with a PCA9685 PWM board. 

This package is designed to be built in a standalone ROS 2 workspace or included as a subrepository (subrepo) in larger projects.

## Features
- Contains a ROS 2 C++ node.
- Easily integrated as a subrepo in a parent workspace.
- Uses standard ROS 2 tools (colcon, rosdep) for building and dependency management.

## Prerequisites

- ROS 2 (e.g., Foxy, Humble, or later) installed and sourced.
- colcon build tool.
- rosdep for dependency management.

## Installation & Build

### Standalone Installation
Clone the repository:

```bash
git clone <repo-url>
```

Create and set up your ROS 2 workspace:

```bash
mkdir -p ~/<your_workspace>/src
cd ~/<your_workspace>/src
git clone <repo-url> 
```

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
Source the setup file:
```

```bash
source ~/<your_workspace>/install/setup.bash
```

Run your node:

```bash
ros2 run pca9685 pca9685_node
```

## Including as a Subrepository (Subrepo)

To include this package in a parent project:

Add as a submodule:

```bash
cd ~/<your_workspace>/src
git submodule add <your-repo-url>  pca9685
```

Build the parent workspace:

```bash
cd ~/<your_workspace>
colcon build --symlink-install
source install/setup.bash
```

Run your node as usual with a Bus and Address parameter:

```bash
ros2 run pca9685 pca9685_node --ros-args -p bus:=1 -p address:=65
```

## Usage

After building and sourcing your workspace, run your node using:

```bash
ros2 run pca9685 pca9685_node
```

### Parameters

The PCA9685 node accepts the following parameters:

- `bus` (int, default: 1): I2C bus number
- `address` (int, default: 0x40): I2C device address in hexadecimal
- `frequency` (int, default: 60): PWM frequency in Hz
- `enabled_channels` (vector<bool>, default: all true): List of 16 boolean values indicating which channels to enable

### Channel Configuration

The `enabled_channels` parameter allows you to specify which of the 16 PWM channels should be initialized. This is useful for:
- Reducing unnecessary topic subscriptions
- Preventing accidental control of unused channels
- Optimizing resource usage

Example configuration file (`config/pca9685_example.yaml`):

```yaml
pca9685_node:
  ros__parameters:
    bus: 1
    address: 0x40
    frequency: 60
    enabled_channels: [
      true,   # Channel 0  - enabled (e.g., steering servo)
      true,   # Channel 1  - enabled (e.g., throttle ESC)
      false,  # Channel 2  - disabled
      false,  # Channel 3  - disabled
      # ... remaining channels
      false   # Channel 15 - disabled
    ]
```

Run with configuration:

```bash
ros2 run pca9685 pca9685_node --ros-args --params-file src/pca9685/config/pca9685_example.yaml
```

### Topics

For each enabled channel `N`, the node subscribes to:
- `/pwm_channel_N` (std_msgs/Int32): PWM pulse width value (0-4095)

## License

This project is licensed under the Apache License 2.0.

## Maintainers

[Andrew Johnson](https://github.com/anjrew) – Maintainer – andrewmjohnson549@gmail.com

## Contributing

Contributions are welcome via PR!
