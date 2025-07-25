# Low Cost Robot Sim2Real

A simulation environment for low-cost robotic arms using MuJoCo and leRobot for teleoperation and control.

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/TataKKKL/low_cost_robot_sim2real.git
cd low_cost_robot_sim2real
```

### 2. Set Up Conda Environment

```bash
# Create a new conda environment
conda create -n robot_sim python=3.10
conda activate robot_sim

# Install MuJoCo
conda install -c conda-forge mujoco

# Install additional dependencies
pip install draccus rerun numpy
```

### 3. Install leRobot

This project uses [leRobot](https://github.com/huggingface/lerobot) for robot teleoperation and control.

```bash
# Clone leRobot repository
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout aec1b29d230341721c8d8f4413c47dcdb424cf5d

# Install leRobot in development mode
pip install -e .

# Return to project root
cd ..
```

**Note:** The project has been tested with leRobot commit: `aec1b29d230341721c8d8f4413c47dcdb424cf5d`

## 🤖 Supported Robots

### SO-100 Arm
- **Model Path**: `teleoperate_simulation/trs_so_arm100/so_arm100.xml`
- **URDF → MJCF**: Derived from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/trs_so_arm100)

### SO-101 Arm
- **Model Path**: `teleoperate_simulation/SO101/so101_new_calib.xml`
- **Scene Path**: `teleoperate_simulation/SO101/scene.xml` (includes background environment)
- **Source**: [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101)

## 🎮 Usage

### Teleoperation Simulation

**Note:** The command differs between operating systems:
- **Linux**: Use `python -m teleoperate_sim`
- **macOS**: Use `mjpython -m teleoperate_sim`

**Important:** The `--teleop.port` varies by computer. Check your system's USB port:
- **Linux**: Usually `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyUSB0`, etc.
- **macOS**: Usually `/dev/tty.usbmodem*` or `/dev/tty.usbserial*`

Run teleoperation with the SO-100 arm (macOS):

```bash
cd teleoperate_simulation
mjpython -m teleoperate_sim \
  --teleop.type=so100_leader \
  --teleop.port=/dev/tty.usbmodem58FA0927201 \
  --teleop.id=my_awesome_leader_arm \
  --mjcf_path=trs_so_arm100/so_arm100.xml \
  --display_data=true
```

Run teleoperation with the SO-101 arm (Linux):

```bash
cd teleoperate_simulation
python -m teleoperate_sim \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm \
  --mjcf_path=SO101/so101_new_calib.xml \
  --display_data=true
```

Run teleoperation with the SO-101 arm in scene environment (Linux):

```bash
cd teleoperate_simulation
python -m teleoperate_sim \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm \
  --mjcf_path=SO101/scene.xml \
  --display_data=true
```

### Position Testing

Test the SO-100 arm:

```bash
mjpython simple_position_test.py --model_path trs_so_arm100/so_arm100.xml
```

Test the SO-101 arm:

```bash
mjpython simple_position_test.py --model_path SO101/so101_new_calib.xml
mjpython simple_position_test.py --model_path SO101/scene.xml
```

## 📁 Project Structure

```
low_cost_robot_sim2real/
├── teleoperate_simulation/
│   ├── lerobot/                 # leRobot library (submodule)
│   ├── trs_so_arm100/          # SO-100 robot models
│   ├── SO101/                  # SO-101 robot models
│   ├── teleoperate_sim.py      # Main teleoperation script
│   ├── simple_position_test.py # Position testing script
│   └── test.py                 # Additional testing
├── README.md                   # This file
└── LICENSE                     # Project license
```

## 🔧 Configuration

### Calibration Offsets
The system includes calibration offsets for each joint (in degrees):
- `shoulder_pan`: Direction reversed (left/right swapped)
- `shoulder_lift`: Configurable offset
- `elbow_flex`: Configurable offset
- `wrist_flex`: Configurable offset
- `wrist_roll`: Configurable offset
- `gripper`: Configurable offset

### Keyframe Support
The simulation supports MuJoCo keyframes for initial positioning. The robot will start in a compact pose before following teleoperator input.

## 📚 Dependencies

- **Python**: 3.10+
- **MuJoCo**: Physics simulation engine
- **leRobot**: Robot teleoperation and control library
- **draccus**: Configuration management
- **rerun**: Visualization and debugging
- **numpy**: Numerical computations

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with the provided scripts
5. Submit a pull request

## 📄 License

This project is licensed under the terms specified in the LICENSE file.

## 🔗 Links

- [leRobot GitHub Repository](https://github.com/huggingface/lerobot)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)

