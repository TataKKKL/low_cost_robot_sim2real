# Teleoperate Simulation

## SO-100

### URDF → MJCF derivation steps
https://github.com/google-deepmind/mujoco_menagerie/tree/main/trs_so_arm100



mjpython -m teleoperate_sim \
  --teleop.type=so100_leader \
  --teleop.port=/dev/tty.usbmodem58FA0927201 \
  --teleop.id=my_awesome_leader_arm \
  --mjcf_path=trs_so_arm100/so_arm100.xml \
  --display_data=true


## SO-101
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101