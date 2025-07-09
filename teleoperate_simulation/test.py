"""
Simulated teleoperation: reads teleoperator hardware input and uses it to control a Mujoco simulation.
No physical robot is connected or commanded. Use this to control a robot in simulation.

Example usage:

mjpython -m lerobot.teleoperate_sim \
  --teleop.type=so100_leader \
  --teleop.port=/dev/tty.usbmodemXXXX \
  --teleop.id=my_leader \
  --mjcf_path=path/to/your_robot.xml \
  --display_data=true
"""

import time
import logging
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
import rerun as rr
import mujoco
import mujoco.viewer
from lerobot.robots import RobotConfig
from lerobot.teleoperators import (
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
)
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import _init_rerun
import numpy as np

from lerobot.teleoperators import koch_leader, so100_leader, so101_leader  # noqa: F401

@dataclass
class TeleoperateSimConfig:
    teleop: TeleoperatorConfig
    mjcf_path: str
    fps: int = 10
    display_data: bool = False

@draccus.wrap()
def teleoperate_sim(cfg: TeleoperateSimConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        _init_rerun(session_name="teleoperation_sim")

    # Load Mujoco model
    model = mujoco.MjModel.from_xml_path(cfg.mjcf_path)
    data = mujoco.MjData(model)

    # Map Mujoco joint names to indices
    mujoco_joint_names = [model.joint(i).name for i in range(model.njnt)]
    print("Mujoco joint names:", mujoco_joint_names)
    mujoco_indices = [0,1,2,3,4,5]

    # 🔧 Calibration offsets for each joint (in degrees)
    # Order: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
    calibration_offsets = [0, 0, 0, 0, 0, 0]
    
    print("🔧 Calibration offsets applied:")
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    for i, (name, offset) in enumerate(zip(joint_names, calibration_offsets)):
        if offset != 0:
            print(f"  Joint {i} ({name}): {offset:+.1f}°")
    
    print("🔄 Direction modifications:")
    print("  Joint 0 (shoulder_pan): Direction REVERSED (left/right swapped)")

    # 🔧 Load keyframe before teleoperation starts
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)  # Load first keyframe ("home")
        print(f"✅ Loaded keyframe 'home' position")
        print("🎯 Robot will start in compact pose, then follow teleoperator")
        
        # Show current keyframe positions
        print("📍 Keyframe joint positions:")
        for i in range(model.njnt):
            joint_name = model.joint(i).name
            position = data.qpos[i]
            print(f"  Joint {i} ({joint_name}): {position:.4f} rad = {np.rad2deg(position):.1f}°")
    else:
        print("⚠️ No keyframes found in XML - using defaults")

    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop.connect()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n🚀 Teleoperation started!")
            print("💡 Robot started in keyframe position and will now follow teleoperator input")
            print("🔄 Restart the script to return to keyframe position")
            
            frame_count = 0
            while viewer.is_running():
                action = teleop.get_action()
                
                # Map the first 6 teleop joint values (in order) to Mujoco joints "1"-"6"
                joint_values = list(action.values())[:6]
                
                # 🔄 Reverse shoulder_pan direction (swap left/right)
                # joint_values[0] = -joint_values[0]
                
                # 🔧 Apply calibration offsets (in degrees, before converting to radians)
                raw_values = joint_values.copy()  # Keep original for debug (after direction reversal)
                for i in range(len(joint_values)):
                    joint_values[i] += calibration_offsets[i]
                
                # Convert from degrees to radians before sending to Mujoco
                joint_values = np.deg2rad(joint_values)
                
                for idx, val in zip(mujoco_indices, joint_values):
                    data.qpos[idx] = val
                
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Debug output every 60 frames (1 second at 60fps)
                if cfg.display_data and frame_count % 60 == 0:
                    print(f"\n--- Frame {frame_count} ---")
                    print("🔄 Joint values (Teleop → Reversed → Calibrated → Radians):")
                    original_values = list(action.values())[:6]
                    for i in range(len(raw_values)):
                        original = original_values[i]
                        reversed_val = raw_values[i]
                        offset = calibration_offsets[i]
                        calibrated = reversed_val + offset
                        radians = joint_values[i]
                        if i == 0:  # shoulder_pan
                            print(f"  Joint {i}: {original:.1f}° → {reversed_val:.1f}° (reversed) + {offset:+.1f}° = {calibrated:.1f}° → {radians:.4f} rad")
                        elif offset != 0:
                            print(f"  Joint {i}: {original:.1f}° + {offset:+.1f}° = {calibrated:.1f}° → {radians:.4f} rad")
                        else:
                            print(f"  Joint {i}: {original:.1f}° → {radians:.4f} rad")
                
                if cfg.display_data:
                    for i, val in enumerate(joint_values, 1):
                        rr.log(f"action_{i}", rr.Scalars(val))
                    if frame_count % 60 != 0:  # Don't duplicate debug output
                        print("Simulated Joint States (action):")
                        for i, v in enumerate(joint_values, 1):
                            print(f"  {i}: {v:.4f}")
                        print("-" * 40)
                
                frame_count += 1
                time.sleep(1.0 / cfg.fps)
                
    except KeyboardInterrupt:
        print("\n🛑 Teleoperation interrupted by user")
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        print("👋 Teleoperation session ended")

if __name__ == "__main__":
    teleoperate_sim()