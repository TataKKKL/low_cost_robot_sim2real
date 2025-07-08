"""
Simple script to load MuJoCo robot and set it to a specific position.
No teleoperation - just position testing.

Usage:
mjpython simple_position_test.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # Load the robot model
    model_path = "trs_so_arm100/so_arm100.xml"  # Adjust path if needed
    print(f"📂 Loading model from: {model_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("✅ Model loaded successfully!")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return

    # 🔍 DEBUG: Show what positions are loaded by default
    print("\n🔍 Default joint positions from XML:")
    joint_names = [model.joint(i).name for i in range(model.njnt)]
    print("Available joints:", joint_names)
    
    for i in range(model.njnt):
        joint_name = model.joint(i).name
        position = data.qpos[i]
        print(f"  Joint {i} ({joint_name}): {position:.4f}")
    
    # 🔍 Check if model has keyframes
    print(f"\n🔍 Model has {model.nkey} keyframes")
    if model.nkey > 0:
        print("Available keyframes:")
        for k in range(model.nkey):
            keyframe_name = model.key(k).name if hasattr(model.key(k), 'name') else f"keyframe_{k}"
            print(f"  Keyframe {k}: {keyframe_name}")
    
    # 💡 OPTION 1: Use XML defaults (current behavior) - ALL ZEROS
    # print("\n💡 Using XML default positions...")
    # No position changes - just use whatever is in the XML file
    # This is what happens when you create MjData(model) - it loads XML defaults
    # XML defaults are all 0.0000 = extended/straight pose
    
    # 💡 OPTION 3: Load from keyframe to get compact position (ACTIVE)
    if model.nkey > 0:
        keyframe_index = 0  # Use first keyframe ("home")
        # Use proper MuJoCo function to load keyframe
        mujoco.mj_resetDataKeyframe(model, data, keyframe_index)
        print(f"\n💡 Loaded 'home' keyframe position using mj_resetDataKeyframe:")
        print(f"🎯 Keyframe values: [0, -1.57079, 1.57079, 1.57079, -1.57079, 0]")
        print(f"🎯 This creates the COMPACT/FOLDED pose!")
    else:
        print("\n💡 No keyframes found - using XML defaults (all zeros)")

    # Show final positions (same as initial since we're not changing anything)
    print("\n🎯 Final joint positions:")
    for i in range(model.njnt):
        joint_name = model.joint(i).name
        position = data.qpos[i]
        print(f"  Joint {i} ({joint_name}): {position:.4f}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\n👀 Viewer launched. Robot is in XML default position!")
        print("💡 You can use the viewer's Reset button to return to this position")
        
        # Just keep the simulation running so you can see the position
        while viewer.is_running():
            # No control input - just hold the position
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)  # 100 Hz

    print("👋 Viewer closed")

if __name__ == "__main__":
    main()


# # Old (incorrect) way
# data.qpos[:] = model.key_qpos[...]  # Only copied positions
# mujoco.mj_forward(model, data)      # Didn't fully initialize state

# # New (correct) way  
# mujoco.mj_resetDataKeyframe(model, data, 0)  # Loads complete keyframe state

## step 1, load the model, and the keyframe correctly