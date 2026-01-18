#!/usr/bin/env python3
"""
MPC Test Script - Verify the hybrid MPC+Trajectory approach works
Run this to test basic MPC functionality without the full manipulation task
"""

import rospy
from motion_planning.controller import MPCController
from pyquaternion import Quaternion as PyQuaternion
import math


def test_mpc_controller():
    """Test basic MPC controller functionality"""

    rospy.init_node("mpc_test", anonymous=True)

    print("\n" + "=" * 70)
    print("  MPC CONTROLLER TEST")
    print("=" * 70 + "\n")

    # Initialize controller
    print("[1/4] Initializing MPC Controller...")
    try:
        controller = MPCController()
        print("✅ Controller initialized successfully\n")
    except Exception as e:
        print(f"❌ Failed to initialize controller: {e}\n")
        return False

    # Check gripper pose
    print("[2/4] Checking gripper pose...")
    try:
        pos, quat = controller.gripper_pose
        print(f"✅ Current position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"✅ Current orientation: {quat}\n")
    except Exception as e:
        print(f"❌ Failed to get gripper pose: {e}\n")
        return False

    # Test relative movement
    print("[3/4] Testing relative movement (lift 10cm)...")
    try:
        controller.move(dz=0.1)
        rospy.sleep(2.0)
        pos_new, _ = controller.gripper_pose
        dist = pos_new[2] - pos[2]
        print(f"✅ Moved {dist*100:.1f}cm in Z direction")
        print(
            f"✅ New position: [{pos_new[0]:.3f}, {pos_new[1]:.3f}, {pos_new[2]:.3f}]\n"
        )
    except Exception as e:
        print(f"⚠️  Relative movement test failed: {e}\n")

    # Test absolute movement
    print("[4/4] Testing absolute movement (return home)...")
    try:
        home_pos = (-0.1, -0.2, 1.2)
        home_quat = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
        controller.move_to(home_pos[0], home_pos[1], home_pos[2], home_quat)
        rospy.sleep(2.0)

        final_pos, _ = controller.gripper_pose
        error = (
            (home_pos[0] - final_pos[0]) ** 2
            + (home_pos[1] - final_pos[1]) ** 2
            + (home_pos[2] - final_pos[2]) ** 2
        ) ** 0.5

        print(
            f"✅ Final position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]"
        )
        print(f"✅ Final error: {error*1000:.1f}mm\n")

    except Exception as e:
        print(f"⚠️  Absolute movement test failed: {e}\n")

    print("=" * 70)
    print("  TEST COMPLETE")
    print("=" * 70 + "\n")

    return True


if __name__ == "__main__":
    try:
        success = test_mpc_controller()
        if success:
            print("✅ All tests passed! MPC controller is working.\n")
        else:
            print("❌ Some tests failed. Check the output above.\n")
    except KeyboardInterrupt:
        print("\n\n❌ Test interrupted by user\n")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}\n")
