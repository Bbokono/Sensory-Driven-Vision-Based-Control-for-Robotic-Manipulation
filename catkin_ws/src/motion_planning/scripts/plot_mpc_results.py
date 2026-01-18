#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def plot_latest():
    if len(sys.argv) > 1:
        latest_file = sys.argv[1]
    else:
        # Find the most recent CSV file in the workspace or current dir
        search_paths = ["/root/catkin_ws/mpc_results_*.csv", "mpc_results_*.csv"]
        list_of_files = []
        for p in search_paths:
            list_of_files.extend(glob.glob(p))

        if not list_of_files:
            print("No MPC result files found.")
            return

        latest_file = max(list_of_files, key=os.path.getctime)

    print(f"Processing data from: {latest_file}")

    df = pd.read_csv(latest_file)

    # Normalize time to start at 0
    df["time"] = df["time"] - df["time"].iloc[0]

    # Create a 3x2 grid of plots
    fig, axs = plt.subplots(3, 2, figsize=(16, 18))
    fig.suptitle(
        f"MPC Controller Performance Analysis\n{os.path.basename(latest_file)}",
        fontsize=16,
    )

    # 1. Position Tracking (X, Y, Z)
    axs[0, 0].plot(df["time"], df["target_x"], "r--", alpha=0.5, label="Target X")
    axs[0, 0].plot(df["time"], df["actual_x"], "r", label="Actual X")
    axs[0, 0].plot(df["time"], df["target_y"], "g--", alpha=0.5, label="Target Y")
    axs[0, 0].plot(df["time"], df["actual_y"], "g", label="Actual Y")
    axs[0, 0].plot(df["time"], df["target_z"], "b--", alpha=0.5, label="Target Z")
    axs[0, 0].plot(df["time"], df["actual_z"], "b", label="Actual Z")
    axs[0, 0].set_title("Position Tracking")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # 2. Error Convergence (Log Scale)
    axs[0, 1].plot(
        df["time"], df["error_dist"], label="Position Error (m)", linewidth=2
    )
    if "error_orient" in df.columns:
        axs[0, 1].plot(
            df["time"], df["error_orient"], label="Orientation Error", linewidth=2
        )
    axs[0, 1].set_title("Error Convergence")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Error Magnitude")
    axs[0, 1].set_yscale("log")  # Log scale highlights convergence
    axs[0, 1].legend()
    axs[0, 1].grid(True, which="both", ls="-", alpha=0.5)

    # 3. Control Inputs (Joint Velocities)
    for i in range(1, 7):
        if f"v{i}" in df.columns:
            axs[1, 0].plot(df["time"], df[f"v{i}"], label=f"Joint {i}")
    axs[1, 0].set_title("Control Effort (Joint Velocities)")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Velocity (rad/s)")
    axs[1, 0].legend(loc="upper right", fontsize="x-small", ncol=2)
    axs[1, 0].grid(True)

    # 4. Computation Time Histogram
    axs[1, 1].hist(df["computation_time"] * 1000, bins=50, color="purple", alpha=0.7)
    axs[1, 1].set_title(
        f"Computation Time (Mean: {df['computation_time'].mean()*1000:.2f}ms)"
    )
    axs[1, 1].set_xlabel("Time (ms)")
    axs[1, 1].set_ylabel("Count")
    axs[1, 1].grid(True)

    # 5. Cartesian Velocity Profile
    dt = df["time"].diff().replace(0, np.nan)
    vx = df["actual_x"].diff() / dt
    vy = df["actual_y"].diff() / dt
    vz = df["actual_z"].diff() / dt
    v_cart = np.sqrt(vx**2 + vy**2 + vz**2)

    axs[2, 0].plot(df["time"], v_cart, color="orange")
    axs[2, 0].set_title("End-Effector Cartesian Speed")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Speed (m/s)")
    axs[2, 0].grid(True)

    # 6. 3D Trajectory
    axs[2, 1].remove()
    ax_3d = fig.add_subplot(3, 2, 6, projection="3d")
    ax_3d.plot(df["target_x"], df["target_y"], df["target_z"], "r--", label="Target")
    ax_3d.plot(df["actual_x"], df["actual_y"], df["actual_z"], "b-", label="Actual")
    ax_3d.set_title("3D Trajectory")
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    output_file = os.path.splitext(latest_file)[0] + ".png"
    plt.savefig(output_file)
    print(f"Plot saved to: {output_file}")


if __name__ == "__main__":
    plot_latest()
