#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


def plot_latest():
    # Find the most recent CSV file in the workspace
    # Updated to find performance_results or mpc_results
    # Check default container path and relative path for host execution
    possible_paths = [
        "/root/catkin_ws",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")),
    ]
    list_of_files = []
    for p in possible_paths:
        if os.path.isdir(p):
            list_of_files.extend(glob.glob(os.path.join(p, "*_results_*.csv")))

    if not list_of_files:
        print(f"No result files found in checked paths: {possible_paths}")
        return

    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Processing data from: {latest_file}")

    df = pd.read_csv(latest_file)

    # Normalize time to start at 0
    df["time"] = df["time"] - df["time"].iloc[0]
    base_name = os.path.splitext(latest_file)[0]

    # ==========================================
    # 1. Joint Velocities Plot (Saved separately)
    # ==========================================
    if "v1" in df.columns:
        print("Generating Joint Velocities plot...")
        plt.figure(figsize=(12, 6))
        for i in range(1, 7):
            col = f"v{i}"
            if col in df.columns:
                plt.plot(df["time"], df[col], label=f"Joint {i}")

        plt.title(f"Joint Velocities\n{os.path.basename(latest_file)}")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.legend(loc="upper right", ncol=6)
        plt.grid(True)

        out_joints = f"{base_name}_joint_velocities.png"
        plt.savefig(out_joints)
        print(f"Saved: {out_joints}")
        plt.close()
    else:
        print("No joint velocity data found in CSV.")

    # ==========================================
    # 2. RMSE / MAE Statistics Plot (Saved separately)
    # ==========================================
    print("Generating RMSE/MAE plot...")
    plt.figure(figsize=(8, 6))

    mae = df["error_dist"].mean()
    rmse = np.sqrt((df["error_dist"] ** 2).mean())

    metrics = ["MAE", "RMSE"]
    values = [mae, rmse]

    bars = plt.bar(metrics, values, color=["orange", "red"], alpha=0.7)
    plt.title(f"Error Statistics (Position)\n{os.path.basename(latest_file)}")
    plt.ylabel("Meters")
    plt.grid(axis="y", linestyle="--", alpha=0.7)

    # Add labels
    for bar in bars:
        height = bar.get_height()
        plt.text(
            bar.get_x() + bar.get_width() / 2.0,
            height,
            f"{height:.5f} m",
            ha="center",
            va="bottom",
        )

    out_stats = f"{base_name}_error_stats.png"
    plt.savefig(out_stats)
    print(f"Saved: {out_stats}")
    plt.close()

    # ==========================================
    # 3. General Performance Summary (Saved separately)
    # ==========================================
    print("Generating Summary plot...")
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(
        f"Robot Controller Performance Summary\n{os.path.basename(latest_file)}",
        fontsize=16,
    )

    # A. Position Tracking
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

    # B. Error Convergence
    axs[0, 1].plot(df["time"], df["error_dist"], label="Position Error", color="k")
    axs[0, 1].set_title("Position Error over Time")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Error (m)")
    axs[0, 1].set_yscale("log")
    axs[0, 1].grid(True, which="both", ls="-", alpha=0.3)

    # C. End-Effector Velocity Profile
    dt = df["time"].diff().replace(0, np.nan)
    vx = df["actual_x"].diff() / dt
    vy = df["actual_y"].diff() / dt
    vz = df["actual_z"].diff() / dt
    ee_velocity = np.sqrt(vx**2 + vy**2 + vz**2)

    axs[1, 0].plot(df["time"], ee_velocity, label="EE Velocity", color="purple")
    axs[1, 0].set_title("End-Effector Cartesian Velocity")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Speed (m/s)")
    axs[1, 0].grid(True)

    # D. 3D Trajectory
    axs[1, 1].remove()
    ax_3d = fig.add_subplot(2, 2, 4, projection="3d")
    ax_3d.plot(df["target_x"], df["target_y"], df["target_z"], "r--", label="Reference")
    ax_3d.plot(df["actual_x"], df["actual_y"], df["actual_z"], "b-", label="Actual")
    ax_3d.set_title("3D Trajectory")
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.legend()

    out_summary = f"{base_name}_summary.png"
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_summary)
    print(f"Saved: {out_summary}")

    # plt.show()


if __name__ == "__main__":
    plot_latest()
