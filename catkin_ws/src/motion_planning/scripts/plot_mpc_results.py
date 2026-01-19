#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import sys
import datetime


def load_and_prep(csv_file):
    df = pd.read_csv(csv_file)

    # Normalize time to start at 0
    if "time" in df.columns and not df.empty:
        df["time"] = df["time"] - df["time"].iloc[0]

    # Calculate missing error columns if plotting ArmController logs
    if "error_x" not in df.columns and "target_x" in df.columns:
        df["error_x"] = df["target_x"] - df["actual_x"]
    if "error_y" not in df.columns and "target_y" in df.columns:
        df["error_y"] = df["target_y"] - df["actual_y"]
    if "error_z" not in df.columns and "target_z" in df.columns:
        df["error_z"] = df["target_z"] - df["actual_z"]
    if "error_norm" not in df.columns:
        if "error_dist" in df.columns:
            df["error_norm"] = df["error_dist"]
        elif "error_x" in df.columns:
            df["error_norm"] = np.sqrt(
                df["error_x"] ** 2 + df["error_y"] ** 2 + df["error_z"] ** 2
            )
    return df


def plot_mpc_results(csv_file=None):
    """Plot MPC controller performance"""

    if csv_file is None:
        # Find latest MPC CSV file
        # Check default container path, current dir, and workspace root relative to script
        search_patterns = [
            "/root/catkin_ws/performance_results_*.csv",
            os.path.join(os.getcwd(), "performance_results_*.csv"),
            os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__), "../../../performance_results_*.csv"
                )
            ),
        ]
        files = []
        for pattern in search_patterns:
            files.extend(glob.glob(pattern))

        if not files:
            print("No MPC result files found!")
            return
        csv_file = max(files, key=os.path.getctime)

    print(f"Processing MPC data from: {csv_file}")

    # Read data
    df = load_and_prep(csv_file)

    # Determine controller type
    controller_type = "MPC"
    if "error_orient" in df.columns:
        controller_type = "Arm"

    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(
        f"{controller_type} Controller Performance\n{os.path.basename(csv_file)}",
        fontsize=16,
    )

    # Plot 1: Position tracking (X, Y, Z)
    time_data = df["time"] - df["time"].iloc[0]

    axes[0, 0].plot(time_data, df["target_x"], "r--", label="Target X", linewidth=2)
    axes[0, 0].plot(time_data, df["actual_x"], "r-", label="Actual X", linewidth=1)
    axes[0, 0].plot(time_data, df["target_y"], "g--", label="Target Y", linewidth=2)
    axes[0, 0].plot(time_data, df["actual_y"], "g-", label="Actual Y", linewidth=1)
    axes[0, 0].plot(time_data, df["target_z"], "b--", label="Target Z", linewidth=2)
    axes[0, 0].plot(time_data, df["actual_z"], "b-", label="Actual Z", linewidth=1)
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Position (m)")
    axes[0, 0].set_title("Position Tracking")
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Plot 2: Position errors
    axes[0, 1].plot(time_data, df["error_x"], "r-", label="Error X")
    axes[0, 1].plot(time_data, df["error_y"], "g-", label="Error Y")
    axes[0, 1].plot(time_data, df["error_z"], "b-", label="Error Z")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Error (m)")
    axes[0, 1].set_title("Position Errors")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Plot 3: Error norm (overall accuracy)
    (l1,) = axes[1, 0].plot(
        time_data, df["error_norm"], "k-", linewidth=2, label="Position Error (m)"
    )
    axes[1, 0].axhline(y=0.005, color="r", linestyle="--", label="5mm threshold")
    axes[1, 0].axhline(y=0.002, color="g", linestyle="--", label="2mm threshold")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Position Error (m)")
    axes[1, 0].set_title("Overall Tracking Accuracy")
    axes[1, 0].grid(True)
    axes[1, 0].set_yscale("log")

    # Add Orientation Error if available
    if "error_orient" in df.columns:
        ax2 = axes[1, 0].twinx()
        (l2,) = ax2.plot(
            time_data,
            df["error_orient"],
            "orange",
            linestyle="-",
            label="Orientation Error",
        )
        ax2.set_ylabel("Orientation Error (quat dist)", color="orange")
        ax2.tick_params(axis="y", labelcolor="orange")
        axes[1, 0].legend(handles=[l1, l2], loc="upper right")
    else:
        axes[1, 0].legend()

    # Plot 4: Joint velocities
    vel_columns = [col for col in df.columns if "joint_vel_" in col]
    if not vel_columns:
        # Fallback for ArmController logs (v1, v2, ...)
        vel_columns = [f"v{i}" for i in range(1, 7) if f"v{i}" in df.columns]

    for col in vel_columns:
        label = (
            col.replace("joint_vel_", "Joint ")
            if "joint_vel_" in col
            else col.replace("v", "Joint ")
        )
        axes[1, 1].plot(time_data, df[col], label=label)
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Velocity (rad/s)")
    axes[1, 1].set_title("Joint Velocities")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    # Plot 5: Error distribution histogram
    axes[2, 0].hist(df["error_norm"].dropna(), bins=50, alpha=0.7, color="blue")
    axes[2, 0].axvline(
        x=df["error_norm"].mean(),
        color="r",
        linestyle="--",
        label=f'Mean: {df["error_norm"].mean():.4f}m',
    )
    axes[2, 0].axvline(
        x=df["error_norm"].std(),
        color="g",
        linestyle="--",
        label=f'Std: {df["error_norm"].std():.4f}m',
    )
    axes[2, 0].set_xlabel("Error Norm (m)")
    axes[2, 0].set_ylabel("Frequency")
    axes[2, 0].set_title("Error Distribution")
    axes[2, 0].legend()
    axes[2, 0].grid(True)

    # Plot 6: Performance summary
    axes[2, 1].axis("off")

    orient_summary = ""
    if "error_orient" in df.columns:
        orient_summary = f"\n    Avg Orient Error: {df['error_orient'].mean():.4f}\n    Final Orient Error: {df['error_orient'].iloc[-1]:.4f}"

    summary_text = f"""
    {controller_type} Controller Performance Summary:
    
    Total Execution Time: {time_data.iloc[-1]:.2f} s
    Average Pos Error: {df["error_norm"].mean():.4f} m
    Max Pos Error: {df["error_norm"].max():.4f} m
    Std Deviation: {df["error_norm"].std():.4f} m{orient_summary}

    Final Position Error:
    X: {df["error_x"].iloc[-1]:.4f} m
    Y: {df["error_y"].iloc[-1]:.4f} m
    Z: {df["error_z"].iloc[-1]:.4f} m
    
    Success Criteria (<5mm): {'✓' if df["error_norm"].iloc[-1] < 0.005 else '✗'}
    """
    axes[2, 1].text(
        0.1,
        0.5,
        summary_text,
        fontsize=10,
        verticalalignment="center",
        transform=axes[2, 1].transAxes,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    plt.tight_layout()

    # Save plot
    plot_file = csv_file.replace(".csv", ".png")
    plt.savefig(plot_file, dpi=300, bbox_inches="tight")
    print(f"Plot saved to: {plot_file}")

    # Show plot
    # plt.show()


def compare_results(file_list):
    print(f"Comparing {len(file_list)} files...")
    data = []
    for f in file_list:
        if not os.path.exists(f):
            print(f"File not found: {f}")
            continue
        df = load_and_prep(f)
        name = os.path.basename(f)
        data.append({"name": name, "df": df})

    if not data:
        print("No valid data to compare.")
        return

    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle("Controller Performance Comparison", fontsize=16)

    # 1. Error Norm Overlay
    ax = axes[0, 0]
    for item in data:
        ax.plot(
            item["df"]["time"],
            item["df"]["error_norm"],
            label=item["name"][:25] + "...",
        )
    ax.set_title("Position Error Norm (Log Scale)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_yscale("log")
    ax.legend()
    ax.grid(True)

    # 2. Bar Chart Comparison
    ax = axes[0, 1]
    names = [d["name"][:15] + "..." for d in data]
    final_errors = [d["df"]["error_norm"].iloc[-1] * 1000 for d in data]  # mm
    mean_errors = [d["df"]["error_norm"].mean() * 1000 for d in data]  # mm

    x = np.arange(len(names))
    width = 0.35
    ax.bar(x - width / 2, final_errors, width, label="Final Error (mm)")
    ax.bar(x + width / 2, mean_errors, width, label="Mean Error (mm)")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15)
    ax.set_title("Error Metrics (mm)")
    ax.legend()
    ax.grid(True, axis="y")

    # 3. Trajectory Overlay (XY)
    ax = axes[1, 0]
    for item in data:
        ax.plot(item["df"]["actual_x"], item["df"]["actual_y"], label=item["name"][:25])
    ax.set_title("XY Trajectory")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True)

    # 4. Text Summary
    ax = axes[1, 1]
    ax.axis("off")

    print("\n" + "=" * 60)
    print(f"{'File':<30} | {'Final(mm)':<10} | {'Mean(mm)':<10} | {'Time(s)':<8}")
    print("-" * 65)

    best_file = min(data, key=lambda x: x["df"]["error_norm"].iloc[-1])

    summary_text = "Comparison Summary:\n\n"
    for item in data:
        df = item["df"]
        final_err = df["error_norm"].iloc[-1] * 1000
        mean_err = df["error_norm"].mean() * 1000
        duration = df["time"].iloc[-1]

        print(
            f"{item['name'][:30]:<30} | {final_err:<10.3f} | {mean_err:<10.3f} | {duration:<8.2f}"
        )

        summary_text += f"--- {item['name'][:30]}... ---\n"
        summary_text += f"Final Error: {final_err:.3f} mm\n"
        summary_text += f"Mean Error:  {mean_err:.3f} mm\n"
        summary_text += f"Duration:    {duration:.2f} s\n\n"

    print("-" * 65)
    print(f"🏆 BEST RESULT: {best_file['name']}")
    print("=" * 60 + "\n")

    ax.text(
        0.05,
        0.95,
        summary_text,
        transform=ax.transAxes,
        verticalalignment="top",
        fontsize=11,
        family="monospace",
    )

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = f"/root/catkin_ws/comparison_{timestamp}.png"
    plt.tight_layout()
    plt.savefig(plot_file, dpi=300)
    print(f"Comparison plot saved to: {plot_file}")


if __name__ == "__main__":
    if len(sys.argv) > 2:
        compare_results(sys.argv[1:])
    else:
        csv_file = sys.argv[1] if len(sys.argv) > 1 else None
        plot_mpc_results(csv_file)
