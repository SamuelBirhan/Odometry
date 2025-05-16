import csv
import math
import rosbag2_py
import sys

from datetime import datetime, timedelta
from pathlib import Path

from typing import List, Tuple

import numpy as np
import rclpy

from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from rclpy.node import Node

from sensor_msgs.msg import JointState

ROBOT_WIDTH = 324.0
WHEEL_DIAMETER = 195.0
GEAR_RATIO = 38.3
TICKS_PER_MM = 128

def calculate_odometry_velocity(
    time_delta_s: float,
    position_l: int,
    position_r: int,
    velocity_l: float,
    velocity_r: float,
    current_orientation: float,
) -> Tuple[float, float, float]:
    """Calculates the odometry of the robot.

    Returns tuple of floats: [dx, dy, dtheta]
    """
    linear_velocity = (velocity_l + velocity_r) / 2.0

    dx = linear_velocity * time_delta_s * math.cos(current_orientation)
    dy = linear_velocity * time_delta_s * math.sin(current_orientation)
    dtheta = (time_delta_s * (velocity_r - velocity_l)) / ROBOT_WIDTH

    return dx, dy, dtheta

def calculate_odometry_position(
    time_delta_s: float,
    position_l: int,
    position_r: int,
    velocity_l: float,
    velocity_r: float,
    current_orientation: float,
    last_position_l: int,
    last_position_r: int,
) -> Tuple[float, float, float]:
    """Calculates the odometry of the robot.

    Returns tuple of floats: [dx, dy, dtheta]
    """

    def calculate_difference(current: int, previous: int) -> int:
        diff = current - previous
        if abs(diff) > 32_768:
            if diff < 0:
                diff = diff + 65_536

            else:
                diff = diff - 65_536
        return diff

    distance_l = calculate_difference(position_l, last_position_l) / TICKS_PER_MM
    distance_r = calculate_difference(position_r, last_position_r) / TICKS_PER_MM

    dx = (distance_r + distance_l) * math.cos(current_orientation) * 0.5
    dy = (distance_r + distance_l) * math.sin(current_orientation) * 0.5
    dtheta = (distance_r - distance_l) / ROBOT_WIDTH

    return dx, dy, dtheta

class OdometryCalculatorVelocities:
    """Odometry calculator for the robot position."""
    
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        # Initialize positions from the first recorded odometry data
        self.x_position_mm = initial_x * 1000  # Convert meters to mm
        self.y_position_mm = initial_y * 1000  # Convert meters to mm
        self.orientation = math.radians(initial_theta)  # Convert degrees to radians

        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.last_timer_execution = datetime.now()

        self.last_position_l = None
        self.last_position_r = None

        self.pioneer_odom = []
        self.our_odom = []


    def odometry_callback(self, msg: Odometry) -> None:
        

        pos = msg.pose.pose.position

        print(
            f"Data from pioneer5: ({pos.x:.3f}, {pos.y:.3f},"
            f"{math.degrees(yaw_from_quaternion(msg.pose.pose.orientation)):.3f})"
        )

        self.pioneer_odom.append(
            [pos.x, pos.y, math.degrees(yaw_from_quaternion(msg.pose.pose.orientation))]
        )

    def encoders_callback(self, msg: JointState) -> None:
        """Callback for the encoders message."""

        if self.last_position_l is None or self.last_position_r is None:
            self.last_position_l, self.last_position_r = msg.position
            return

        current_execution_time = datetime.now()
        dt = (current_execution_time - self.last_timer_execution).total_seconds()

        # dx, dy, dtheta = calculate_odometry_velocity(
        #     dt,
        #     msg.position[0],
        #     msg.position[1],
        #     msg.velocity[0],
        #     msg.velocity[1],
        #     self.orientation,
        # )

        dx, dy, dtheta = calculate_odometry_position(
            dt,
            msg.position[0],
            msg.position[1],
            msg.velocity[0],
            msg.velocity[1],
            self.orientation,
            self.last_position_l,
            self.last_position_r,
        )

        self.x_position_mm += dx
        self.y_position_mm += dy
        self.orientation += dtheta

        if self.orientation > math.pi:
            self.orientation -= 2 * math.pi
        elif self.orientation < -math.pi:
            self.orientation += 2 * math.pi
        self.our_odom.append(
            [
                self.x_position_mm / 1000.0,
                self.y_position_mm / 1000.0,
                math.degrees(self.orientation),
            ]
        )

        print(
            f"x: {self.x_position_mm / 1000.0:.3f} "
            f"y: {self.y_position_mm / 1000.0:.3f} "
            f"angle: {math.degrees(self.orientation):.3f}"
        )

        self.last_position_l = msg.position[0]
        self.last_position_r = msg.position[1]

        self.last_timer_execution = current_execution_time 

def yaw_from_quaternion(quaternion):
    """Converts quaternion (w in last place) to yaw angle"""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return yaw


import rclpy.serialization
from rosidl_runtime_py.utilities import get_message

def read_rosbag(db3_path: str):
    """Reads the ROS2 bag file and extracts joint states and odometry data."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=db3_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    reader.open(storage_options, converter_options)

    joint_states = []
    odometry_data = []

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    while reader.has_next():
        topic, serialized_bytes, t = reader.read_next()

        # Deserialize the message
        msg_type = get_message(type_map[topic])
        msg = rclpy.serialization.deserialize_message(serialized_bytes, msg_type)

        if topic == "/pioneer5/joint_states":
            joint_states.append(msg)
        elif topic == "/pioneer5/odom":
            odometry_data.append(msg)

    return joint_states, odometry_data




import matplotlib.pyplot as plt
from pathlib import Path

def plot_odometry_comparison(computed_odom, ground_truth_odom, filename):
    """Plots and saves the difference between computed and recorded odometry."""
    x_axis = list(range(min(len(computed_odom), len(ground_truth_odom))))

    # 1️Plot Ground Truth vs Computed Odometry Path with Dots
    plt.figure()
    plt.scatter(
        [x for x, y, theta in ground_truth_odom], 
        [y for x, y, theta in ground_truth_odom], 
        color="b", marker="o", s=5, label="Recorded Position"
    )
    plt.scatter(
        [x for x, y, theta in computed_odom], 
        [y for x, y, theta in computed_odom], 
        color="r", marker="o", s=5, label="Calculated Position"
    )
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Computed vs. Recorded Odometry")
    plt.legend(loc="best")
    plt.savefig(f"{filename}_path.png")  #  Save Figure
    plt.close()

    # 2️ Plot Odometry Error in Position (X & Y) - Using Dots
    plt.figure()
    for i, value in enumerate(x_axis):
        plt.plot(value, ground_truth_odom[i][0] - computed_odom[i][0], 'b.', markersize=5, label="X Error" if i == 0 else "")
        plt.plot(value, ground_truth_odom[i][1] - computed_odom[i][1], 'r.', markersize=5, label="Y Error" if i == 0 else "")
    plt.xlabel("No of measurement")
    plt.ylabel("Odometry error [m]")
    plt.title("Odometry error in position")
    plt.legend(loc="upper right")
    plt.savefig(f"{filename}_position_error.png")  #  Save Figure
    plt.close()

    # 3️ Plot Odometry Error in Orientation (Theta) - Using Dots
    plt.figure()
    for i, value in enumerate(x_axis):
        plt.plot(value, ground_truth_odom[i][2] - computed_odom[i][2], 'b.', markersize=5, label="Theta Error" if i == 0 else "")
    plt.xlabel("No of measurement")
    plt.ylabel("Odometry error [deg]")
    plt.title("Odometry error in orientation")
    plt.legend(loc="upper right")
    plt.savefig(f"{filename}_orientation_error.png")  #  Save Figure
    plt.close()

    print(f" Figures saved: {filename}_path.png, {filename}_position_error.png, {filename}_orientation_error.png")

def main(args: List[str]) -> None:
    """Processes odometry from ROS bag and compares with computed odometry."""
    db3_path = args[0]  # Expecting the .db3 file path as a command-line argument
    rosbag_filename = Path(db3_path).stem  # Extract filename without extension for saving

    joint_states, odometry_data = read_rosbag(db3_path)

    # Extract initial position and orientation from the first recorded odometry message
    first_odom = odometry_data[0]  
    initial_x_recorded = first_odom.pose.pose.position.x
    initial_y_recorded = first_odom.pose.pose.position.y
    initial_theta_recorded = math.degrees(yaw_from_quaternion(first_odom.pose.pose.orientation))

    # Initialize the odometry calculator with the recorded starting position
    odometry_calculator = OdometryCalculatorVelocities(initial_x_recorded, initial_y_recorded, initial_theta_recorded)

    for msg in joint_states:
        odometry_calculator.encoders_callback(msg)

    computed_odom = odometry_calculator.our_odom
    ground_truth_odom = []

    # Extract the recorded odometry from the ROS bag
    for msg in odometry_data:
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        ground_truth_odom.append([pos.x, pos.y, math.degrees(yaw)])

    # Extract the last computed and recorded position (Last Index)
    if len(computed_odom) > 0 and len(ground_truth_odom) > 0:
        last_x_computed, last_y_computed, last_theta_computed = computed_odom[-1]
        last_x_recorded, last_y_recorded, last_theta_recorded = ground_truth_odom[-1]

        # Compute errors
        error_x = abs(last_x_recorded - last_x_computed)
        error_y = abs(last_y_recorded - last_y_computed)
        error_theta = abs(last_theta_recorded - last_theta_computed)

        # Print Last Position Comparison with Rosbag Filename
        print(f"\n Last Position Comparison ({rosbag_filename}):")
        print(f"{'Measurement':<15} {'Recorded':<20} {'Calculated':<20} {'Error':<10}")
        print("-" * 75)
        print(f"{'x [m]':<15} {last_x_recorded:<20.4f} {last_x_computed:<20.4f} {error_x:<10.4f}")
        print(f"{'y [m]':<15} {last_y_recorded:<20.4f} {last_y_computed:<20.4f} {error_y:<10.4f}")
        print(f"{'θ [°]':<15} {last_theta_recorded:<20.4f} {last_theta_computed:<20.4f} {error_theta:<10.4f}")
        print("-" * 75)
    else:
        print(f"\n⚠️ Not enough data points to extract the last position from {rosbag_filename}.")

    # Compare computed vs recorded odometry & Save Figures
    plot_odometry_comparison(computed_odom, ground_truth_odom, rosbag_filename)


def main_csv(args: List[str]) -> None:
    """Main script entrypoint for processing CSV data."""
    x, y, orientation = 0, 0, 0
    last_time, last_position_l, last_position_r = None, None, None
    file = Path(args[0])

    with file.open("r") as csvfile:
        reader = csv.DictReader(csvfile, delimiter=";", quotechar="|")
        for row in reader:
            if not last_time or not last_position_l or not last_position_r:
                last_time = float(row["#time[s]"])
                last_position_l = float(row["pos_l"])
                last_position_r = float(row["pos_r[ticks]"])
                continue

            dt = float(row["#time[s]"]) - last_time
            dx, dy, dtheta = calculate_odometry_position(
                dt,
                float(row["pos_l"]),
                float(row["pos_r[ticks]"]),
                float(row["vel_l"]),
                float(row["vel_r[mm/s]"]),
                float(orientation),
                float(last_position_l),
                float(last_position_r),
            )

            # Alternative calculation method (commented out)
            # dx, dy, dtheta = calculate_odometry_velocity(
            #     dt,
            #     float(row["pos_l"]),
            #     float(row["pos_r[ticks]"]),
            #     float(row["vel_l"]),
            #     float(row["vel_r[mm/s]"]),
            #     float(orientation),
            # )

            last_time = float(row["#time[s]"])
            last_position_l = float(row["pos_l"])
            last_position_r = float(row["pos_r[ticks]"])
            x += dx
            y += dy
            orientation += dtheta

    print(
        f"Final x = {x/1000:.4f} [m], "
    
        f"y = {y/1000:.4f} [m], "
        f"theta = {math.degrees(orientation):.4f} [degrees] "
        )


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))




