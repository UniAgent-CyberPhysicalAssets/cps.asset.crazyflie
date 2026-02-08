import sys
import numpy as np

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.mem import CompressedSegment
from cflib.crazyflie.mem import CompressedStart
from cflib.crazyflie.mem import MemoryElement


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


# # The trajectory to fly
a = 0.25  # where the Beizer curve control point should be https://spencermortensen.com/articles/bezier-circle/
h = 0.55  # [m] how high we should fly
t = 2.0  # seconds per step, one circle has 4 steps


# r1 = 0.2  # [m] the radius at which the first half of the drones are
# r2 = np.sqrt(0.8)  # [m] the radius at which every second other drone is
# # loops = 2  # how many loops we should fly

def upload_trajectory(cf: Crazyflie, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    trajectory_mem.trajectory = trajectory

    upload_result = trajectory_mem.write_data_sync()
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(
        trajectory_id,
        0,
        len(trajectory),
        type=HighLevelCommander.TRAJECTORY_TYPE_POLY4D_COMPRESSED)

    total_duration = 0
    # Skip the start element
    for segment in trajectory[1:]:
        total_duration += segment.duration
    return total_duration


def rotate_beizer_node(xl, yl, alpha):
    x_rot = []
    y_rot = []
    for x, y in zip(xl, yl):
        x_rot.append(x * np.cos(alpha) - y * np.sin(alpha))
        y_rot.append(x * np.sin(alpha) + y * np.cos(alpha))
    return x_rot, y_rot


def create_circle_trajectory(alpha, r):
    x_start, y_start = rotate_beizer_node([r], [0.0], alpha)
    beizer_point_1_x, beizer_point_1_y = rotate_beizer_node([r, r * a, 0.0], [r * a, r, r], alpha)
    beizer_point_2_x, beizer_point_2_y = rotate_beizer_node([-r * a, -r, -r], [r, r * a, 0.0], alpha)
    beizer_point_3_x, beizer_point_3_y = rotate_beizer_node([-r, -r * a, 0.0], [-r * a, -r, -r], alpha)
    beizer_point_4_x, beizer_point_4_y = rotate_beizer_node([r * a, r, r], [-r, -r * a, 0.0], alpha)
    trajectory = [
        CompressedStart(x_start[0], y_start[0], h, 0.0),
        CompressedSegment(t, beizer_point_1_x, beizer_point_1_y, [h], []),
        CompressedSegment(t, beizer_point_2_x, beizer_point_2_y, [h], []),
        CompressedSegment(t, beizer_point_3_x, beizer_point_3_y, [h], []),
        CompressedSegment(t, beizer_point_4_x, beizer_point_4_y, [h], []),
    ]
    return trajectory


def build_trajectory(sequence_json):
    trajectory = []

    for elem in sequence_json:
        if elem["type"] == "start":
            trajectory.append(
                CompressedStart(
                    elem["x"],
                    elem["y"],
                    elem["z"],
                    elem["yaw"]
                )
            )

        elif elem["type"] == "segment":
            trajectory.append(
                CompressedSegment(
                    elem["duration"],
                    elem["x"],
                    elem["y"],
                    elem["z"],
                    elem["yaw"]
                )
            )

    return trajectory
