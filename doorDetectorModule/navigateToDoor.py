from doorDetector import DoorDetector
import math
from djitellopy import Tello
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# func taken from the nice dudes on intewebes
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radian

# path_to_point_map - path to the csv containing the point map generated.
# path_to_tello_location - path to the txt file of keyFrameTrajectory.
# return type - None
def main(path_to_point_map, path_to_tello_location):
    dD = DoorDetector(path_to_point_map)
    door = dD.find_door_coordinates(True)

    # get information from last line - last instance saved
    with open(path_to_tello_location, 'r') as f:
        lines = f.read().splitlines()
        last_line = lines[-1]
        loc = last_line.split()
        if len(loc) != 8:
            print ("length of line isn't 8!")
            return

    pos = np.array(loc[1:4]).astype(np.float32)
    print(f'our pos {pos} vs door pos {door}')
    quaternion = np.array(loc[4:], dtype=float)
    angles = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    curr_angle = angles[2]
    wanted_angle = math.atan2((door[1]-pos[1]),(door[0]-pos[0]))
    dist = math.dist(pos[:1], door[:1])


    # draw info
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(door[0], door[1], door[2], c='g', s=1000)
    ax.scatter(pos[0], pos[1], pos[2], c='b')
    ax.scatter(dD.x, dD.y, dD.z, c='r', s = 0.1)
    # draw vectors
    ax.quiver(pos[0], pos[1], pos[2], math.cos(wanted_angle), math.sin(wanted_angle), 0, color='g')
    ax.quiver(pos[0], pos[1], pos[2], math.cos(curr_angle), math.sin(curr_angle), 0, color='b')


    plt.show()

    turn_angle = wanted_angle - curr_angle
    turn_angle = math.degrees(turn_angle)
    print(turn_angle)

    tello = Tello()
    tello.connect()
    tello.takeoff()

    tello.rotate_counter_clockwise(int(turn_angle))
    tello.move_forward(int(100*dist))

    tello.land()



if __name__ == '__main__':
    main('pointData.csv', 'KeyFrameTrajectory.txt')