import djitellopy
from scipy.spatial.transform import Rotation as R
from doorDetectorModule.doorDetector import DoorDetector
import math
import djitellopy as Tello

def main(path_to_point_map, path_to_tello_location):
    dD = DoorDetector(path_to_point_map)
    door = dD.find_door_coordinates()

    with open(path_to_tello_location, 'r') as f:
        lines = f.read().splitlines()
        last_line = lines[-1]
        loc = last_line.split()
        if len(loc) != 8:
            print ("length of line isn't 8!")
            return

    pos = loc[1:4]
    print(pos)
    quaternion = loc[5:]
    print(quaternion)
    quaternion = R.from_quat(quaternion)
    angles = quaternion.as_rotvec()
    curr_angle = angles[0]
    wanted_angle = math.atan2(pos[1]-door[1], pos[0]-door[0])
    dist = math.dist(pos[:1], door[:1])

    turn_angle = curr_angle-wanted_angle
    turn_angle = math.degrees(turn_angle)
    print(turn_angle)

    tello = Tello()
    tello.connect()
    tello.takeoff()

    tello.rotate_clockwise(turn_angle)
    tello.move_forward(dist)

    tello.land()



if __name__ == '__main__':
    main('doorDetectorModule/pointData0.csv', 'doorDetectorModule/KeyFrameTrajectory.txt')