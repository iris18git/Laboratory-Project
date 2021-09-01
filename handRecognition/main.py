import cv2
import time
import math
from handRecognition.HandTrackingModule import HandDetector
import djitellopy as tello


def nothing(x):
    pass


class Drone:
    def __init__(self, drone, mv_speed, yaw_speed):
        self.tello = drone
        self.mv_speed = mv_speed
        self.yaw_speed = yaw_speed
        self.depth_speed = 0

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

        self.send_rc_control = True

    def get_image(self):
        img = self.tello.get_frame_read().frame
        return img

    def update(self, mv_angle, depth, yaw_angle):
        if mv_angle:
            self.left_right_velocity = int(math.cos(mv_angle) * self.mv_speed)
            self.up_down_velocity = int(math.sin(mv_angle) * self.mv_speed)
        else:
            self.left_right_velocity = 0
            self.up_down_velocity = 0

        if yaw_angle:
            self.yaw_velocity = (int(math.cos(yaw_angle) * self.yaw_speed))
        else:
            self.yaw_velocity = 0

        if depth:
            self.for_back_velocity = int(depth * self.depth_speed * 10)
        else:
            self.for_back_velocity = 0

        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                       self.up_down_velocity, self.yaw_velocity)

    def update_speed(self, _, yaw_speed=None, depth_speed=None, mv_speed=None):
        try:
            if yaw_speed:
                self.yaw_speed = yaw_speed
            else:
                self.yaw_speed = cv2.getTrackbarPos('yaw_speed', 'image')
            if depth_speed:
                self.depth_speed = depth_speed
            else:
                self.depth_speed = cv2.getTrackbarPos('depth_speed', 'image')

            if mv_speed:
                self.mv_speed = mv_speed
            else:
                self.mv_speed = cv2.getTrackbarPos('mv_speed', 'image')
            self.tello.set_speed(self.mv_speed)
        except:
            pass

    def update_state(self, _):
        if cv2.getTrackbarPos('takeoff', 'image') == 1:
            print("takeoff")
            self.tello.takeoff()
        else:
            print("land")
            self.tello.land()

    def run(self):
        p_time = 0
        c_time = 0
        detector = HandDetector(min_tracking_confidence=0.5, min_detection_confidence=0.5, max_num_hands=1, static_image_mode=False)

        self.tello.connect()
        self.tello.streamon()
        self.update_speed(0, mv_speed=self.mv_speed, yaw_speed=self.yaw_speed)
        print(f"battery: {self.tello.get_battery()}%")

        # cv2
        cv2.namedWindow('image')
        cv2.createTrackbar('mv_speed', 'image', self.mv_speed, 100, self.update_speed)
        cv2.createTrackbar('depth_speed', 'image', 0, 100, self.update_speed)
        cv2.createTrackbar('yaw_speed', 'image', self.yaw_speed, 100, self.update_speed)
        cv2.createTrackbar('movement/yaw', 'image', 0, 1, nothing)
        cv2.createTrackbar('takeoff', 'image', 0, 1, self.update_state)

        while True:
            # check if window is still open
            if cv2.getWindowProperty('image', cv2.WND_PROP_VISIBLE) < 1:
                break

            # capture image
            img = self.get_image()

            # parse image
            img = detector.find_hands(img)
            lm_list0 = detector.find_position(img, 0)

            # update drone
            if cv2.getTrackbarPos('takeoff', 'image') == 1:
                angle = find_angle(lm_list0) if lm_list0 else None
                if cv2.getTrackbarPos('movement/yaw', 'image') == 0:
                    self.update(angle, detector.find_depth(8), None)
                else:
                    self.update(None, None, angle)

            # calculate fps
            c_time = time.time()
            fps = 1 / (c_time - p_time)
            p_time = c_time
            cv2.putText(img,
                        f"fps: {int(fps)}, lr: {self.left_right_velocity}, fb: {self.for_back_velocity}, ud: {self.up_down_velocity}, yaw: {self.yaw_velocity}",
                        (10, 30), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)

            # display image
            cv2.imshow("image", img)
            cv2.waitKey(1)

        self.tello.streamoff()
        print("end of session")


def find_angle(lm_list):
    if lm_list is not None:
        return math.atan2(lm_list[0][2] - lm_list[8][2], lm_list[8][1] - lm_list[0][1])
    print("lm_list is None")
    return None


def main():
    # drone startup
    drone = Drone(tello.Tello(), 25, 15)
    drone.run()


if __name__ == "__main__":
    main()
