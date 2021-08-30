import cv2
import mediapipe as mp


class HandDetector():
    def __init__(self, static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5,
                 min_tracking_confidence=0.5):
        self.mode = static_image_mode
        self.maxHands = max_num_hands
        self.detectionCon = min_detection_confidence
        self.trackCon = min_tracking_confidence

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode, max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.results = None

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)

        # iterate on recognized hands
        if self.results.multi_hand_landmarks is not None:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)

        return img

    def find_position(self, img, hand=0):
        lm_list = []

        if self.results.multi_hand_landmarks is not None:
            if len(self.results.multi_hand_landmarks) <= hand:
                print(f"hand {hand} not present")
                return None
            selected_hand = self.results.multi_hand_landmarks[hand]
            for id, lm in enumerate(selected_hand.landmark):
                height, width, channel = img.shape
                cx, cy = int(lm.x * width), int(lm.y * height)
                lm_list.append([id, cx, cy])
            return lm_list
        return None

    def find_depth(self, id, hand=0):
        if id > 20:
            print("id out of range")
        elif self.results.multi_hand_landmarks is not None:
            if len(self.results.multi_hand_landmarks) <= hand:
                print(f"hand {hand} not present")
                return None
            selected_hand = self.results.multi_hand_landmarks[hand]
            return selected_hand.landmark[id].z
        return None

