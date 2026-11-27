"""Run one AirHUST Gazebo course and retain the observations and landing measurement."""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]


def recognize(frame, stage, detector):
    h, w = frame.shape[:2]
    if stage == 1:
        roi = frame[h//2-h//10:h//2+h//10, w//2-w//10:w//2+w//10]
        hue, sat, val = cv2.split(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV))
        masks = [(hue < 12) | (hue > 165), (hue > 35) & (hue < 85),
                 (hue > 95) & (hue < 140)]
        scores = [np.mean(mask & (sat > 70) & (val > 45)) for mask in masks]
        if max(scores) > .35:
            return ['red', 'green', 'blue'][np.argmax(scores)]
    elif stage == 2:
        ok, texts, _, _ = detector.detectAndDecodeMulti(frame)
        if ok and any(texts):
            return next(text for text in texts if text)
        return detector.detectAndDecode(frame)[0]
    elif stage == 3:
        _, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            return str(ids.flatten()[0])
    elif stage == 4:
        import pytesseract
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
            x, y, width, height = cv2.boundingRect(contour)
            if min(width, height) < 30:
                continue
            region = cv2.resize(gray[y:y+height, x:x+width], None, fx=2, fy=2)
            for rotation in range(4):
                text = pytesseract.image_to_string(np.rot90(region, rotation).copy(),
                    config='--psm 7 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ').strip()
                if text == 'AIRHUST':
                    return text
    else:
        result = detector.predict(frame, imgsz=224, conf=.25, device='cpu', verbose=False)[0]
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            if abs((x1+x2)/2-w/2) < .25*w and abs((y1+y2)/2-h/2) < .25*h:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                print('confidence', float(box.conf[0]), flush=True)
                return result.names[int(box.cls[0])]


def wait_for(check, seconds, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not rospy.is_shutdown():
        if check():
            return
        time.sleep(.1)
    raise RuntimeError('Timeout: ' + label)


class Mission:
    def __init__(self):
        self.world = self.local = self.image = self.goal = None
        self.state = State()
        self.bridge = CvBridge()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.models, queue_size=1)
        rospy.Subscriber('/mavros/state', State, lambda m: setattr(self, 'state', m))
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         lambda m: setattr(self, 'local', m.pose.position))
        rospy.Subscriber('/usb_cam/image_raw', Image, lambda m: setattr(self, 'image', m),
                         queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.timer = rospy.Timer(rospy.Duration(.05), self.publish)

    def models(self, msg):
        if 'iris' in msg.name:
            self.world = msg.pose[msg.name.index('iris')].position

    def publish(self, _):
        loc, world, goal = self.local, self.world, self.goal
        if loc is None or world is None or goal is None:
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.w = 1
        pos = msg.pose.position
        pos.x, pos.y, pos.z = loc.x+goal[0]-world.x, loc.y+goal[1]-world.y, loc.z+goal[2]-world.z
        self.pub.publish(msg)

    def arrive(self, x, y, z):
        self.goal = (x, y, z)
        def reached():
            p = self.world
            return math.hypot(p.x-x, p.y-y) < .10 and abs(p.z-z) < .13
        wait_for(reached, 120, 'waypoint ' + str(self.goal))
        rospy.sleep(.7)

    def run(self, stage, out, report):
        detector = None
        if stage == 2:
            detector = cv2.QRCodeDetector()
        elif stage == 3:
            detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
        elif stage == 5:
            from ultralytics import YOLO
            detector = YOLO(str(ROOT/'models/logo.pt'))
        wait_for(lambda: self.state.connected and self.local is not None and self.world is not None
                 and self.image is not None, 150, 'camera, gazebo, mavros')
        altitude = 1.5 if stage == 1 else 1.2
        self.goal = (0, 0, altitude)
        rospy.sleep(2)
        for attempt in range(12):
            if self.state.mode != 'OFFBOARD':
                self.mode(0, 'OFFBOARD')
            elif not self.state.armed:
                self.arm(True)
            else:
                break
            rospy.sleep(1)
        if not self.state.armed or self.state.mode != 'OFFBOARD':
            raise RuntimeError('OFFBOARD or arming rejected')
        self.arrive(0, 0, altitude)
        for y, expected in ([(-1,'red'), (0,'green'), (1,'blue')] if stage == 1 else [(0,None)]):
            self.arrive(1, y, altitude)
            last = self.image.header.stamp
            for attempt in range(8):
                wait_for(lambda: self.image.header.stamp > last, 20, 'fresh image')
                img = self.image
                last = img.header.stamp
                frame = self.bridge.imgmsg_to_cv2(img, 'bgr8').copy()
                value = recognize(frame, stage, detector)
                cv2.imwrite(str(out/('view_%d.jpg' % y)), frame)
                if value and (expected is None or value == expected):
                    report['observations'].append(dict(value=value, stamp=last.to_sec(), y=y))
                    print('observed', value, flush=True)
                    break
                rospy.sleep(.35)
            else:
                raise RuntimeError('No matching observation at y=' + str(y))
        self.mode(0, 'AUTO.LAND')
        wait_for(lambda: not self.state.armed, 120, 'automatic disarm')
        p = self.world
        error = math.hypot(p.x-1, p.y-y)
        report.update(x=p.x, y=p.y, z=p.z, error_m=error, passed=error < .30 and p.z < .3)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', type=int, choices=range(1,6))
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True, parents=True)
    rospy.init_node('airhust_sim_course')
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Start the course Gazebo world first')
    mission = Mission()
    report = dict(stage=args.stage, passed=False, observations=[])
    try:
        mission.run(args.stage, args.output, report)
    except Exception as exc:
        report['error'] = str(exc)
        if mission.state.armed:
            mission.mode(0, 'AUTO.LAND')
    finally:
        (args.output/'result.json').write_text(json.dumps(report, indent=2))
        print(report, flush=True)
    raise SystemExit(0 if report['passed'] else 1)
