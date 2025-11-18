#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe.media_library import *
from time import sleep, time
import rclpy
from rclpy.node import Node

class PoseCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)

        self.PWMServo_X = 0
        self.PWMServo_Y = -45
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y

        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)

        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.event = threading.Event()
        self.event.set()
        

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.car_ctrl_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def go_quadrilateral(self):
        # SQUARE PATTERN: 4 sides with 90° turns
        # Moves forward, turns right, repeats 4 times
        import math
        
        side_duration = 3.0       # Time to move forward per side
        turn_duration = math.pi / 2  # 90° turn (1.57 seconds at 1.0 rad/s)
        
        for i in range(4):
            # Move forward for one side
            self.media_ros.pub_vel(0.3, 0.0, 0.0)
            sleep(side_duration)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
            
            # Turn 90 degrees right
            self.media_ros.pub_vel(0.0, 0.0, 1.0)
            sleep(turn_duration)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
    
    def go_diamond(self):
        # DIAMOND PATTERN: Square rotated 45° using diagonal movements
        # Pattern: ◇ (forward-right, back-right, back-left, forward-left)
        import math
        
        # Diagonal velocity components (45° angles)
        vel = 0.3  # Base velocity for each axis
        side_duration = 3.0  # Time per side
        # Actual diagonal speed: sqrt(0.3² + 0.3²) ≈ 0.424 m/s
        # Distance per side: 0.424 * 3.0 ≈ 1.27m
        
        # Side 1: Forward-Right diagonal (↗)
        self.media_ros.pub_vel(vel, -vel, 0.0)  # x=forward, y=right (negative)
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)
        
        # Side 2: Backward-Right diagonal (↘)
        self.media_ros.pub_vel(-vel, -vel, 0.0)  # x=backward, y=right
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)
        
        # Side 3: Backward-Left diagonal (↙)
        self.media_ros.pub_vel(-vel, vel, 0.0)  # x=backward, y=left (positive)
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)
        
        # Side 4: Forward-Left diagonal (↖)
        self.media_ros.pub_vel(vel, vel, 0.0)  # x=forward, y=left
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)

    def go_s(self):
        self.media_ros.pub_vel(0.3, 0.0,-0.5)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)
        self.media_ros.pub_vel(0.3, 0.0,0.5)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)

    def Go_circle(self,flag):
        if (flag == 1):
            self.media_ros.pub_vel(0.3, 0.0,0.5)
            sleep(13)
            self.media_ros.pub_vel(0.0, 0.0,0.0)
        if (flag != 1):
            self.media_ros.pub_vel(0.3, 0.0,-0.5)
            sleep(13)
            self.media_ros.pub_vel(0.0, 0.0,0.0)
    
    def go_figure_eight(self):
        # Figure-8: True infinity symbol (∞) with proper closure
        # Pattern: Start → Right loop → Center → Left loop → Return to start
        import math
        
        # Configuration for precise circles
        linear_vel = 0.3          # Forward speed (m/s)
        angular_vel = 2 * math.pi / 9  # Exactly 360° in 9 seconds (≈0.698 rad/s)
        loop_duration = 9.0       # Time for each complete circle
        transition_time = 0.3     # Time to cross center between loops
        
        # Calculate radius: r = v/ω ≈ 0.43m (43cm radius circles)
        # Total pattern: ~19.2 seconds, returns to start
        
        # First loop: Clockwise circle (right side)
        self.media_ros.pub_vel(linear_vel, 0.0, angular_vel)
        sleep(loop_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)  # Brief pause for stability
        
        # Transition to center (crossing point of infinity symbol)
        self.media_ros.pub_vel(linear_vel, 0.0, 0.0)
        sleep(transition_time)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)
        
        # Second loop: Counter-clockwise circle (left side)
        self.media_ros.pub_vel(linear_vel, 0.0, -angular_vel)
        sleep(loop_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)
        
        # Return to starting position (close the figure-8)
        self.media_ros.pub_vel(linear_vel, 0.0, 0.0)
        sleep(transition_time)
        
        # Final stop at starting position
        self.media_ros.pub_vel(0.0, 0.0, 0.0)    
        
    def go_fast_spin(self):
        # Spin in place very fast for 12 seconds
        self.media_ros.pub_vel(0.0, 0.0, 2.0)
        sleep(12)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
    
    def go_triangle(self):
        # Triangle pattern: 3 sides with 135-degree turns (wider angles)
        # LARGER triangle with longer sides and wider angles
        # Based on square pattern: 1.0 rad/s for 2.2s = 90°
        # For 135°: 1.0 rad/s for 2.356s (increased from 120° for more obtuse triangle)
        for i in range(3):
            # Move forward for one side (increased from 2.5s to 4s for larger triangle)
            self.media_ros.pub_vel(0.3, 0.0, 0.0)
            sleep(4.0)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
            # Turn 135 degrees (2.356 radians) - wider angle than standard equilateral
            self.media_ros.pub_vel(0.0, 0.0, 1.0)
            sleep(3.4)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
    
    def go_back_forth(self):
        # Move forward and backward repeatedly (2 times for cleaner pattern)
        for i in range(2):
            # Forward
            self.media_ros.pub_vel(0.3, 0.0, 0.0)
            sleep(2.0)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
            # Backward
            self.media_ros.pub_vel(-0.3, 0.0, 0.0)
            sleep(2.0)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)


    def car_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            gesture = self.hand_detector.get_gesture(lmList)
            print("gesture detected: ", gesture)
            
            # Check named gestures FIRST (higher priority, most specific)
            if gesture == "Yes":	
                self.go_quadrilateral()
                sleep(1.5)

            elif gesture == "OK":
                self.Go_circle(1)
                sleep(1.5)

            elif gesture == "Thumb_down":
                self.media_ros.pub_vel(0.3, 0.0,0.0)
                sleep(2)
                sleep(2)
                self.media_ros.pub_vel(0.0, 0.0,0.0)
                sleep(1.5)
            
            elif gesture == "Rock_on":  # Diamond pattern
                self.go_diamond()
                sleep(1.5)
            
            elif gesture == "Thumb_up":  # Counter-clockwise circle
                self.Go_circle(0)
                sleep(1.5)
                
            elif gesture == "Shaka":  # Figure-8 infinity pattern
                self.go_figure_eight()
                sleep(1.5)
                
            elif gesture == "Spin":  # Fast spin in place
                self.go_fast_spin()
                sleep(1.5)
                
            elif gesture == "Three":  # Triangle pattern
                self.go_triangle()
                sleep(1.5)
                
            elif gesture == "Four":  # Back and forth movement
                self.go_back_forth()
                sleep(1.5)

            # Then check generic finger patterns (lower priority)
            elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2:
                self.go_s()
                sleep(1.5)

            elif sum(fingers) == 5: 
                self.media_ros.pub_vel(0.0, 0.0,0.0)
                sleep(1.5)

            self.event.set()

def main():
    rclpy.init()
    pose_ctrl = PoseCtrlArm('posectrlarm')
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        ret, frame = capture.read()
        frame = pose_ctrl.process(frame)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
    rclpy.spin(pose_ctrl)
