#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe.media_library import *
from time import sleep, time
import rclpy
from rclpy.node import Node
import math


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
    # Polling state
        self.is_polling = False
        self.polling_gesture = None
        self.polling_start_time = None
        self.accumulated_detection_time = 0.0
        self.last_correct_detection_time = None
        self.last_frame_time = None
    
    # Grace period tracking
        self.grace_period_active = False
    
    # Wrong gesture tracking
        self.wrong_gesture_duration = 0.0
    
    # Cooldown state
        self.in_cooldown = False
        self.cooldown_end_time = None
    
    # Console feedback
        self.last_print_time = None
    
    # Configuration parameters (easy to tune)
        self.POLLING_DURATION = 3.5 #3.5 seconds
        self.DETECTION_THRESHOLD = 0.75  # 75%
        self.WRONG_GESTURE_TOLERANCE = 0.5  # seconds
        self.GRACE_PERIOD_DURATION = 0.3  # seconds
        self.COOLDOWN_DURATION = 1.0  # seconds

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
        self.media_ros.pub_vel(0.3, 0.0,0.0)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)
        self.media_ros.pub_vel(0.0, 0.0,1.0)
        sleep(2.2)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)


        self.media_ros.pub_vel(0.3, 0.0,0.0)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)
        self.media_ros.pub_vel(0.0, 0.0,1.0)
        sleep(2.2)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)

        self.media_ros.pub_vel(0.3, 0.0,0.0)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)
        self.media_ros.pub_vel(0.0, 0.0,1.0)
        sleep(2.2)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)


        self.media_ros.pub_vel(0.3, 0.0,0.0)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)
        self.media_ros.pub_vel(0.0, 0.0,1.0)
        sleep(2.2)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(1)

    def go_square(self):
        # SQUARE PATTERN: 4 sides with 90 degree turns
        # Moves forward, turn, repeat 4 times

        side_duration = 3.0          # Time to move forward per side
        turn_duration = math.pi / 2  # 90 degree turn (1.57 seconds at 1.0 rad/s)

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
        # DIAMOND PATTERN: Sqaure rotated 45 degrees using diagonal movements

        # Diagonal velocity components (45-degree angles)
        vel = 0.3 # Base velocity for each axis
        side_duration = 3.0 # Time per side

        # Side 1: Forward-Right diagonal
        self.media_ros.pub_vel(vel, -vel, 0.0) # x = forward, y =right (negative)
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)

        # Side 2: Back-Right diagonal
        self.media_ros.pub_vel(-vel, -vel, 0.0) # x = backward, y -right
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)

        # Side 3: Backward-Left diagonal
        self.media_ros.pub_vel(-vel, vel, 0.0) # x = backward, y = left (positive)
        sleep(side_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.5)

        # Side 4: Forward-Left diagonal
        self.media_ros.pub_vel(vel, vel, 0.0) # x = forward, y = left
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
    def go_christmas_dance(self):
#    Epic Christmas dance routine - combines spins, forward/back moves, and side shuffles

            
            # === INTRO: Quick spin ===
        self.media_ros.pub_vel(0.0, 0.0, 3.0)  # Fast spin
        sleep(.5)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)
            
       # === PERFORMANCE LOOP: Face each side (4 sides) ===
        for side in range(4):
        # Quick forward-back groove (faster!)
                for i in range(2):
                    # Quick forward
                        self.media_ros.pub_vel(0.5, 0.0, 0.0)  # Faster velocity
                        sleep(0.3)  # Shorter duration
                        self.media_ros.pub_vel(0.0, 0.0, 0.0)
                        sleep(0.05)
                    
                    # Quick back
                        self.media_ros.pub_vel(-0.5, 0.0, 0.0)
                        sleep(0.3)
                        self.media_ros.pub_vel(0.0, 0.0, 0.0)
                        sleep(0.05)
                # Side shuffle (looking at this side of audience)
                self.media_ros.pub_vel(0.0, 0.5, 0.0)  # Quick strafe
                sleep(0.4)
                self.media_ros.pub_vel(0.0, -0.5, 0.0)  # Back to center
                sleep(0.4)
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                sleep(0.1)
        
            
             # Turn 90 degrees to face next side (unless it's the last side)
                if side < 3:
                    self.media_ros.pub_vel(0.0, 0.0, 2.0)
                    sleep(math.pi / 4)  # 90-degree turn (0.785 seconds)
                    self.media_ros.pub_vel(0.0, 0.0, 0.0)
                    sleep(0.1)
            
            # === FINALE: Ultra fast spin to original position! ===
        self.media_ros.pub_vel(0.0, 0.0, 5.0)  # Maximum speed!
        sleep(1.5)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
          
            
    def go_figure_eight(self):
        # Figure 8: True infinity symbol
        # Pattern: Start -> RIght loop -> Center -> Left loop -> Return to start

        # Configuration for precision circles
        linear_vel = 0.3               # Forward speed (m/s)
        angular_vel = 2 * math.pi / 9  # Exactly 360 degrees in 9 seconds (approximately 0.698 rad/s)
        loop_duration = 9.0            # Time for each circle to complete
        transition_time = 0.3          # Time to cross center between loops

        #$ First loop: clockwise circle (smaller/tigher than full circle)
        self.media_ros.pub_vel(linear_vel, 0.0, angular_vel)
        sleep(loop_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2) # Brief pause for stability

        # Smooth transition to center (crossing point of infinite circle)
        self.media_ros.pub_vel(linear_vel, 0.0, 0.0)
        sleep(transition_time)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)

        # Second loop: counter-clockwise circle (left side)
        self.media_ros.pub_vel(linear_vel, 0.0, -angular_vel)
        sleep(loop_duration)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(0.2)

        # Return to starting position (close the figure-8)
        self.media_ros.pub_vel(linear_vel, 0.0, 0.0)
        sleep(transition_time)

        #Final stop at starting position
        self.media_ros.pub_vel(0.0, 0.0, 0.0)

    def go_fast_spin(self):
        # Spin in place very fast for 12 seconds
        self.media_ros.pub_vel(0.0, 0.0, 4.0)
        sleep(12)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)

    def go_triangle(self):
        # Triangle pattern 3 sides with 135-degree turns
        # Based on square pattern: 1.0 rad/s for 2.2s = 90 degrees
        # For 135-degrees: 1.0 rad/s for 2.356s (or adjust velocity)
        for i in range(3):
            # Move forward for one side
            self.media_ros.pub_vel(0.3, 0.0, 0.0)
            sleep(4.0)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
            # Turn 135 degrees (2.356 radians)
            self.media_ros.pub_vel(0.0, 0.0, 1.1)
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
            self.media_ros.pub_vel(-0.3, 0.0, 0.0)
            sleep(2.0)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(0.5)
    def car_ctrl_threading(self, lmList, bbox):
            if self.event.is_set():
                self.event.clear()
                
                current_time = time()
                gesture = self.hand_detector.get_gesture(lmList)
                
                # Initialize last_frame_time on first run
                if self.last_frame_time is None:
                    self.last_frame_time = current_time
                    self.event.set()
                    return
                
                # Calculate time delta since last frame
                time_delta = current_time - self.last_frame_time
                self.last_frame_time = current_time
                # ===== COOLDOWN CHECK =====
                if self.in_cooldown:
                    if current_time >= self.cooldown_end_time:
                        print("Cooldown ended, ready for new gestures")
                        self.in_cooldown = False
                    else:
                        # Still in cooldown, ignore all gestures
                        self.event.set()
                        return
                
                # ===== START POLLING =====
                if not self.is_polling:
                    # Check if valid gesture detected (not empty/None)
                    if gesture and gesture not in ["", "None"]:
                        print(f"\n?? Starting polling for: {gesture}")
                        self.is_polling = True
                        self.polling_gesture = gesture
                        self.polling_start_time = current_time
                        self.accumulated_detection_time = 0.0
                        self.last_correct_detection_time = current_time
                        self.wrong_gesture_duration = 0.0
                        self.last_print_time = current_time
                   # ===== ACTIVE POLLING =====
                elif self.is_polling:
                    elapsed_time = current_time - self.polling_start_time
                    
                    # Handle current gesture
                    if gesture == self.polling_gesture:
                        # Correct gesture detected
                        self.accumulated_detection_time += time_delta
                        self.last_correct_detection_time = current_time
                        self.wrong_gesture_duration = 0.0
                        
                    elif gesture in ["", "None"]:
                        # Grace period logic
                        time_since_correct = current_time - self.last_correct_detection_time
                        if time_since_correct < self.GRACE_PERIOD_DURATION:
                            # Within grace period - no penalty
                            pass
                        else:
                            # Grace period expired - time keeps running but not accumulated
                            pass
                            
                    else:
                        # Different gesture detected
                        self.wrong_gesture_duration += time_delta
                        
                        if self.wrong_gesture_duration >= self.WRONG_GESTURE_TOLERANCE:
                            print(f"? Wrong gesture '{gesture}' held too long ({self.wrong_gesture_duration:.2f}s) - RESET")
                            self._reset_polling()
                            self.event.set()
                            return
                 # Periodic console updates (every 1 second)
                    if current_time - self.last_print_time >= 1.0:
                        percentage = (self.accumulated_detection_time / elapsed_time * 100) if elapsed_time > 0 else 0
                        time_remaining = self.POLLING_DURATION - elapsed_time
                        print(f"[{elapsed_time:.1f}s] {self.polling_gesture}: {self.accumulated_detection_time:.2f}s/{elapsed_time:.2f}s ({percentage:.0f}%) | {time_remaining:.1f}s remaining")
                        self.last_print_time = current_time
                    
                    # Check if polling duration complete
                    if elapsed_time >= self.POLLING_DURATION:
                        percentage = self.accumulated_detection_time / elapsed_time
                        
                        if percentage >= self.DETECTION_THRESHOLD:
                            print(f"? SUCCESS! {self.polling_gesture}: {self.accumulated_detection_time:.2f}s/{elapsed_time:.2f}s ({percentage*100:.0f}%) - EXECUTING!\n")
                            self._execute_gesture(self.polling_gesture)
                            self._reset_polling()
                            
                            # Enter cooldown
                            self.in_cooldown = True
                            self.cooldown_end_time = current_time + self.COOLDOWN_DURATION
                            print(f"??  Cooldown active for {self.COOLDOWN_DURATION}s\n")
                        else:
                            print(f"? FAILED! {self.polling_gesture}: {self.accumulated_detection_time:.2f}s/{elapsed_time:.2f}s ({percentage*100:.0f}%) - Below {self.DETECTION_THRESHOLD*100:.0f}% threshold\n")
                            self._reset_polling()
                
                self.event.set()
                
    def _reset_polling(self):
            """Reset all polling state variables"""
            self.is_polling = False
            self.polling_gesture = None
            self.polling_start_time = None
            self.accumulated_detection_time = 0.0
            self.last_correct_detection_time = None
            self.wrong_gesture_duration = 0.0
    
    def _execute_gesture(self, gesture):
            """Execute the movement for the confirmed gesture"""
            if gesture == "Yes":
                self.go_quadrilateral()
                
            elif gesture == "OK":
                self.go_christmas_dance() 
                
            elif gesture == "Thumb_down":
                self.media_ros.pub_vel(0.3, 0.0, 0.0)
                sleep(4)
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                
            elif gesture == "Rock_on":
                self.go_diamond()
                
            elif gesture == "Thumb_up":
                self.go_fast_spin()
                
            elif gesture == "Shaka":
                self.go_figure_eight()
                
            elif gesture == "Spin":
                self.go_fast_spin()
                
            elif gesture == "Square":
                self.go_square()
                
            elif gesture == "Three":
                self.go_triangle()
                
            elif gesture == "Four":
                self.go_back_forth()    
def printDescription():
        print("""GESTURE  ACTION MAP
                YES
                  Gesture: Thumbs up + index pointing
                  Action: Quadrilateral driving pattern

                OK
                  Gesture: Thumb + index circle
                  Action: Christmas Dance beware 20s long

                THUMB_DOWN
                  Gesture: Thumb pointing down
                  Action: Move forward for 4 seconds, then stop

                ROCK_ON
                  Gesture: Index + pinky extended ("horns")
                  Action: Diamond shaped movement

                THUMB_UP
                  Gesture: Thumb pointing up
                  Action: Fast spin for 12 seconds

                SHAKA
                  Gesture: Thumb + pinky 
                  Action: Figure 8 pattern

                SPIN
                  Gesture: Middle finger 0.0
                  Action: Fast spin for 12 seconds

                SQUARE
                  Gesture: Custom gesture
                  Action: Square path with 90 degree turns

                THREE
                  Gesture: Three fingers extended
                  Action: Triangle pattern (135 degree turns)

                FOUR
                  Gesture: Four fingers extended
                  Action: Forward and backward twice

                ROCK
                  Gesture: Index + pinky (detected via finger count)
                  Action: S pattern movement

                ALL FINGERS UP
                  Gesture: Five fingers extended
                  Action: STOP""")

def main():
    printDescription()
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
