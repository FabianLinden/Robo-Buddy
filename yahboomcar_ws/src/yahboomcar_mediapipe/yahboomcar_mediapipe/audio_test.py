#!/usr/bin/env python3
# encoding: utf-8
"""
Audio Test Script for Yahboom Robot
Demonstrates how to play sounds through USB speakers
"""

import rclpy
from media_library import Media_ROS
from time import sleep

def main():
    """Test audio playback functionality"""
    
    rclpy.init()
    
    # Create Media_ROS instance
    media_ros = Media_ROS()
    
    print("\n" + "="*50)
    print("🔊 YAHBOOM ROBOT AUDIO TEST")
    print("="*50 + "\n")
    
    # Test 1: Play a sound and wait for it to finish
    print("Test 1: Playing sound (blocking mode)")
    print("Note: Replace with your actual sound file path")
    # media_ros.play_sound("/home/robot/sounds/hello.mp3", wait_finish=True)
    # print("Sound finished playing\n")
    
    # Test 2: Play sound without blocking (async)
    print("Test 2: Playing sound (non-blocking mode)")
    # media_ros.play_sound_async("/home/robot/sounds/beep.wav")
    print("Sound is playing in background, robot can continue...")
    sleep(2)  # Do other things while sound plays
    print("Done with other tasks\n")
    
    # Test 3: Volume control
    print("Test 3: Volume control")
    media_ros.set_volume(0.3)  # 30% volume
    print("Volume set to 30%")
    # media_ros.play_sound("/home/robot/sounds/test.wav")
    
    media_ros.set_volume(0.7)  # 70% volume
    print("Volume set to 70%")
    # media_ros.play_sound("/home/robot/sounds/test.wav")
    
    media_ros.set_volume(1.0)  # 100% volume
    print("Volume set to 100%\n")
    
    # Test 4: Stop sound
    print("Test 4: Stop sound playback")
    # media_ros.play_sound_async("/home/robot/sounds/long_audio.mp3")
    sleep(2)
    media_ros.stop_sound()
    print("Sound stopped\n")
    
    print("="*50)
    print("Audio tests complete!")
    print("="*50)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

