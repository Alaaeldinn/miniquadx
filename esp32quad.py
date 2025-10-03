#!/usr/bin/env python3
import rospy
import socket
import sys
import termios
import tty
import os

# ============ CONFIG ============
ESP32_IP   = "192.168.1.120"   # <-- change to your ESP32-CAM IP
ESP32_PORT = 8888              # must match ESP32 UDP port

THROTTLE_BASE = 1100           # initial throttle (us)
PITCH_BASE    = 1500
ROLL_BASE     = 1500
YAW_BASE      = 1500

STEP = 50                      # increment in us per key press
# ================================

# state
throttle = THROTTLE_BASE
pitch    = PITCH_BASE
roll     = ROLL_BASE
yaw      = YAW_BASE
armed    = False

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd: str):
    """Send raw UDP command string to ESP32"""
    sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
    rospy.loginfo(f"Sent: {cmd}")

def send_motors():
    """Send motor mix (very simple for now)"""
    # simple mixer (replace with PID later)
    # for demo: throttle plus offsets
    m1 = throttle + (pitch - 1500) - (roll - 1500) + (yaw - 1500)
    m2 = throttle + (pitch - 1500) + (roll - 1500) - (yaw - 1500)
    m3 = throttle - (pitch - 1500) + (roll - 1500) + (yaw - 1500)
    m4 = throttle - (pitch - 1500) - (roll - 1500) - (yaw - 1500)

    # clamp
    m1 = max(1000, min(2000, m1))
    m2 = max(1000, min(2000, m2))
    m3 = max(1000, min(2000, m3))
    m4 = max(1000, min(2000, m4))

    cmd = f"M1:{m1} M2:{m2} M3:{m3} M4:{m4}"
    send_command(cmd)

def get_key():
    """Read single key press (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    global throttle, pitch, roll, yaw, armed

    rospy.init_node("keyboard_controller")
    rospy.loginfo("Keyboard controller started. Use keys to control drone.")

    print("""
    Controls:
      W/S - pitch forward/back
      A/D - roll left/right
      ↑/↓ (i/k) - throttle up/down
      ←/→ (j/l) - yaw left/right
      Enter - ARM motors
      Space - DISARM motors
      q - quit
    """)

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w': pitch += STEP
        elif key == 's': pitch -= STEP
        elif key == 'a': roll -= STEP
        elif key == 'd': roll += STEP
        elif key == 'i': throttle += STEP
        elif key == 'k': throttle -= STEP
        elif key == 'j': yaw -= STEP
        elif key == 'l': yaw += STEP

        elif key == '\r':   # Enter
            send_command("ARM 1")
            armed = True
        elif key == ' ':
            send_command("ARM 0")
            armed = False

        elif key == 'q':
            send_command("ARM 0")
            rospy.loginfo("Quitting...")
            break

        # clamp ranges
        throttle = max(1000, min(2000, throttle))
        pitch    = max(1000, min(2000, pitch))
        roll     = max(1000, min(2000, roll))
        yaw      = max(1000, min(2000, yaw))

        # send only if armed
        if armed:
            send_motors()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

