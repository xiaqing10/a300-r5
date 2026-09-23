#!/usr/bin/env python3
# Keyboard teleop for the a300 safety-avoidance demo with HARD speed limits.
#
# Standard teleop_twist_keyboard key layout, but q/w/e can never push the
# linear/angular speed above the configured MAX_* limits. It publishes to
# /cmd_vel_desired so the desired velocity flows through the safety
# controller, which applies the obstacle-avoidance correction.
#
# Keys:
#   i  forward       j  turn left
#   ,  backward      l  turn right
#   k  stop
#   u o m .          diagonal
#   q/z  faster/slower (all speeds)     (clamped to MAX)
#   w/x  faster/slower (linear only)    (clamped to MAX_LINEAR)
#   e/c  faster/slower (angular only)   (clamped to MAX_ANGULAR)
#   Ctrl-C to quit

import sys
import threading

import rclpy
import rcl_interfaces.msg
import geometry_msgs.msg

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

HELP = """
Move a300 with the keyboard (publishes to /cmd_vel_desired):
   u    i    o
   j    k    l
   m    ,    .

i: forward   ,: backward   j: left   l: right   k: stop
q/z : faster/slower (both)        w/x : faster/slower (linear)
e/c : faster/slower (angular)
Speeds are clamped: linear<=%.2f m/s, angular<=%.2f rad/s.
Ctrl-C to quit.
"""

# Hard speed ceilings (m/s and rad/s).
MAX_LINEAR = 0.6
MAX_ANGULAR = 1.5
DEFAULT_SPEED = 0.3
DEFAULT_TURN = 1.0


def getKey(settings):
    if sys.platform == "win32":
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreSettings(s):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, s)


def vels(speed, turn):
    return "currently:\tspeed %.2f m/s\tturn %.2f rad/s" % (speed, turn)


def main():
    settings = saveSettings()
    rclpy.init()
    node = rclpy.create_node("a300_teleop_keyboard")

    ro = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter("stamped", False, ro).value
    frame_id = node.declare_parameter("frame_id", "", ro).value
    speed = node.declare_parameter("speed", DEFAULT_SPEED, ro).value
    turn = node.declare_parameter("turn", DEFAULT_TURN, ro).value
    max_linear = node.declare_parameter("max_linear", MAX_LINEAR, ro).value
    max_angular = node.declare_parameter("max_angular", MAX_ANGULAR, ro).value
    topic = node.declare_parameter("topic", "cmd_vel_desired", ro).value

    if stamped and not frame_id:
        frame_id = "base_link"
    Twist = geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist
    pub = node.create_publisher(Twist, topic, 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    move_bindings = {
        "i": (1, 0, 0, 0),
        "o": (1, 0, 0, -1),
        "j": (0, 0, 0, 1),
        "l": (0, 0, 0, -1),
        "u": (1, 0, 0, 1),
        ",": (-1, 0, 0, 0),
        ".": (-1, 0, 0, 1),
        "m": (-1, 0, 0, -1),
        "O": (1, -1, 0, 0),
        "I": (1, 0, 0, 0),
        "J": (0, 1, 0, 0),
        "L": (0, -1, 0, 0),
        "U": (1, 1, 0, 0),
        "<": (-1, 0, 0, 0),
        ">": (-1, -1, 0, 0),
        "M": (-1, 1, 0, 0),
    }
    speed_bindings = {
        "q": (1.1, 1.1),
        "z": (0.9, 0.9),
        "w": (1.1, 1.0),
        "x": (0.9, 1.0),
        "e": (1.0, 1.1),
        "c": (1.0, 0.9),
    }

    x = y = z = th = 0.0
    status = 0
    msg = Twist()
    if stamped:
        twist = msg.twist
    else:
        twist = msg

    print(HELP % (max_linear, max_angular))
    print(vels(speed, turn))
    try:
        while True:
            key = getKey(settings)
            if key in move_bindings:
                x, y, z, th = move_bindings[key]
            elif key in speed_bindings:
                lf, af = speed_bindings[key]
                speed = min(speed * lf, max_linear)
                turn = min(turn * af, max_angular)
                print(vels(speed, turn))
                if status == 14:
                    print(HELP % (max_linear, max_angular))
                status = (status + 1) % 15
            else:
                x = y = z = th = 0.0
                if key == "\x03":
                    break

            if stamped:
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(msg)
    except Exception as e:
        print(e)
    finally:
        if stamped:
            msg.header.stamp = node.get_clock().now().to_msg()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(msg)
        rclpy.shutdown()
        spinner.join()
        restoreSettings(settings)


if __name__ == "__main__":
    main()
