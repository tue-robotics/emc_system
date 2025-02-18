#!/usr/bin/env python

from __future__ import print_function

import threading

# import rospy
import rclpy, time
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d

r/f : increase/decrease max speeds by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0),
        's':(-1,0,0),
        'a':(0,0,1),
        'd':(0,0,-1),
        'q':(1, 0, 1),
        'e':(1, 0, -1)
        }

speedBindings={
        'r':(1.1,1.1),
        'f':(.9,.9),
         }


class PublishThread(threading.Thread):
    def __init__(self, rate, robot_name):
        super(PublishThread, self).__init__()
        # if not rospy.has_param('base_ref_'):
        #     raise Exception("Could not find base_ref_ on parameter server")
        # cmd_vel_topic = rospy.get_param('base_ref_')
        # self.publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        self.publisher = node.create_publisher(Twist, 'cmd_vel', 1)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            time.sleep(0.5)
            i += 1
            i = i % 5
        if not rclpy.ok():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, th, speed, turn):
        self.condition.acquire()
        self.x = float(x)
        self.y = float(y)
        self.th = float(th)
        self.speed = float(speed)
        self.turn = float(turn)
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    robot_name = "pico"
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

    # rospy.init_node('teleop_twist_keyboard_'+robot_name)

    rclpy.init(args=sys.argv)
    node = rclpy.create_node('teleop_twist_keyboard_'+robot_name)

    speed = node.declare_parameter("~speed", 0.25).value
    turn = node.declare_parameter("~turn", 0.5).value
    repeat = node.declare_parameter("~repeat_rate", 0.0).value
    key_timeout = node.declare_parameter("~key_timeout", 0.0).value

    if key_timeout == 0.0:
        key_timeout = None


    pub_thread = PublishThread(repeat, robot_name)

    x = 0
    y = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while (1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                continue
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and th == 0:
                    continue
                x = 0
                y = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
