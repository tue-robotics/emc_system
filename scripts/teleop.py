#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Teleoperation instructions:
---------------------------

w : accelerate forward
s : accelerate backward

a : accelerate to the left
d : accelerate to the right

anything else : stop moving

CTRL-C to quit
"""

accelBindings={
        'w':(0.2,0),
        'a':(0,0.2),
        's':(-0.2,0),
        'd':(0,-0.2)
          }

class PublishThread(threading.Thread):
    def __init__(self, rate, robot_name):
        super(PublishThread, self).__init__()
        if not rospy.has_param('base_ref_'):
            raise Exception("Could not find base_ref_ on parameter server")
        cmd_vel_topic = rospy.get_param('base_ref_')
        self.publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
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
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, speed, turn):
        self.condition.acquire()
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
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
    return "speed:\tlinear %.2f\tangular %.2f " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    robot_name = "pico"
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

    rospy.init_node('teleop_twist_keyboard_'+robot_name)

    speed = rospy.get_param("~speed", 0.0)
    turn = rospy.get_param("~turn", 0.0)
    repeat = rospy.get_param("~repeat_rate", 10.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat, robot_name)

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(speed, turn)

        print(msg)
        print(vels(speed, turn))
        while (1):
            key = getKey(key_timeout)
            if key in accelBindings.keys():
                accel = accelBindings[key]
                speed += accel[0]
                turn += accel[1]
                speed = max(min(speed,1),-1)
                turn = max(min(turn,1),-1)
            elif key =='':
                    continue
            else :
                speed = 0
                turn = 0
                if key =='\x03': # ctrl+c
                    pub_thread.update(speed, turn) # before stopping teleop, publish 0 velocity one last time
                    break
            
            print(vels(speed, turn))
            pub_thread.update(speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
