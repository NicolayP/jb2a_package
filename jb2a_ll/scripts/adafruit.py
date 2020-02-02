'''
Layout: import adafruit libs,
    Struct containing info about every servo. List of angles (can switch to
     class if it becomes mandatory.)
    memorises the last pwm value to deliver a "odometrie"(odom) reading.
    subscribe to aboslute angle topics
    sends the message to every servo.

    publish to odom
    publish every angle at a given rate.

'''

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
