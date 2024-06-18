#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import signal
import sys
import math

# Configuration des broches
F_TRIG_PIN = 2
F_ECHO_PIN = 3
L_TRIG_PIN = 21
L_ECHO_PIN = 20
R_TRIG_PIN = 5
R_ECHO_PIN = 6
LED_PIN = 17

# Initialisation des GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(F_TRIG_PIN, GPIO.OUT)
GPIO.setup(F_ECHO_PIN, GPIO.IN)
GPIO.setup(L_TRIG_PIN, GPIO.OUT)
GPIO.setup(L_ECHO_PIN, GPIO.IN)
GPIO.setup(R_TRIG_PIN, GPIO.OUT)
GPIO.setup(R_ECHO_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)

# Initialisation du node ROS et du publisher
rospy.init_node('gopigo_navigator', anonymous=True)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

# Variables d'état
current_orientation = 0.0

# Fonction de callback pour l'odométrie
def odom_callback(data):
    global current_orientation
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    current_orientation = yaw

# Abonnement au topic d'odométrie
rospy.Subscriber('/odom', Odometry, odom_callback)

# Fonction pour mesurer la distance
def distance(TRIG_PIN, ECHO_PIN):
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    pulse_start = time.time()
    pulse_end = time.time()
    
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    dist = pulse_duration * 17150
    return round(dist, 2)

# Fonction pour tourner à un angle spécifique
def turn_safely(direction, max_angle=math.pi/2):
    global current_orientation
    target_angle = current_orientation + max_angle if direction == 'left' else current_orientation - max_angle

    twist = Twist()
    
    step_angle = 0.1  # Angle de rotation par étape
    while abs(current_orientation - target_angle) > step_angle:
        rospy.loginfo("Turning!!")
        twist.angular.z = 0.3 if direction == 'left' else -0.3
        cmd_pub.publish(twist)
        rate.sleep()

        # Re-mesurer les distances
        distF = distance(F_TRIG_PIN, F_ECHO_PIN)
        distL = distance(L_TRIG_PIN, L_ECHO_PIN)
        distR = distance(R_TRIG_PIN, R_ECHO_PIN)
        
        rospy.loginfo(f"Distances - Front: {distF} cm, Left: {distL} cm, Right: {distR} cm")

        #if distF < 30 and distL < 30 and distR < 30:
         #  rospy.logwarn("Obstacle detected during turn, stopping turn")
          # twist.angular.z = 0.0
           #cmd_pub.publish(twist)
           #return  # Abandonner le virage si un obstacle est détecté

    twist.angular.z = 0.0
    cmd_pub.publish(twist)

# Fonction pour reculer et réévaluer la situation
def back_and_reassess():
    twist = Twist()
    twist.linear.x = -0.2  # Reculer
    cmd_pub.publish(twist)
    time.sleep(1.7)  # Reculer pendant 1 seconde, ajustez au besoin
    twist.linear.x = 0.0
    cmd_pub.publish(twist)

    # Re-mesurer les distances après le recul
    distF = distance(F_TRIG_PIN, F_ECHO_PIN)
    distL = distance(L_TRIG_PIN, L_ECHO_PIN)
    distR = distance(R_TRIG_PIN, R_ECHO_PIN)

    rospy.loginfo(f"New distances after backing up - Front: {distF} cm, Left: {distL} cm, Right: {distR} cm")
    
    if distL > distR:
        rospy.loginfo("Attempting to turn left")
        turn_safely('left')
    else:
        rospy.loginfo("Attempting to turn right")
        turn_safely('right')

# Gestion des signaux pour un arrêt propre
def signal_handler(sig, frame):
    print('Interrupt received, shutting down...')
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Boucle principale
try:
    while not rospy.is_shutdown():
        distF = distance(F_TRIG_PIN, F_ECHO_PIN)
        distL = distance(L_TRIG_PIN, L_ECHO_PIN)
        distR = distance(R_TRIG_PIN, R_ECHO_PIN)
        rospy.loginfo(f"Distances - Front: {distF} cm, Left: {distL} cm, Right: {distR} cm")

        twist = Twist()

        if (distF < 17 and (distR > 17 or distL > 17)):
            if (distL > distR):
                GPIO.output(LED_PIN, GPIO.HIGH)
                rospy.loginfo("Turning left")
                turn_safely('left', max_angle=math.pi/2)
            elif (distR > distL):
                GPIO.output(LED_PIN, GPIO.HIGH)
                rospy.loginfo("Turning right")
                turn_safely('right', max_angle=math.pi/2)
        elif (distF < 17 and distR < 17 and distL < 17):
            GPIO.output(LED_PIN, GPIO.HIGH)
            rospy.loginfo("No option infront and on the sides, backing up and reassessing")
            back_and_reassess()

        GPIO.output(LED_PIN, GPIO.LOW)
        rospy.loginfo("Path is clear, moving forward")
        twist.linear.x = 0.2
        cmd_pub.publish(twist)

        if (distF > 165 and distR > 165):
            rospy.loginfo("Maze Solved!")
            twist.linear.x = 0.0
            cmd_pub.publish(twist)
            sys.exit(0)
        time.sleep(0.5)

except rospy.ROSInterruptException:
    pass
finally:
    GPIO.cleanup()
    print("GPIO cleanup done.")

