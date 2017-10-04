#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Vector3 #um es wie in Wii-remote zu machen
from std_msgs.msg import String

class VoiceCtrl:
	#[GER]	Defintion des Konstruktors	
    	#[ENG]	the constructor of the class
    def  __init__(self):
    	#[GER]	Initialisierung der Ros node mit "voice_teleop"
        #[ENG]	initialize the ROS node with a name e.g. "voice_teleop"
        rospy.init_node('voice_teleop')

        #init member
        self.voiceRecOff = False
        self.speed = 0.5

        #[GER]	Twist message an das cmd_vel topic publishen
        #[ENG]	Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # **Real System**
        #self.servo_cmd_vel_pub = rospy.Publisher('/servo', Twist, queue_size=5)
        # **Real System**

        #[GER]	An das /recognizer/output topic subscriben um Sprachkommandos zu empfangen
        #[ENG]	Subscribe to the /recognizer/output topic to receive voice commands
        rospy.Subscriber('/recognizer/output', String, self.voice_command_callback)

        #[GER]	mittels Rate Objekt while not loop mit 5 Hz zu durchlaufen
        #[ENG]Create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)

        #[GER]	Twist message initialisieren 
        #[ENG]	Initialize the Twist message we will publish
        self.cmd_vel = Twist()

        #[GER]	Standardmaessig soll das Fahrzeug i, Stillstand sein
        #[ENG]	make sure to make the robot stop by default
        #max = 1 
        self.cmd_vel.linear.x=0;
        self.cmd_vel.angular.z=0;

		# **Real System**
        #self.servo_cmd_vel = Vector3()
        #self.servo_cmd_vel.x = 1500;
        #self.servo_cmd_vel.y = 1500;
        # **Real System**


       	#[GER]	Einfaches mapping der Steuerungsbefehle 
		#[ENG]	Simple motion control command mapping
        self.commands =             ['stop',
                                'forward',
                                'back',
                                'left',
                                'right',
                                'voicecontrol off',
                                'voicecontrol on'
                                ]
        
        rospy.loginfo("Ready to receive voice commands")
		
		#[GER]	Kontnuierliches Publishen der cmd_vel message um den Roboter in Bewegung zu halten, 
		#	da andernfalls fuer jeden "Schritt" explizites Sprachkommando notwendig waere
		#[ENG]	Continiously publishing the cmd_vel message to keep the robot moving - otherwise robot would make one step per command
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            # **Real System**
            #self.servo_cmd_vel.publish(self.servo_cmd_vel)
            # **Real System**
            rate.sleep()

    def voice_command_callback(self, msg):
    	#[GER]	Steuerungskommando aus Spracherkennung speichern
        #[ENG] 	Get the motion command from the recognized phrase
        command = msg.data
        #[GER] 	Standardmaessig ist die Spracherkennung an
        #[ENG]	By default Voicerecognition is enabled
        if (command in self.commands and command == 'voicecontrol on'):
        	self.voiceRecOff = False
        	rospy.loginfo("Voicerecognition turned on")
        
        #[GER] 	Sofern sich das Kommando im Mapping befindet und die Spracherkennung aktiv ist folgt die Auswertung
        #[ENG]	if received command is within our mapping and Voicerecognition is enabled we basically switch case
        #	the command and execute motion commands
        if (command in self.commands and self.voiceRecOff == False):
        	
            if command == 'forward':
                self.cmd_vel.linear.x = self.speed
                self.cmd_vel.angular.z = 0.0
                # **Real System**
                #self.servo_cmd_vel.x = 1500
                #self.servo_cmd_vel.y = 1500
                # **Real System**
            elif command == 'back':
                self.cmd_vel.linear.x = -self.speed
                self.cmd_vel.angular.z = 0.0
                # **Real System**
                #self.servo_cmd_vel.x = 1500
                #self.servo_cmd_vel.y = 1500
                # **Real System**
            elif command == 'left':
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.5
                # **Real System**
                #self.servo_cmd_vel.x = 1500
                #self.servo_cmd_vel.y = 1500
                # **Real System**
            elif command == 'right':
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -self.speed
                # **Real System**
                #self.servo_cmd_vel.x = 1500
                #self.servo_cmd_vel.y = 1500
                # **Real System**
            elif command == 'stop':
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                # **Real System**
                #self.servo_cmd_vel.x = 1500
                #self.servo_cmd_vel.y = 1500
                # **Real System** ssh = tas2016
            elif command == 'voicecontrol off':
                self.voiceRecOff = True
                rospy.loginfo("Voicerecognition turned off")

        else: #command not found
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            # **Real System**
            #self.servo_cmd_vel.x = 1500
            #self.servo_cmd_vel.y = 1500   
            # **Real System**
        print ("linear speed : " + str(self.cmd_vel.linear.x))
        print ("angular speed: " + str(self.cmd_vel.angular.z))
        # **Real System**
        #print ("linear speed : " + str(self.servo_cmd_vel.x))
        #print ("angular speed: " + str(self.servo_cmd_vel.z))
		# **Real System**


if __name__=="__main__":
    try:
      VoiceCtrl()
      #[GER]	spin() Programmablauf bis voice_teleop node beendet wird
      #[ENG]	spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
    except rospy.ROSInterruptException:
      rospy.loginfo("Voice navigation terminated")


      #basis ros python construct

