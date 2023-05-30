#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, sin, cos, degrees

# Variable globale pour stocker la pose de la tortue
current_pose = Pose()

# Callback appelée lorsqu'un message est reçu sur le topic "pose"
def pose_callback(msg):
    global current_pose
    current_pose = msg

# Initialisation du nœud ROS
rospy.init_node('turtle_regulation_pemba_nathan')

# Souscription au topic "pose" avec la fonction de rappel pose_callback
rospy.Subscriber('pose', Pose, pose_callback)

# Définition du waypoint avec les coordonnées (7, 7)
# Coordonnées des points A et B
xA, yA = 0, 0  # Coordonnées de la tortue
xB, yB = 7, 7  # Coordonnées du waypoint

# Calcul de l'angle désiré en degrés
desired_angle = degrees(atan2(yB - yA, xB - xA))

# Calcul de l'erreur
error = atan2(sin(desired_angle - current_pose.theta), cos(desired_angle - current_pose.theta))

print("Erreur :", error)

# Constante de proportionnalité Kp
Kp = 7.0

# Calcul de la commande en cap du robot
command = Kp * error

# Création d'un éditeur pour publier sur le topic "cmd_vel"
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Fonction d'aide pour créer un objet Twist avec la vitesse angulaire en z
def create_twist(angular):
    twist = Twist()
    twist.angular.z = angular
    return twist

# Boucle principale
rate = rospy.Rate(10)  # Fréquence de publication (10 Hz)
while not rospy.is_shutdown():
    # Publication de la commande en cap
    pub.publish(create_twist(command))

    rate.sleep()
