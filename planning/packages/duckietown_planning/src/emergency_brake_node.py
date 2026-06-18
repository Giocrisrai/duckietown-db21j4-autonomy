#!/usr/bin/env python3
"""Nodo ROS: frena de emergencia si el ToF detecta un obstáculo cercano."""
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from duckietown_planning.safety import should_brake


class EmergencyBrakeNode:
    """Nodo que suscribe a mensajes de distancia del ToF y publica comandos de freno."""

    def __init__(self):
        """Inicializa el nodo ROS y configura los suscriptores/publicadores."""
        rospy.init_node("emergency_brake_node")
        self.threshold = rospy.get_param("~threshold_m", 0.15)
        self.pub = rospy.Publisher("~emergency_stop", Bool, queue_size=1)
        rospy.Subscriber("~range", Range, self.cb, queue_size=1)
        rospy.loginfo("emergency_brake_node listo (umbral=%.2fm)", self.threshold)

    def cb(self, msg):
        """Callback que procesa mensajes de distancia y publica decisión de freno.

        Args:
            msg: Mensaje Float32 con la distancia en metros.
        """
        frenar = should_brake(msg.range, self.threshold)
        self.pub.publish(Bool(data=frenar))


if __name__ == "__main__":
    EmergencyBrakeNode()
    rospy.spin()
