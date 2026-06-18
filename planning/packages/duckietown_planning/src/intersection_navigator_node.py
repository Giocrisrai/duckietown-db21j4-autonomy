#!/usr/bin/env python3
# planning/packages/duckietown_planning/src/intersection_navigator_node.py
"""Nodo reactivo: en cada interseccion elige y ejecuta un giro."""
import rospy
from std_msgs.msg import String, Bool
from duckietown_planning.intersection_policy import choose_turn

class IntersectionNavigatorNode:
    def __init__(self):
        rospy.init_node("intersection_navigator_node")
        self.planned_turn = None
        self._prev_stop = False
        self.pub_turn = rospy.Publisher("~turn_cmd", String, queue_size=1)
        rospy.Subscriber("~at_stop_line", Bool, self.cb_stop_line, queue_size=1)
        rospy.Subscriber("~planned_turn", String, self.cb_planned, queue_size=1)
        rospy.loginfo("intersection_navigator_node listo")

    def cb_planned(self, msg):
        self.planned_turn = msg.data or None

    def cb_stop_line(self, msg):
        rising = msg.data and not self._prev_stop   # solo flanco False->True
        self._prev_stop = msg.data
        if not rising:
            return
        # Salidas disponibles: en una interseccion estandar Duckietown asumimos
        # las tres; el FSM real puede filtrarlas. Reactivo robusto por defecto.
        available = rospy.get_param("~available_turns", ["left", "straight", "right"])
        turn = choose_turn(available, planned=self.planned_turn)
        rospy.loginfo("Interseccion -> giro: %s (plan=%s)", turn, self.planned_turn)
        self.pub_turn.publish(String(data=turn))
        self.planned_turn = None   # se consume el plan

if __name__ == "__main__":
    IntersectionNavigatorNode()
    rospy.spin()
