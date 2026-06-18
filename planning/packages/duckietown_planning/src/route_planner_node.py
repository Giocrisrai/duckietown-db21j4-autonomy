#!/usr/bin/env python3
# planning/packages/duckietown_planning/src/route_planner_node.py
"""Nodo global: dado un mapa + destino, publica el proximo giro planeado.

Si no hay mapa o destino, NO publica nada -> el navegador reactivo (Task 14)
decide por su cuenta. Asi el global es puramente aditivo y con fallback seguro.
"""
import rospy
from std_msgs.msg import String
from duckietown_planning.map_graph import load_map
from duckietown_planning.route_search import plan_route
from duckietown_planning.localization import IntersectionLocalizer

class RoutePlannerNode:
    def __init__(self):
        rospy.init_node("route_planner_node")
        map_path = rospy.get_param("~map_path", "")
        start = rospy.get_param("~start", "")
        goal = rospy.get_param("~goal", "")
        self.route = []
        self.loc = None
        if map_path and start and goal:
            graph = load_map(map_path)
            self.route = plan_route(graph, start, goal) or []
            self.loc = IntersectionLocalizer(graph, start)
            if self.route:
                rospy.loginfo("Ruta planeada %s->%s: %s", start, goal, self.route)
            else:
                rospy.logwarn("plan_route no encontro ruta %s->%s: el reactivo decidira solo.", start, goal)
        else:
            rospy.logwarn("Sin mapa/destino: el reactivo decidira solo.")
        self.pub = rospy.Publisher("~planned_turn", String, queue_size=1, latch=True)
        rospy.Subscriber("~executed_turn", String, self.cb_executed_turn, queue_size=1)
        self._publish_next()

    def _publish_next(self):
        if self.route:
            self.pub.publish(String(data=self.route[0]))

    def cb_executed_turn(self, msg):
        # El navegador ejecuto un giro: avanzamos la ruta y publicamos el proximo.
        if not self.route:
            return
        ejecutado = msg.data or self.route[0]
        self.route.pop(0)
        if self.loc:
            try:
                self.loc.update(ejecutado)
            except ValueError as e:
                rospy.logwarn("Localizacion desincronizada: %s", e)
        self._publish_next()

if __name__ == "__main__":
    RoutePlannerNode()
    rospy.spin()
