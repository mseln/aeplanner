#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

from pigain.msg import Node
from pigain.srv import Query, QueryResponse
from pigain.srv import BestNode, BestNodeResponse

from aeplanner.srv import Reevaluate

import numpy as np
from rtree import index

import gp

class PIGain:
    def __init__(self):
        self.s = rospy.Service('gp_query_server', Query, self.query_server)
        self.best_node_srv = rospy.Service('best_node_server', BestNode, self.best_node_srv_callback)
        self.reevaluate_client = rospy.ServiceProxy('reevaluate', Reevaluate)

        self.visualize_mean  = rospy.get_param('~visualize/mean',  False)
        self.visualize_sigma = rospy.get_param('~visualize/sigma', False)
        self.visualize_pts   = rospy.get_param('~visualize/pts',   False)
        self.resolution      = rospy.get_param('~visualize/resolution',   1)

        self.gain_sub = rospy.Subscriber('gain_node', Node, self.gain_callback)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.marker_pub = rospy.Publisher('pig_markers', MarkerArray, queue_size=10)
        self.mean_pub = rospy.Publisher('mean_markers', MarkerArray, queue_size=10)
        self.sigma_pub = rospy.Publisher('sigma_markers', MarkerArray, queue_size=10)

        if self.visualize_pts:
            rospy.Timer(rospy.Duration(1), self.rviz_callback)
        if self.visualize_mean or self.visualize_sigma:
            rospy.Timer(rospy.Duration(5), self.evaluate)

        # Get environment boundaries 
        try:
            self.min = rospy.get_param('boundary/min')
            self.max = rospy.get_param('boundary/max')
        except KeyError:
            rospy.logwarn("Boundary parameters not specified")
            rospy.logwarn("Defaulting to (-100, -100, 0), (100, 100, 3)...")
            self.min = [-100, -100, 0]
            self.max = [ 100,  100, 3]

        try:
            self.range = rospy.get_param('aep/gain/r_max') * 2
        except KeyError:
            rospy.logwarn("Range max parameter not specified")
            rospy.logwarn("Defaulting to 8 m...")

        self.bbx = (self.min[0], self.min[1], self.min[2], self.max[0], self.max[1], self.max[2])

        self.x = None
        self.y = None
        self.z = None

        self.hyperparam = gp.HyperParam(l = 1, sigma_f = 1, sigma_n = 0.1)

        # Create r-tree
        p = index.Property()
        p.dimension = 3
        self.idx = index.Index(properties = p)
        self.id = 0

        rospy.Timer(rospy.Duration(5), self.reevaluate_timer_callback)

    """ Save current pose of agent """
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    """ Reevaluate gain in all cached nodes that are closer to agent than self.range """
    def reevaluate_timer_callback(self, event):
        rospy.loginfo("reevaluate start")
        if self.x is None or self.y is None or self.z is None:
            rospy.logwarn("No position received yet...")
            rospy.logwarn("Make sure that 'pose' has been correctly mapped and that it is being published")
            return

        bbx = (self.x-self.range, self.y-self.range, self.z-self.range, 
               self.x+self.range, self.y+self.range, self.z+self.range)

        hits = self.idx.intersection(bbx, objects=True)

        reevaluate_list = []
        reevaluate_position_list = []
        for item in hits:
            if(item.object.gain > 2):
                reevaluate_position_list.append(item.object.position)
                reevaluate_list.append(item)
        try:
            res = self.reevaluate_client(reevaluate_position_list)
        except rospy.ServiceException, e:
            rospy.logerr("Calling reevaluate service failed")
            return

        for i, item in enumerate(reevaluate_list):
            item.object.gain = res.gain[i]
            item.object.yaw = res.yaw[i]

            self.idx.delete(item.id, (item.object.position.x, item.object.position.y, item.object.position.z))
            self.idx.insert(item.id, (item.object.position.x, item.object.position.y, item.object.position.z), obj=item.object)

        rospy.loginfo("reevaluate done")

    """ Insert node with estimated gain in rtree """
    def gain_callback(self, msg):
        self.idx.insert(self.id, (msg.position.x, msg.position.y, msg.position.z), obj=msg)
        self.id += 1

    """ Handle query to Gaussian Process """
    def query_server(self, req):
        bbx = (req.point.x-2, req.point.y-2, req.point.z-2, 
               req.point.x+2, req.point.y+2, req.point.z+2)
        y = np.empty((0))
        x = np.empty((0,3))
        hits = self.idx.intersection(bbx, objects=True)
        nearest = self.idx.nearest(bbx, 1, objects=True)

        for item in hits:
            y = np.append(y, [item.object.gain], axis=0)
            x = np.append(x, [[item.object.position.x, item.object.position.y, item.object.position.z]], axis = 0)

        yaw = 0
        for item in nearest:
            yaw = item.object.yaw

        if y.shape[0] == 0:
            response = QueryResponse()
            response.mu = 0
            response.sigma = 1
            return response

        xstar = np.array([[req.point.x, req.point.y, req.point.z]])

        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)

        response = QueryResponse()
        response.mu = mean
        response.sigma = sigma
        response.yaw = yaw

        return response

    """ Return all nodes with gain higher than req.threshold """
    def best_node_srv_callback(self, req):
        hits = self.idx.intersection(self.bbx, objects=True)

        best_gain = -1
        best_pose = None
        response = BestNodeResponse()
        for item in hits:
            if item.object.gain > req.threshold:
                response.best_node.append(item.object.position)
            if item.object.gain > best_gain:
                best_gain = item.object.gain

        response.gain = best_gain
        return response

    """ Evaluate potential information gain function over grid and publish it in rviz """
    def evaluate(self, event):
        y = np.empty((0))
        x = np.empty((0,3))
        xstar = np.empty((0,3))
        hits = self.idx.intersection(self.bbx, objects=True)
        for item in hits:
            y = np.append(y, [item.object.gain], axis=0)
            x = np.append(x, [[item.object.position.x, item.object.position.y, item.object.position.z]], axis = 0)

        xt = np.arange(self.min[0], self.max[0], self.resolution)
        yt = np.arange(self.min[1], self.max[1], self.resolution)
        zt = [1]

        for xx in xt:
            for yy in yt:
                for zz in zt:
                    xstar = np.append(xstar, [[xx, yy, zz]], axis = 0)

        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)
        mean_markers = MarkerArray()
        sigma_markers = MarkerArray()
        for id, pts in enumerate(zip(xstar, mean, sigma)):
            mean_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[1], max(1-pts[2], 0)))
            # sigma_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[2] * 2))
        
        self.mean_pub.publish(mean_markers)
        self.sigma_pub.publish(sigma_markers)

    """ Publish all cached nodes in rviz """
    def rviz_callback(self, event):
        markers = MarkerArray()
        hits = self.idx.intersection(self.bbx, objects=True)
        for item in hits:
            markers.markers.append(self.node_to_marker(item.id, item.object))
        
        self.marker_pub.publish(markers)


    def np_array_to_marker(self, id, p, v=0, a=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.color.r = v / 72.0
        marker.color.g = 0 
        marker.color.b = 0.5
        marker.color.a = a
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = p[0]
        marker.pose.position.y = p[1]
        marker.pose.position.z = p[2]
        marker.lifetime = rospy.Time(10)

        return marker


    def node_to_marker(self, id, node):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = node.gain / 72.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z
        marker.lifetime = rospy.Time(1.2)

        return marker




if __name__ == '__main__':
    rospy.init_node('pigain', anonymous=True)
    pigain = PIGain()
    rospy.spin()
