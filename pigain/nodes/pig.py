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

        self.visualize_mean  = rospy.get_param('~visualize_mean',  False)
        self.visualize_sigma = rospy.get_param('~visualize_sigma', False)
        self.visualize_pts   = rospy.get_param('~visualize_pts',   False)

        self.gain_sub = rospy.Subscriber('gain_node', Node, self.gain_callback)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.marker_pub = rospy.Publisher('pig_markers', MarkerArray, queue_size=10)
        self.mean_pub = rospy.Publisher('mean_markers', MarkerArray, queue_size=10)
        self.sigma_pub = rospy.Publisher('sigma_markers', MarkerArray, queue_size=10)

        if self.visualize_pts:
            rospy.Timer(rospy.Duration(1), self.rviz_callback)
        if self.visualize_mean or self.visualize_sigma:
            rospy.Timer(rospy.Duration(1), self.evaluate)

        self.minx = -100
        self.maxx =  100
        self.miny = -100
        self.maxy =  100
        self.minz = -100
        self.maxz =  100

        self.x = None
        self.y = None
        self.z = None

        self.range = 6
        self.bbx = (self.minx, self.miny, self.minz, self.maxx, self.maxy, self.maxz)
        self.hyperparam = gp.HyperParam(l = 1, sigma_f = 1, sigma_n = 0.1)


        # Create r-tree
        p = index.Property()
        p.dimension = 3
        self.idx = index.Index(properties = p)
        self.id = 0

        rospy.Timer(rospy.Duration(1), self.reevaluate_timer_callback)

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    def reevaluate_timer_callback(self, event):
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
            if(item.object.gain > 1):
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

    def gain_callback(self, msg):
        self.idx.insert(self.id, (msg.position.x, msg.position.y, msg.position.z), obj=msg)
        self.id += 1

    def query_server(self, req):
        bbx = (req.point.x-1, req.point.y-1, req.point.z-1, 
               req.point.x+1, req.point.y+1, req.point.z+1)
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


    def best_node_srv_callback(self, req):
        bbx = (self.minx, self.miny, self.minz, self.maxx, self.maxy, self.maxz)
        hits = self.idx.intersection(bbx, objects=True)

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


    def evaluate(self, event):
        y = np.empty((0))
        x = np.empty((0,3))
        xstar = np.empty((0,3))
        hits = self.idx.intersection(self.bbx, objects=True)
        for item in hits:
            y = np.append(y, [item.object.gain], axis=0)
            x = np.append(x, [[item.object.position.x, item.object.position.y, item.object.position.z]], axis = 0)

        # nx, ny, nz = (32, 32, 6)
        nx, ny, nz = (10, 10, 4)
        xt = np.linspace(-10, 10, nx)
        yt = np.linspace(-5,  5, ny)
        zt = np.linspace(0.5, 2.5, nz)

        for xx in xt:
            for yy in yt:
                for zz in zt:
                    xstar = np.append(xstar, [[xx, yy, zz]], axis = 0)
                    # print(str(xx) + " " + str(yy) + " " +str(zz))

        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)
        mean_markers = MarkerArray()
        sigma_markers = MarkerArray()
        for id, pts in enumerate(zip(xstar, mean, sigma)):
            if(pts[2] < 0.5):
                mean_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[1] / 16))
                sigma_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[2] * 2))
        
        self.mean_pub.publish(mean_markers)
        self.sigma_pub.publish(sigma_markers)




    def rviz_callback(self, event):
        markers = MarkerArray()
        hits = self.idx.intersection(self.bbx, objects=True)
        for item in hits:
            markers.markers.append(self.node_to_marker(item.id, item.object))
        
        self.marker_pub.publish(markers)


    def np_array_to_marker(self, id, a, v=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = v
        marker.color.g = 0
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = a[0]
        marker.pose.position.y = a[1]
        marker.pose.position.z = a[2]
        marker.lifetime = rospy.Time(10)

        return marker


    def node_to_marker(self, id, node):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = node.gain / 16
        marker.color.g = 0.5
        marker.color.b = 0.0
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
