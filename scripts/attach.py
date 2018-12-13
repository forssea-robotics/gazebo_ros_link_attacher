#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /ros_link_attacher_plugin/attach")
    attach_srv = rospy.ServiceProxy('/ros_link_attacher_plugin/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /ros_link_attacher_plugin/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "cube2"
    req.link_name_2 = "link"
    req.pos.x = 2.0
    req.pos.y = 0.0
    req.pos.z = 0.0

    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """

    rospy.loginfo("Attaching cube2 and cube3")
    req = AttachRequest()
    req.model_name_1 = "cube2"
    req.link_name_1 = "link"
    req.model_name_2 = "cube3"
    req.link_name_2 = "link"
    req.pos.x = 2.0
    req.pos.y = 0.0
    req.pos.z = 0.0

    attach_srv.call(req)
