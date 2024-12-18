#!/usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def add_object(object_id, box_pose, box_size, color) -> None:
    """
    Add a box to the planning scene
    Args:
        object_id (str): name of the object
        box_pose (Pose): position=Point(x, y, z), orientation=Quaternion(x, y, z, w)
        box_size (list): [x, y, z]
        colo
    """
    scene = PlanningSceneInterface()

    rospy.sleep(2)  # Allow the scene to initialize

    # Convert Pose to PoseStamped
    box_pose_stamped = PoseStamped()
    box_pose_stamped.header.frame_id = "world"
    box_pose_stamped.pose = box_pose

    # Add the object to the scene
    scene.add_box(object_id, box_pose_stamped, box_size)
    # Publish a marker to set the color of the object
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = hash(object_id)
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = box_pose
    marker.scale.x = box_size[0]
    marker.scale.y = box_size[1]
    marker.scale.z = box_size[2]
    marker.color = color
    marker.lifetime = rospy.Duration()

    marker_pub.publish(marker)
    rospy.loginfo(f"Set color for {object_id}")

if __name__ == "__main__":
    rospy.init_node('add_object_to_scene')

    try:
        # Define colors
        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)

        # Add the first box
        pose1 = Pose(Point(0.60, -0.20, 0.10), Quaternion(*quaternion_from_euler(0.00, -0.00, 0.00)))
        add_object(object_id="box1", box_pose=pose1, box_size=[0.40, 0.65, 0.40], color=green)

        # Add the second box
        pose2 = Pose(Point(-0.25, 0.60, 0.00), Quaternion(*quaternion_from_euler(0.00, 0.00, 0.00)))
        add_object(object_id="box2", box_pose=pose2, box_size=[0.65, 0.40, 0.20], color=green)

        # Add the third box
        pose3 = Pose(Point(-0.10, 0.60, 0.10), Quaternion(*quaternion_from_euler(0.00, 0.00, 0.00)))
        add_object(object_id="box3", box_pose=pose3, box_size=[0.05, 0.20, 0.20], color=red)

        # Add the fifth box
        pose4 = Pose(Point(-0.20, 0.40, 0.10), Quaternion(*quaternion_from_euler(0.00, 0.00, 0.00)))
        add_object(object_id="box4", box_pose=pose3, box_size=[0.30, 0.05, 0.20], color=red)


        # rospy.sleep(2)

        # # Add the fourth box
        # pose5 = Pose(Point(-0.50, 0.60, 0.10), Quaternion(*quaternion_from_euler(0.00, 0.00, 0.00)))
        # add_object(object_id="box5", box_pose=pose3, box_size=[0.05, 0.20, 0.20], color=red)

        
        # rospy.sleep(2)
        
        # # Add the sixth box
        # pose6 = Pose(Point(-0.28, 0.70, 0.10), Quaternion(*quaternion_from_euler(0.00, 0.00, 0.00)))
        # add_object(object_id="box6", box_pose=pose3, box_size=[0.30, 0.05, 0.20], color=red)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass