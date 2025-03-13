import sys
import rclpy
import moveit_commander
# from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_cylinder(scene, name, height, radius, x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    scene.add_cylinder(name, pose, height, radius)

def attach_object(scene, group, object_name, touch_links):
    eef_link = group.get_end_effector_link()
    scene.attach_box(eef_link, object_name, touch_links=touch_links)

def detach_object(scene, group, object_name):
    eef_link = group.get_end_effector_link()
    scene.remove_attached_object(eef_link, name=object_name)

def remove_object(scene, object_name):
    scene.remove_world_object(object_name)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("add_cylinder_objects", anonymous=True)
    
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    
    rospy.sleep(2)  # Allow MoveIt to update
    
    # Add Can (r = 6.62cm, h = 12.2cm)
    add_cylinder(scene, "can", height=0.122, radius=0.0662, x=0.5, y=0.0, z=0.061)
    
    # Add Bottle (r = 5.6cm, h = 18.8cm)
    add_cylinder(scene, "bottle", height=0.188, radius=0.056, x=0.3, y=-0.2, z=0.094)
    
    rospy.sleep(1)
    
    # Attach the can to the gripper
    attach_object(scene, group, "can", touch_links=["gripper_link"])
    rospy.sleep(1)
    
    # Detach and remove the can
    detach_object(scene, group, "can")
    rospy.sleep(1)
    remove_object(scene, "can")
    
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
