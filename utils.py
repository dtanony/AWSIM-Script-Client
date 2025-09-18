from geometry_msgs.msg import Pose, PoseWithCovariance

def dict_to_ros_pose(input):
    pose = Pose()
    pose.position.x = input['position']['x']
    pose.position.y = input['position']['y']
    pose.position.z = input['position']['z']
    pose.orientation.x = input['quaternion']['x']
    pose.orientation.y = input['quaternion']['y']
    pose.orientation.z = input['quaternion']['z']
    pose.orientation.w = input['quaternion']['w']
    return pose
