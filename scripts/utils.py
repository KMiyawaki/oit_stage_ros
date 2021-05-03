import rospy
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class RosWinMessenger(object):
    def __init__(self, to_win_publisher, topic_name_from_win, publish_rate=1, wait_message_timeout=1):
        self.to_win_publisher = to_win_publisher
        self.topic_name_from_win = topic_name_from_win
        self.wait_message_timeout = wait_message_timeout
        self.sleep_time = 1 / publish_rate

    def wait_response(self, publish_msg=None, target_words=None, timeout=None):
        tm = rospy.get_time()
        message = None
        while message is None:
            if timeout is not None and rospy.get_time() - tm >= timeout:
                return None
            if publish_msg is not None:
                self.to_win_publisher.publish(publish_msg)
                rospy.sleep(self.sleep_time)
            message = wait_string_message(
                self.topic_name_from_win, self.wait_message_timeout)
            if message is None:
                continue
            if target_words is None:
                return message
            for word in target_words:
                if word in message:
                    return message
        return message


def wait_string_message(topic_name, timeout):
    try:
        recv = rospy.wait_for_message(topic_name, String, timeout)
        if recv:
            return recv.data
    except Exception as e:
        rospy.logdebug(str(e))
    return None


def make_navigation_goal(x, y, theta):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    return goal


def navigation(action_server, x, y, theta, timeout=120):
    node_name = rospy.get_name()
    rospy.loginfo("%s:Sending goal", node_name)
    action_server.send_goal(make_navigation_goal(x, y, theta))
    finished = action_server.wait_for_result(rospy.Duration(timeout))
    state = action_server.get_state()
    if finished:
        rospy.loginfo("%s:Finished: (%d)", node_name, state)
    else:
        rospy.loginfo("%s:Time out: (%d)", node_name, state)
