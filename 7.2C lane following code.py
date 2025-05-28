import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import BoolStamped
import time

class StopSignController:
    def __init__(self):
        rospy.init_node('stop_sign_controller')
        
        self.pub_car_cmd = rospy.Publisher("/birdie/car_cmd", Twist2DStamped, queue_size=1)
        self.sub_stop_sign = rospy.Subscriber("/birdie/stop_sign_detected", BoolStamped, self.stop_sign_callback)
        self.sub_lane_cmd = rospy.Subscriber("/birdie/lane_controller/car_cmd", Twist2DStamped, self.lane_cmd_callback)

        self.stop_detected = False
        self.ignore_stop_sign = False
        self.state = "LANE_FOLLOWING"
        self.stop_time = None
        self.ignore_duration = 5  # seconds to ignore Stop Sign after stopping

    def stop_sign_callback(self, msg):
        if msg.data and not self.ignore_stop_sign and self.state == "LANE_FOLLOWING":
            rospy.loginfo("Stop sign detected")
            self.state = "STOPPING"
            self.stop_detected = True

    def lane_cmd_callback(self, msg):
        if self.state == "LANE_FOLLOWING":
            self.pub_car_cmd.publish(msg)
        elif self.state == "STOPPING":
            self.publish_stop()
            self.state = "WAITING"
            self.stop_time = rospy.Time.now().to_sec()
        elif self.state == "WAITING":
            if rospy.Time.now().to_sec() - self.stop_time > 3:  # 3-second wait
                rospy.loginfo("Done waiting. Resume lane following and ignore stop sign temporarily.")
                self.ignore_stop_sign = True
                self.ignore_start_time = rospy.Time.now().to_sec()
                self.state = "LANE_FOLLOWING"
        elif self.ignore_stop_sign:
            if rospy.Time.now().to_sec() - self.ignore_start_time > self.ignore_duration:
                self.ignore_stop_sign = False

    def publish_stop(self):
        stop_cmd = Twist2DStamped()
        stop_cmd.v = 0
        stop_cmd.omega = 0
        self.pub_car_cmd.publish(stop_cmd)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = StopSignController()
    controller.run()
