from human_arm.hand_scripts.scripts.mangekyo import main
import rospy
from nav_msgs.msg import Odometry
import tf


def main():
    try:
        (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        listener.lookupTransform()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("error in transform listning")
    
    # odom_publisher.publish()





if __name__=="__main__":
    rospy.init_node("pub_odom", anonymous=True)
    odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=10)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        main()
