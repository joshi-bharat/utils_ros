from nav_msgs.msg import Odometry
import rospy

svin_topic = '/okvis_node/okvis_odometry'
svin_file = open('svin_traj.txt', 'w')
uber_topic = '/pose_graph_node/uber_odometry'
uber_file = open('uber_traj.txt', 'w')
prim_topic = '/pose_graph_node/prim_odometry'
prim_file = open('prim_traj.txt', 'w')


def uber_callback(odom_msg):
    position = odom_msg.pose.pose.position
    quaterion = odom_msg.pose.pose.orientation
    stamp = str(odom_msg.header.stamp.to_sec())
    uber_file.write('{} {} {} {} {} {} {} {}\n'.format(stamp, position.x, position.y, position.z,
                                                       quaterion.x, quaterion.y, quaterion.z, quaterion.w))


def svin_callback(odom_msg):
    position = odom_msg.pose.pose.position
    quaterion = odom_msg.pose.pose.orientation
    stamp = str(odom_msg.header.stamp.to_sec())
    svin_file.write('{} {} {} {} {} {} {} {}\n'.format(stamp, position.x, position.y, position.z,
                                                       quaterion.x, quaterion.y, quaterion.z, quaterion.w))


def prim_callback(odom_msg):
    position = odom_msg.pose.pose.position
    quaterion = odom_msg.pose.pose.orientation
    stamp = str(odom_msg.header.stamp.to_sec())
    prim_file.write('{} {} {} {} {} {} {} {}\n'.format(stamp, position.x, position.y, position.z,
                                                       quaterion.x, quaterion.y, quaterion.z, quaterion.w))


if __name__ == '__main__':
    rospy.init_node('odom_subscriber', anonymous=True)

    svin_sub = rospy.Subscriber(svin_topic, Odometry, svin_callback, queue_size=100)
    prim_sub = rospy.Subscriber(prim_topic, Odometry, prim_callback, queue_size=100)
    uber_sub = rospy.Subscriber(uber_topic, Odometry, uber_callback, queue_size=100)

    while not rospy.is_shutdown():
        rospy.spin()

    svin_file.close()
    uber_file.close()
    prim_file.close()
