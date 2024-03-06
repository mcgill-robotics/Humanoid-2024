import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from humanoid_msgs.msg import UnityState
import quaternion


def unityStateCb(msg):
    unityQuat = msg.quat
    unityAngVel = msg.ang_vel
    unityLinAccel = msg.lin_accel
    unityGlobalVel = msg.global_vel

    quat  = q_NWU_NED * unityQuat
    quat = np.quaternion(quat.w, quat.x, quat.y, quat.z)
    pub_quat = Quaternion(quat.x, quat.y, quat.z, quat.w)

    global_vel = Vector3(x=unityGlobalVel.z, y=-unityGlobalVel.x, z=unityGlobalVel.y)
    
    global_lin_accel = np.array[unityLinAccel.z, -unityLinAccel.x, unityLinAccel.y]
    local_lin_accel = quat * global_lin_accel * quat.conj()
    pub_local_lin = Vector3(x=local_lin_accel.x, y=local_lin_accel.y, z=local_lin_accel.z)

    global_ang_vel = np.array[unityAngVel.z, -unityAngVel.x, unityAngVel.y]
    local_ang_vel = quat * global_ang_vel * quat.conj() 
    pub_ang_vel = Vector3(x=local_ang_vel.x, y=local_ang_vel.y, z=local_ang_vel.z)

    pub_global_vel.publish(global_vel)
    pub_quat.publish(pub_quat)
    pub_ang_vel.publish(pub_ang_vel)
    pub_local_lin.publish(pub_local_lin)
    

if __name__ == '__main__':
    rospy.init_node('unity_bridge')

    # subscribers
    rospy.Subscriber('/unity/state', UnityState, unityStateCb)

    # publishers
    pub_local_lin = rospy.Publisher('/state/local_lin_accel', Vector3, queue_size=1)
    pub_quat = rospy.Publisher('/state/quat', Vector3, queue_size=1)
    pub_ang_vel = rospy.Publisher('/state/ang_vel', Vector3, queue_size=1)
    pub_global_vel = rospy.Publisher('/state/global_vel', Vector3, queue_size=1)

    # REFERENCE FRAME DEFINITIONS
    q_NWU_NED = np.quaternion(0,1,0,0)

    rospy.spin()