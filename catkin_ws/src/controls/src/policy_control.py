import rospy
from humanoid_msgs.msg import ServoSetpoints
import numpy as np
import torch
from ppo_inference_model import ActorCritic
from state_generator import PPOStateGenerator, model_output_mapping
    
def generateControl():
    stateGenerator.generateStateObservation()
    state = np.concatenate(stateGenerator.stacked_state).reshape(1, -1)
    state = torch.FloatTensor(state).to(device)
    action = ppo.act(state)
    stateGenerator.updateAction(action)
    
    setpoints = ServoSetpoints()
    
    setpoints.left_ankle_pitch = action[model_output_mapping.index("left_ankle_pitch")]
    setpoints.left_knee_pitch = action[model_output_mapping.index("left_knee")]
    setpoints.left_hip_roll = action[model_output_mapping.index("left_hip_roll")]
    setpoints.left_hip_pitch = action[model_output_mapping.index("left_hip_pitch")]
    setpoints.left_hip_yaw = action[model_output_mapping.index("left_hip_yaw")]
    setpoints.right_ankle_pitch = action[model_output_mapping.index("right_ankle_pitch")]
    setpoints.right_knee_pitch = action[model_output_mapping.index("right_knee_pitch")]
    setpoints.right_hip_roll = action[model_output_mapping.index("right_hip_roll")]
    setpoints.right_hip_pitch = action[model_output_mapping.index("right_hip_pitch")]
    setpoints.right_hip_yaw = action[model_output_mapping.index("right_hip_yaw")]
    setpoints.torso_roll = action[model_output_mapping.index("torso_roll")]
    setpoints.torso_yaw = action[model_output_mapping.index("torso_yaw")]
    setpoints.left_elbow_pitch = action[model_output_mapping.index("left_elbow")]
    setpoints.right_elbow_pitch = action[model_output_mapping.index("right_elbow")]
    setpoints.left_shoulder_pitch = action[model_output_mapping.index("left_shoulder_pitch")]
    setpoints.right_shoulder_pitch = action[model_output_mapping.index("right_shoulder_pitch")]
    
    setpoint_publisher.publish(setpoints)

stateGenerator = PPOStateGenerator()

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
if not torch.cuda.is_available(): print("WARN: Running policy on CPU!")

ppo = ActorCritic(255, 16)
ppo.load(rospy.get_param("~model_checkpoint_path"))

if __name__ == '__main__':
    rospy.init_node('policy_controller')
    setpoint_publisher = rospy.Publisher('/servo/setpoints', ServoSetpoints, queue_size=1)
    
    timer = rospy.Timer(rospy.Duration(float(rospy.get_param("~control_interval"))), generateControl)

    rospy.spin()