import rospy
from humanoid_msgs.msg import ServoCommand
import numpy as np
import torch
import torch.nn as nn
from state_generator import PPOStateGenerator, model_output_mapping, joint_limits
    
# NOTE: taken from Humanoid-MuJoCo repo! If any changes are made there, they must be reflected here!
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()

        # actor
        self.actor = nn.Sequential(
                        nn.Linear(state_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, 64),
                        nn.Tanh(),
                        nn.Linear(64, action_dim),
                        nn.Tanh()
                    )
        # critic
        self.critic = nn.Sequential(
                        nn.Linear(state_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, 64),
                        nn.Tanh(),
                        nn.Linear(64, 1)
                    )
    
    def act(self, state):
        action = self.actor(state)
        return action.detach().cpu().numpy()
    
    def load(self, checkpoint_path):
        self.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))
    
def scaleActionToJointLimits(action):
    scaled_action = []
    for i in range(len(action)):
        joint_range = joint_limits[model_output_mapping[i]]
        if action[i] < 0:
            scaled_cmd = 150 + action[i] * joint_range[0]
        else:
            scaled_cmd = 150 + action[i] * joint_range[1]
        scaled_action.append(scaled_cmd)
    return np.array(scaled_action)
    
def generateControl():
    stateGenerator.generateStateObservation()
    state = np.concatenate(stateGenerator.stacked_state).reshape(1, -1)
    state = torch.FloatTensor(state).to(device)
    action = ppo.act(state)
    stateGenerator.updateAction(action)
    
    scaled_action = scaleActionToJointLimits(action)
    
    setpoints = ServoCommand()
    
    setpoints.left_leg_ankle_setpoint = scaled_action[model_output_mapping.index("left_ankle_pitch")]
    setpoints.left_leg_knee_setpoint = scaled_action[model_output_mapping.index("left_knee")]
    setpoints.left_leg_hip_roll_setpoint = scaled_action[model_output_mapping.index("left_hip_roll")]
    setpoints.left_leg_hip_pitch_setpoint = scaled_action[model_output_mapping.index("left_hip_pitch")]
    setpoints.left_leg_hip_yaw_setpoint = scaled_action[model_output_mapping.index("left_hip_yaw")]
    setpoints.right_leg_ankle_setpoint = scaled_action[model_output_mapping.index("right_ankle_pitch")]
    setpoints.right_leg_knee_setpoint = scaled_action[model_output_mapping.index("right_knee_pitch")]
    setpoints.right_leg_hip_roll_setpoint = scaled_action[model_output_mapping.index("right_hip_roll")]
    setpoints.right_leg_hip_pitch_setpoint = scaled_action[model_output_mapping.index("right_hip_pitch")]
    setpoints.right_leg_hip_yaw_setpoint = scaled_action[model_output_mapping.index("right_hip_yaw")]
    setpoints.torso_roll_setpoint = scaled_action[model_output_mapping.index("torso_roll")]
    setpoints.torso_yaw_setpoint = scaled_action[model_output_mapping.index("torso_yaw")]
    setpoints.left_arm_elbow_setpoint = scaled_action[model_output_mapping.index("left_elbow")]
    setpoints.right_arm_elbow_setpoint = scaled_action[model_output_mapping.index("right_elbow")]
    setpoints.left_arm_shoulder_setpoint = scaled_action[model_output_mapping.index("left_shoulder_pitch")]
    setpoints.right_arm_shoulder_setpoint = scaled_action[model_output_mapping.index("right_shoulder_pitch")]
    
    setpoint_publisher.publish(setpoints)

stateGenerator = PPOStateGenerator()

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
if not torch.cuda.is_available(): print("WARN: Running policy on CPU!")

ppo = ActorCritic(255, 16)
ppo.load(rospy.get_param("~model_checkpoint_path"))

if __name__ == '__main__':
    rospy.init_node('policy_controller')
    setpoint_publisher = rospy.Publisher('/servosCommand', ServoCommand, queue_size=1)
    
    timer = rospy.Timer(rospy.Duration(float(rospy.get_param("~control_interval"))), generateControl)

    rospy.spin()