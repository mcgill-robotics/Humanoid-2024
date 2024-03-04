# Each joint motor has a float32 array with their feedback values
# Each array has 3 values in this order: [position, velocity, load]
float32[3] left_leg_ankle_fb
float32[3] left_leg_knee_fb
float32[3] left_leg_hip_roll_fb
float32[3] left_leg_hip_pitch_fb
float32[3] left_leg_hip_yaw_fb
float32[3] right_leg_ankle_fb
float32[3] right_leg_knee_fb
float32[3] right_leg_hip_roll_fb
float32[3] right_leg_hip_pitch_fb
float32[3] right_leg_hip_yaw_fb
float32[3] torso_roll_fb
float32[3] torso_yaw_fb
float32[3] left_arm_elbow_fb
float32[3] right_arm_elbow_fb
float32[3] left_arm_shoulder_fb
float32[3] right_arm_shoulder_fb