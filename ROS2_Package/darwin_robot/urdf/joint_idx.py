index = {
    "j_pan": 19,
    "j_pelvis_l": 8,
    "j_pelvis_r": 7,
    "j_shoulder_l": 2,
    "j_shoulder_r": 1,
    "j_tilt": 20,
    "j_thigh1_l": 10,
    "j_thigh1_r": 9,
    "j_high_arm_l": 4,
    "j_high_arm_r": 3,
    "j_thigh2_l": 12,
    "j_thigh2_r": 11,
    "j_low_arm_l": 6,
    "j_low_arm_r": 5,
    "j_tibia_l": 14,
    "j_tibia_r": 13,
    "j_gripper_l": 22,  # not real
    "j_gripper_r": 21,  # not real
    "j_ankle1_l": 16,
    "j_ankle1_r": 15,
    "j_ankle2_l": 18,
    "j_ankle2_r": 17,
}

positions = {
    "j_pan": 0.0,
    "j_tilt": 0.0,
    "j_pelvis_l": 0.0,
    "j_thigh1_l": 0.0,
    "j_thigh2_l": 0.4,
    "j_tibia_l": -0.6,
    "j_ankle1_l": -0.2,
    "j_ankle2_l": 0.0,
    "j_pelvis_r": 0.0,
    "j_thigh1_r": 0.0,
    "j_thigh2_r": -0.4,
    "j_tibia_r": 0.6,
    "j_ankle1_r": 0.2,
    "j_ankle2_r": 0.0,
    "j_shoulder_l": 0.0,
    "j_high_arm_l": 0.7,
    "j_low_arm_l": -1.0,
    "j_gripper_l": -0.99,
    "j_shoulder_r": 0.0,
    "j_high_arm_r": 0.7,
    "j_low_arm_r": 1.0,
    "j_gripper_r": -0.99,
}

joint_names = [
    "j_pan",
    "j_pelvis_l",
    "j_pelvis_r",
    "j_shoulder_l",
    "j_shoulder_r",
    "j_tilt",
    "j_thigh1_l",
    "j_thigh1_r",
    "j_high_arm_l",
    "j_high_arm_r",
    "j_thigh2_l",
    "j_thigh2_r",
    "j_low_arm_l",
    "j_low_arm_r",
    "j_tibia_l",
    "j_tibia_r",
    "j_gripper_l",
    "j_gripper_r",
    "j_ankle1_l",
    "j_ankle1_r",
    "j_ankle2_l",
    "j_ankle2_r",
]

default_positions = [0.0 for _ in range(22)]

for name in joint_names:
    default_positions[index[name] - 1] = positions[name]
print(default_positions[:18])
