import pybullet as p
from time import sleep
import pybullet_data

DEBUG = True

# Connect to the PyBullet physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

# Load the plane and the GO2 robot
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1.2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
go2_id = p.loadURDF("/go2_description/urdf/go2_description.urdf", startPos, startOrientation, useFixedBase=True)
go2Pos, go2Orn = p.getBasePositionAndOrientation(go2_id)

# print the joints of the robot
if DEBUG:
    num_joints = p.getNumJoints(go2_id)
    print(f"Number of joints: {num_joints}")
    for joint in range(num_joints):
        joint_info = p.getJointInfo(go2_id, joint)
        print(f"Joint {joint}: {joint_info[1].decode('utf-8')} with angle: {p.getJointState(go2_id, joint)}")

# static list of the joints of the robot
leg_joints = {
    "Head_upper_joint": 0,
    "Head_lower_joint": 1,
    "FL_hip_joint": 2,
    "FL_thigh_joint": 3,
    "FL_calf_joint": 4,
    "FL_calflower_joint": 5,
    "FL_calflower1_joint": 6,
    "FL_foot_joint": 7,
    "FL_calf_rotor_joint": 8,
    "FL_thigh_rotor_joint": 9,
    "FL_hip_rotor_joint": 10,
    "FR_hip_joint": 11,
    "FR_thigh_joint": 12,
    "FR_calf_joint": 13,
    "FR_calflower_joint": 14,
    "FR_calflower1_joint": 15,
    "FR_foot_joint": 16,
    "FR_calf_rotor_joint": 17,
    "FR_thigh_rotor_joint": 18,
    "FR_hip_rotor_joint": 19,
    "RL_hip_joint": 20,
    "RL_thigh_joint": 21,
    "RL_calf_joint": 22,
    "RL_calflower_joint": 23,
    "RL_calflower1_joint": 24,
    "RL_foot_joint": 25,
    "RL_calf_rotor_joint": 26,
    "RL_thigh_rotor_joint": 27,
    "RL_hip_rotor_joint": 28,
    "RR_hip_joint": 29,
    "RR_thigh_joint": 30,
    "RR_calf_joint": 31,
    "RR_calflower_joint": 32,
    "RR_calflower1_joint": 33,
    "RR_foot_joint": 34,
    "RR_calf_rotor_joint": 35,
    "RR_thigh_rotor_joint": 36,
    "RR_hip_rotor_joint": 37,
    "imu_joint": 38,
    "radar_joint": 39,
}

walk_sequence = [
    {
        # "FR_thigh_joint": -1.0,
        "FR_calf_joint": 0.2,
        # "FR_calflower1_joint": 1.0,
    },
    # {
    #     "FR_thigh_joint": -1.0,
    #     "FR_calf_joint": 0.0,
    # },
]

timing_sequence = [
    2.0,
    # 2.0,
]

try:
    for _ in range(int(1.0*240)):
            p.stepSimulation()
            sleep(1 / 240)

    for i in range(len(walk_sequence)):
        phase = walk_sequence[i]
        indicies = [leg_joints[joint] for joint in phase.keys()]
        target_positions = list(phase.values())
        p.setJointMotorControlArray(
             go2_id, 
             jointIndices=indicies, 
             controlMode=p.POSITION_CONTROL, 
             targetPositions=target_positions)

        for _ in range(int(timing_sequence[i]*240)):
            p.stepSimulation()
            sleep(1 / 240)
    while True:
        p.stepSimulation()
        sleep(1 / 240)
except KeyboardInterrupt:
    print("Closing the simulation")
    
p.disconnect()