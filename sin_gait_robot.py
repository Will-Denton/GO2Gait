import pybullet as p
from time import sleep, time
import pybullet_data
import math

DEBUG = False

# Connect to the PyBullet physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

# Load the plane and the GO2 robot
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=5.0)

startPos = [0, 0, 0.8]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
go2_id = p.loadURDF("/go2_description/urdf/go2_description.urdf", startPos, startOrientation, useFixedBase=False    )
go2Pos, go2Orn = p.getBasePositionAndOrientation(go2_id)

# static list of the joints of the robot
leg_joints = {
    "FL_hip_joint": 2,
    "FL_thigh_joint": 3,
    "FL_calf_joint": 4,
    "FR_hip_joint": 11,
    "FR_thigh_joint": 12,
    "FR_calf_joint": 13,
    "RL_hip_joint": 20,
    "RL_thigh_joint": 21,
    "RL_calf_joint": 22,
    "RR_hip_joint": 29,
    "RR_thigh_joint": 30,
    "RR_calf_joint": 31,
}

# Front thigh joints have a range of -1.5708 to 3.4907
# Rear thigh joints have a range of -0.5236 to 4.5379
# Calf joints have a range of -2.7227 to 0.3.

# joint_configs = {
#     "FR_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": math.pi/2.0, "wave": "cos"},
#     "FL_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
#     "RR_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
#     "RL_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": -math.pi/2.0, "wave": "cos"},
#     "FR_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": -math.pi/4.0, "wave": "cos"},
#     "FL_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": 3 * math.pi/4.0, "wave": "cos"},
#     "RR_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": 7 * math.pi/4.0, "wave": "cos"},
#     "RL_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": -3 * math.pi/4.0, "wave": "cos"},
#     "FL_hip_joint": {"angle": 0.0, "wave": "constant"},
#     "FR_hip_joint": {"angle": 0.0, "wave": "constant"},
#     "RL_hip_joint": {"angle": 0.0, "wave": "constant"},
#     "RR_hip_joint": {"angle": 0.0, "wave": "constant"},
# }

joint_configs = {
    "FR_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
    "FL_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
    "RR_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
    "RL_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
    "FR_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
    "FL_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
    "RR_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
    "RL_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
    "FL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "FR_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RR_hip_joint": {"angle": 0.0, "wave": "constant"},
}

for joint_name, config in joint_configs.items():
    if config["wave"] != "constant":
        config["amplitude"] = (config["max_angle"] - config["min_angle"]) / 2
        config["offset"] = (config["max_angle"] + config["min_angle"]) / 2

joint_indices = {name: leg_joints[name] for name in joint_configs}

start_time = time()
try:
    for _ in range(int(0.5*240)):
        p.stepSimulation()
        sleep(1 / 240)
    while True:
        # Calculate elapsed time
        elapsed_time = time() - start_time
        
        target_positions = []
        for joint_name, config in joint_configs.items():
            if config["wave"] == "sin":
                angle = config["offset"] + config["amplitude"] * math.sin(2 * math.pi * config["frequency"] * elapsed_time + config["angle_offset"])
            elif config["wave"] == "cos":
                angle = config["offset"] + config["amplitude"] * math.cos(2 * math.pi * config["frequency"] * elapsed_time + config["angle_offset"])
            elif config["wave"] == "constant":
                angle = config["angle"]
            else:
                raise ValueError(f"Unsupported wave type: {config['wave']}")
            target_positions.append(angle)
        
        # Set the joint position using motor control array
        p.setJointMotorControlArray(
            bodyUniqueId=go2_id,
            jointIndices=list(joint_indices.values()),
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_positions
        )
        
        # Step simulation
        p.stepSimulation()
        sleep(1 / 240)
except KeyboardInterrupt:
    print("Closing the simulation")

p.disconnect()