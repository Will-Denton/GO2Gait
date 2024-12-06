import pybullet as p
from time import sleep, time
import pybullet_data
import math
import matplotlib.pyplot as plt

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

# Joint configurations for the shuffle gait
shuffle_joint_configs = {
    "FR_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": math.pi/2.0, "wave": "cos"},
    "FL_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": 0.0, "wave": "cos"},
    "RR_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": math.pi, "wave": "cos"},
    "RL_thigh_joint": {"min_angle": -0.3, "max_angle": 0.2, "frequency": 0.8, "angle_offset": -math.pi/2.0, "wave": "cos"},
    "FR_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": -math.pi/4.0, "wave": "cos"},
    "FL_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": 3 * math.pi/4.0, "wave": "cos"},
    "RR_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": 7 * math.pi/4.0, "wave": "cos"},
    "RL_calf_joint": {"min_angle": 0.0, "max_angle": -0.3, "frequency": 0.8, "angle_offset": -3 * math.pi/4.0, "wave": "cos"},
    "FL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "FR_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RR_hip_joint": {"angle": 0.0, "wave": "constant"},
}

# Joint configurations for the run gait
run_joint_configs = {
    "FR_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 2.7, "angle_offset": math.pi, "wave": "cos"},
    "FL_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 2.7, "angle_offset": 0.0, "wave": "cos"},
    "RR_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 2.7, "angle_offset": 0.0, "wave": "cos"},
    "RL_thigh_joint": {"min_angle": 0.75, "max_angle": 1.4, "frequency": 2.7, "angle_offset": math.pi, "wave": "cos"},
    "FR_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 2.7, "angle_offset": math.pi / 3.0, "wave": "cos"},
    "FL_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 2.7, "angle_offset": 4 * math.pi / 3.0, "wave": "cos"},
    "RR_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 2.7, "angle_offset": 4 * math.pi / 3.0, "wave": "cos"},
    "RL_calf_joint": {"min_angle": -0.8, "max_angle": -2.0, "frequency": 2.7, "angle_offset": math.pi / 3.0, "wave": "cos"},
    "FL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "FR_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RL_hip_joint": {"angle": 0.0, "wave": "constant"},
    "RR_hip_joint": {"angle": 0.0, "wave": "constant"},
}

def connect_pybullet():
    # Connect to the PyBullet physics server
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)
    return physicsClient

def add_plane():
    # Load the plane
    planeId = p.loadURDF("plane.urdf")
    p.changeDynamics(planeId, -1, lateralFriction=1.0)
    return planeId

def add_robot():
    # load the GO2 robot
    startPos = [0, 0, 0.8]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    go2_id = p.loadURDF("/go2_description/urdf/go2_description.urdf", startPos, startOrientation, useFixedBase=False)
    go2Pos, go2Orn = p.getBasePositionAndOrientation(go2_id)

    # Set the initial joint positions
    for _, config in run_joint_configs.items():
        if config["wave"] != "constant":
            config["amplitude"] = (config["max_angle"] - config["min_angle"]) / 2
            config["offset"] = (config["max_angle"] + config["min_angle"]) / 2

    return go2_id

def run_sim():
    # Connect to the PyBullet physics server
    physicsClient = connect_pybullet()

    # Add the plane
    planeId = add_plane()

    # Add the robot
    go2_id = add_robot()

    # Setup the joint position arrays
    joint_indices = {name: leg_joints[name] for name in run_joint_configs}
    joint_angle_logs = {joint_name: [] for joint_name in run_joint_configs.keys()}
    time_log = []
    
    # Run the simulation
    start_time = time()
    try:
        for _ in range(int(0.5*240)):
            p.stepSimulation()
            sleep(1 / 240)
        while True:
            # Calculate elapsed time
            elapsed_time = time() - start_time
            target_positions = []
            for joint_name, config in run_joint_configs.items():
                if config["wave"] == "sin":
                    angle = config["offset"] + config["amplitude"] * math.sin(2 * math.pi * config["frequency"] * elapsed_time + config["angle_offset"])
                elif config["wave"] == "cos":
                    angle = config["offset"] + config["amplitude"] * math.cos(2 * math.pi * config["frequency"] * elapsed_time + config["angle_offset"])
                elif config["wave"] == "constant":
                    angle = config["angle"]
                else:
                    raise ValueError(f"Unsupported wave type: {config['wave']}")
                
                # Append the target position to the list
                target_positions.append(angle)
                joint_angle_logs[joint_name].append(angle)

            # Append the elapsed time to the time log
            time_log.append(elapsed_time)
                
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

    # Disconnect from the PyBullet physics server
    p.disconnect()

    # Plot the joint angles over time
    plt.figure(figsize=(10, 8))
    for joint_name, angles in joint_angle_logs.items():
        plt.plot(time_log[:len(angles)], angles, label=joint_name)

    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (rad)")
    plt.title("Joint Angles Over Time")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    run_sim()