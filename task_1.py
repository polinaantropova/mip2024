import pybullet as p
import numpy as np
import time
from control.matlab import place


def define_timelines(_dt, _max_time):
    timeframe = np.arange(0.0, _max_time, _dt)

    return {
        "time": timeframe,
        "position": np.zeros(len(timeframe)),
        "velocity": np.zeros(len(timeframe)),
        "control": np.zeros(len(timeframe))
    }


def get_x(model):
    plummet_1_idx = 1
    plummet_2_idx = 3
    states = p.getJointStates(model, [plummet_1_idx, plummet_2_idx])

    return [states[0][0], states[0][1], states[1][0], states[1][1]]


def set_u(model, v1):
    plummet_1_idx = 1

    p.setJointMotorControl2(bodyIndex=model, jointIndex=plummet_1_idx,
                                controlMode=p.TORQUE_CONTROL, force=v1)

def calculate_u(X):
    m = 2  # масса каждого стержня
    l = 0.5  # длина каждого стержня
    g = 10  # ускорение свободного падения

    a = g / l / (1 - m)
    b = 1 / (1 - m)

    A = np.array([
        [0, 1, 0, 0],
        [-a, 0, m * a, 0],
        [0, 0, 0, 1],
        [a, 0, -a, 0]
    ])

    B = np.array([[0], [b], [0], [-b]])
    p = [-20, -18, -16, -14]
    K = place(A, B, p)

    print(f"poles theor: {p}")
    poles_fact = np.linalg.eig(A-B@K)
    print(f"poles fact: {poles_fact}")

    return K @ X

def plot(_output):
    import matplotlib.pyplot as plt
    plt.subplot(3, 1, 1)
    plt.grid(True)
    plt.plot(_output["time"], _output["position"], label="Position")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.grid(True)
    plt.plot(_output["time"], _output["velocity"], label="Velocity")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.grid(True)
    plt.plot(_output["time"], _output["control"], label="Control")
    plt.legend()

    plt.show()

dt = 1/240  # pybullet simulation step
maxTime = 5
output = define_timelines(dt, maxTime)

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0, 0, -10)
boxId = p.loadURDF("./two-link.urdf", useFixedBase=True)

# turn off internal damping
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=[1, 3], targetVelocities=[0, 0],
                            controlMode=p.VELOCITY_CONTROL, forces=[0,0])

# go to the starting position
p.stepSimulation()
idx = 1
for t in output["time"][1:]:
    X = get_x(boxId)
    print(X)
    u = calculate_u(X)

    set_u(boxId, u)
    output["position"][idx] = X[0] 
    output["velocity"][idx] = X[1]
    output["control"][idx] = u
    p.stepSimulation()
    time.sleep(dt)
    idx += 1

p.disconnect()

plot(output)