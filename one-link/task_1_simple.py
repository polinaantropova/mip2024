import pybullet as p
import numpy as np
from control.matlab import place, lqr 
import matplotlib.pyplot as plt

def output (logTime, logPos, logVel, logCtrl):
    plt.subplot(3,1,1)
    plt.grid(True)
    plt.plot(logTime, logPos, label = "simPos")
    plt.plot([logTime[0],logTime[-1]],[thd,thd],'r', label='refPos')
    plt.legend()

    plt.subplot(3,1,2)
    plt.grid(True)
    plt.plot(logTime, logVel, label = "simVel")
    plt.legend()

    plt.subplot(3,1,3)
    plt.grid(True)
    plt.plot(logTime, logCtrl, label = "simCtrl")
    plt.legend()

    plt.show()

# задаём параметры симуляции
f = 0.1
dt = 1/240

# задаём положение маятника
th0 = np.deg2rad(20) # стартовое
thd = np.deg2rad(180) # желаемое

jIdx = 1
e_int = 0
e_prev = 0

# задаём массивы для сбора показателей
maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = th0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)

idx = 0
u = 0

# описываем маятник
g = 10
L = 0.5
m = 1
kf = 0.5

# строим вспомогательные коэффициенты
a = g/L
b = -1/(m*L*L)
c = kf*b

# строим матрицы A, B и K
A = np.array([[0, 1],
            [a, c]])
B = np.array(([0], [b]))
poles = np.array([-20,-40])
K = -place(A,B,poles)
print(f"poles theor: {poles}")
poles_fact = np.linalg.eig(A+B@K)
print(f"poles fact: {poles_fact}")

# запускаем симуляцию
physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-10)
boxId = p.loadURDF("./pendulum.urdf", useFixedBase=True)
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

# отправляемся в начальное положение
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# отключаем мотор
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for t in logTime[1:]:
    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    e = th1-thd
    ed = (e-e_prev)/dt
    e_int += e*dt
    e_prev = e

    # рассчитываем управление
    u = -K[0,0]*e - K[0,1]*dth1
    logCtrl[idx] = u
    
    # применяем управление
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx, 
        controlMode=p.TORQUE_CONTROL, 
        force=u
    )
    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = dth1

    idx += 1
    logPos[idx] = th1

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logCtrl[idx] = u

output (logTime, logPos, logVel, logCtrl)

p.disconnect()