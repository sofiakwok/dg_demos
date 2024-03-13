from matplotlib import pyplot as plt
import numpy as np
import pinocchio

x = np.arange(-1.0, 1.0, 0.0001)
yaw = -2 * np.pi * x
roll = pitch = np.zeros(yaw.shape)

rpy = np.vstack([roll, pitch, yaw])
print(rpy)

quaternion = np.zeros(4)

for i, _ in enumerate(x):
    mat = pinocchio.utils.rpyToMatrix(rpy[:, i])
    quat = pinocchio.Quaternion(mat)
    # print(quat)
    quaternion = np.vstack([quaternion, quat.coeffs()])

# Plots.
plt.figure("quat yaw")
plt.plot(x, quaternion[1:, 0], label="quat_x")
plt.plot(x, quaternion[1:, 1], label="quat_y")
plt.plot(x, quaternion[1:, 2], label="quat_z")
plt.plot(x, quaternion[1:, 3], label="quat_w")
plt.plot(x, yaw, label="yaw")
plt.legend()
plt.show()
