import matplotlib.pyplot as plt
import numpy as np
import os


class State:
    def __init__(self, timestamp: float, pos: np.ndarray, vel: np.ndarray, quat: np.ndarray,
                 acc_bias: np.ndarray, gyro_bias: np.ndarray):
        self._timestamp = timestamp
        self._pos = pos
        self._vel = vel
        self._quat = quat
        self._acc_bias = acc_bias
        self._gyro_bias = gyro_bias

    @property
    def time(self):
        return self._timestamp

    @property
    def pos(self):
        return self._pos

    @property
    def vel(self):
        return self._vel

    @property
    def quat(self):
        return self._quat

    @property
    def acc_bias(self):
        return self._acc_bias

    @property
    def gyro_bias(self):
        return self._gyro_bias


if __name__ == "__main__":
    gps_path = os.path.join(os.path.dirname(__file__), "data/gps_pos1.bag")
    x_data = []
    y_data = []
    with open(gps_path, "r") as f:
        for data in f.readlines():
            data = data.split()
            x_data.append(float(data[1]))
            y_data.append(float(data[2]))
    state_path = os.path.join(os.path.dirname(__file__), "data/state1.bag")
    with open(state_path, "r") as f:
        lines = f.readlines()
        size = len(lines)
        state_matrix = np.zeros(dtype=np.float32, shape=(size, 17))
        for i, line in enumerate(lines):
            value = line.split()
            state = State(float(value[0]),
                          np.array([float(value[1]), float(value[2]), float(value[3])]),
                          np.array([float(value[4]), float(value[5]), float(value[6])]),
                          np.array([float(value[7]), float(value[8]), float(value[9]), float(value[10])]),
                          np.array([float(value[11]), float(value[12]), float(value[13])]),
                          np.array([float(value[14]), float(value[15]), float(value[16])]))

            state_matrix[i, :] = np.hstack(
                (state.time, state.pos, state.vel, state.quat, state.acc_bias, state.gyro_bias))

        plt.figure()
        # 绘制位置轨迹
        plt.plot(state_matrix[:, 1], state_matrix[:, 2], color="blue", label="Position Trajectory")

        plt.scatter(x_data, y_data, color="red", label="GPS Trajectory")
        plt.scatter(state_matrix[0, 1], state_matrix[0, 2], color="black", label="Start Point")
        plt.scatter(state_matrix[-1, 1], state_matrix[-1, 2], color="green", label="End Point")
        plt.xlabel("x:m")
        plt.ylabel("y:m")
        plt.title("Position Trajectory")
        plt.legend()
        plt.savefig("Position Trajectory.png")
        # 绘制速度随时间变化
        plt.figure()
        plt.plot(state_matrix[:, 0], state_matrix[:, 4], color="blue", label="Velocity X")
        plt.plot(state_matrix[:, 0], state_matrix[:, 5], color="red", label="Velocity Y")
        plt.plot(state_matrix[:, 0], state_matrix[:, 6], color="green", label="Velocity Z")
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.title("Velocity Trajectory")
        plt.legend()
        plt.savefig("Velocity Trajectory.png")
        # 绘制bias随时间变化
        plt.figure()
        plt.plot(state_matrix[:, 0], state_matrix[:, 11], color="blue", label="Acc Bias X")
        plt.plot(state_matrix[:, 0], state_matrix[:, 12], color="red", label="Acc Bias Y")
        plt.plot(state_matrix[:, 0], state_matrix[:, 13], color="green", label="Acc Bias Z")
        plt.plot(state_matrix[:, 0], state_matrix[:, 14], color="yellow", label="Gyro Bias X")
        plt.plot(state_matrix[:, 0], state_matrix[:, 15], color="black", label="Gyro Bias Y")
        plt.plot(state_matrix[:, 0], state_matrix[:, 16], color="purple", label="Gyro Bias Z")

        plt.xlabel("Time")
        plt.ylabel("Bias")
        plt.title("Bias Trajectory")
        plt.legend()
        plt.savefig("Bias Trajectory.png")

        plt.show()
