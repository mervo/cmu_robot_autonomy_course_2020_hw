import numpy as np
import math

import matplotlib.pyplot as plt


class DMP_trajectory_generator:
    def __init__(self, num_basis):
        '''
        DMP trajectory generator class
        inputs:
        num_basis = number of Gaussian basis functions to use
        '''
        self.T = 3  # value in seconds (time length of original trajectory)
        self.num_basis = num_basis
        self.K = 25 * 25 / 4
        self.B = 25
        self.dt = 0.003  # time step of trajectory
        self.C = np.linspace(0, 1, self.num_basis)  # Basis function centers
        self.H = (0.65 * (1. / (self.num_basis - 1.)) ** 2)  # Basis function widths

    def learn_weights_from_raw_trajectory(self, q, dq, ddq):
        '''
        This function learns DMP weights (i.e. shape parameters) for the trajectory.

        inputs: q (raw joint trajectory), dq (joint velocities), ddq (joint accelerations) are from the raw trajectory provided to you.
        output: learned weights
        '''
        Time = np.linspace(0, self.T, 1000)
        T = Time[-1]
        PHI = []
        F = []
        for k in range(len(Time)):
            Phi = [math.exp(-0.5 * ((Time[k] / Time[-1]) - c) ** 2 / self.H) for c in self.C]
            Phi = Phi / np.sum(Phi)
            PHI.append(Phi)

            f = ((ddq[k] * T ** 2) - self.K * (q[-1] - q[k]) + self.B * (dq[k] * T)) / (q[-1] - q[0])
            F.append(f)

        # calculate weights via linear regression
        w = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(PHI), PHI)), np.transpose(PHI)), F)

        self.weights = w

        return self.weights

    def generate_dmp_trajectory(self):
        '''
        TODO: implement this method to use your learned weights to generate a smooth
        DMP trajectory.       

        output: dmp trajectory based on starting position and learned weights
        '''
        q0 = np.zeros(5)  # starting joint positions
        q = q0
        q_goal = np.array([0, 0, 0.7, 3, 1])  # goal joint positions - feel free to play around with this!
        qd = np.zeros(5)
        qdd = np.zeros(5)

        q_traj = [q]
        qd_traj = [qd]
        qdd_traj = [qdd]

        t = 0
        for i in range(1000):
            t = t + self.dt
            # TODO: Write your code here
            if t <= self.T:
                phi = [math.exp(-0.5 * ((t / self.T) - c) ** 2 / self.H) for c in self.C]  # compute force term
                phi = phi / np.sum(phi)
                f = np.dot(phi, self.weights)
            else:
                f = 0  # Set force to 0 after t reaches T to ensure system goes to goal

            # Simulate spring and damper to get trajectory
            qdd = self.K * (q_goal - q) / self.T ** 2 - self.B * qd / self.T + (q_goal - q0) * f / self.T ** 2
            qdd_traj.append(qdd)
            qd = qd + qdd * self.dt
            qd_traj.append(qd)
            q = q + qd * self.dt
            q_traj.append(q)

        dmp_trajectory = q_traj, qd_traj, qdd_traj

        return dmp_trajectory


if __name__ == "__main__":
    q = np.load('q.npy')  # raw joint trajectory
    dq = np.load('dq.npy')
    ddq = np.load('ddq.npy')

    print(f'q: {q}\n'
          f'dq: {dq}\n'
          f'ddq: {ddq}\n')

    # TODO: Fit weights to the provided trajectory and use the learned weights to generate a DMP trajectory

    dmp_trajectory_generator = DMP_trajectory_generator(5)
    dmp_trajectory_generator.learn_weights_from_raw_trajectory(q, dq, ddq)

    q_traj, dq_traj, ddq_traj = dmp_trajectory_generator.generate_dmp_trajectory()

    # print(f'q: {q_traj}\n'
    #       f'dq: {dq_traj}\n'
    #       f'ddq: {ddq_traj}\n')

    # TODO: Plot your DMP trajectory
    import seaborn as sns

    joints = list(zip(*q_traj))
    x = np.linspace(1, len(joints[0]), len(joints[0]))
    sns.set()
    plt.xlabel("Time Step")
    plt.ylabel("Joint Configuration (rad)")
    plt.title("Time History Plot of Joints")
    joint = 0
    for y in joints:
        plt.plot(x, y, label=f'Joint {joint}')
        joint += 1
    plt.legend()
    plt.show()
