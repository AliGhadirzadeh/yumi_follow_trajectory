#!/usr/bin/env python
import numpy as np
from time import sleep
from scipy import interpolate
import argparse
import os
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--file-name", type=str, default=None, help="Filename of the waypoint file with .npy extention")
parser.add_argument("--time-step", type=float, default=0.5, help="The timestep between the waypoints")
parser.add_argument("--sample-rate", type=float, default=20, help="The control sample rate (Hz)")

args = parser.parse_args()

assert(os.path.isfile(args.file_name))

waypoints = np.load(args.file_name)

npoints = waypoints.shape[0]

time_waypoints = np.arange(npoints) * args.time_step
sampling_time = np.arange(time_waypoints[0], time_waypoints[npoints-1], 1.0/args.sample_rate)
nsamples = sampling_time.shape[0]
traj_jpos = np.zeros((nsamples,7))
traj_jvel = np.zeros((nsamples,7))
traj_gripper = np.zeros((nsamples,1))

for j in range(7):
    tck = interpolate.CubicSpline(time_waypoints, waypoints[:,j])
    traj_jpos[:, j] = tck(sampling_time)
    traj_jvel[:, j] = tck(sampling_time,1)

    plt.subplot(311)
    plt.plot(sampling_time, traj_jpos[:,j])
    plt.plot(time_waypoints, waypoints[:,j], '*')
    plt.subplot(312)
    plt.plot(sampling_time, traj_jvel[:,j])

tck = interpolate.CubicSpline(time_waypoints, (waypoints[:,7]-0.5)*10.0, bc_type='natural')
traj_gripper = tck(sampling_time)
plt.subplot(313)
plt.plot(sampling_time, traj_gripper)
plt.plot(time_waypoints, (waypoints[:,7]-0.5)*10.0, '*')
plt.show()

traj = np.append(sampling_time.reshape(nsamples,1), traj_jpos, axis = 1)
traj = np.append(traj, traj_jvel, axis = 1)
traj = np.append(traj, traj_gripper.reshape(nsamples,1), axis = 1)

np.save('traj.npy', traj)
np.savetxt('traj.txt', traj)
