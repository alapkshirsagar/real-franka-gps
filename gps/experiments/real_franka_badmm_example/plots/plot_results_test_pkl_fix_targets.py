import matplotlib.pyplot as plt
import csv
import numpy as np
import pandas as pd
import sys
sys.path.append('/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/python')
import gps
from mpl_toolkits.mplot3d import Axes3D

# #8 conditions
timesteps = 100
nConditions = 90
iteration = 10
nTrainConditions = 12
if nTrainConditions == 8:
    num_theta = 2
elif nTrainConditions == 12:
    num_theta = 3

filepath = "/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/experiments/real_franka_badmm_example/data_files/pkl_files/12staticstatic/test90con/"
average_testing_error = np.zeros([nConditions])
px_train = np.zeros([nTrainConditions])
py_train = np.zeros([nTrainConditions])
pz_train = np.zeros([nTrainConditions])

px_test = np.zeros([nConditions])
py_test = np.zeros([nConditions])
pz_test = np.zeros([nConditions])

pickle_object = pd.read_pickle(filepath+"pol_sample_itr_"+str(iteration)+".pkl")
# import pdb; pdb.set_trace()
# print(object)
train_locations = [np.array([r*np.cos(theta*np.pi/180),r*np.sin(theta*np.pi/180),z]) for r in np.linspace(0.70, 0.80, num = 2)
        for theta in np.linspace(0, 45, num = num_theta)
            for z in np.linspace(0.2, 0.25, num = 2)]

test_locations = [np.array([r*np.cos(theta*np.pi/180),r*np.sin(theta*np.pi/180),z]) for r in np.linspace(0.70, 0.80, num = 3)
        for theta in np.linspace(0, 45, num = 10)
            for z in np.linspace(0.2, 0.25, num = 3)]

robot_starting_loction = [0.5771669156551361, -0.03971900790929794, 0.08279954075813295]

for condition in range(nConditions):
    distance = []
    # import pdb; pdb.set_trace()
    for i in [timesteps-1]:#range(0,timesteps):
        distance.append(np.linalg.norm(pickle_object[condition].get_samples()[0]._data[3][i,3:6]))
        # print(np.linalg.norm(object[condition].get_samples()[0].get_X(i))))
        # print(np.linalg.norm(pickle_object[condition].get_samples()[0]._data[3][i,3:6]))
    # print(condition)
    average_testing_error[condition] = np.mean(distance)
    px_test[condition] = test_locations[condition][0]
    py_test[condition] = test_locations[condition][1]
    pz_test[condition] = test_locations[condition][2]

for condition in range(nTrainConditions):
    px_train[condition] = train_locations[condition][0]
    py_train[condition] = train_locations[condition][1]
    pz_train[condition] = train_locations[condition][2]

print("Average testing error:", np.mean(average_testing_error))


# import pdb; pdb.set_trace()

fig = plt.figure()

ax = fig.add_subplot(111, projection="3d")
ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

ax.view_init(elev=45, azim=-135)

cm = plt.cm.get_cmap("YlOrRd")
sc = ax.scatter(px_test, py_test, pz_test, c=average_testing_error, cmap=cm, vmin = 0.0, vmax = 0.1)
ax.set_xlabel('x (m)', fontsize = 'large')
ax.set_ylabel('y (m)', fontsize = 'large')
ax.set_zlabel('z (m)', fontsize = 'large')
ax.set_xlim(0.25, 0.85)
ax.set_ylim(-0.05, 0.55)
ax.set_zlim(0.0, 0.6)
cbar = plt.colorbar(sc)
cbar.ax.tick_params(labelsize = 'large')
cbar.set_label("Error (m)", fontsize = 'large')
sc = ax.scatter(px_train, py_train, pz_train, marker='s', cmap=cm, s=50, facecolors="none", edgecolors="k", alpha = 0.8, linewidths=0)
sc = ax.scatter(robot_starting_loction[0], robot_starting_loction[1], robot_starting_loction[2], marker='o', cmap=cm, s=100, facecolors="none", edgecolors="k", alpha = 1.0, linewidths=0)

plt.savefig("12_StaticTrainer_StaticTester_90con.png")
plt.show()

