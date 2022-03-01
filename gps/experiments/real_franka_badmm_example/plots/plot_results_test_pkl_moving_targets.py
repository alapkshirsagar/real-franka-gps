import matplotlib.pyplot as plt
import csv
import numpy as np
import pandas as pd
import sys
sys.path.append('/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/python')
import gps
from mpl_toolkits.mplot3d import Axes3D

timesteps = 100
nConditions = 90
iteration = 10
nTrainConditions = 12

## Get training locations
# filepath = "/media/franka2/DATA/Kuka_Franka_Experiment_Data/real_franka_badmm_example_8_MovingTrainer_MovingTester_90con_100hz/data_files/"
filepath = "/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/experiments/real_franka_badmm_example/data_files/pkl_files/12movingmoving/test90con/"

# human_data_file = open('/home/franka2/Kuka_Franka_GPS/src/real-franka-gps/gps/src/gps_agent_pkg/scripts/hand_data_train.csv')
human_data_file = open('/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/experiments/real_franka_badmm_example/plots/hand_data_train.csv')
csvreader = csv.reader(human_data_file)    
nTrajectoriesRecorded = 4 
nTrajectoriesDesired = nTrainConditions #Number of trajectories desired 8/12 

trajectory_human = np.zeros((nTrajectoriesRecorded*100,3))
trajectory_rotated = np.zeros((nTrajectoriesDesired*100,3)) #this includes all the trajectories of 100 timesteps each
theta = np.linspace(0,np.pi/4,num = nTrajectoriesDesired/nTrajectoriesRecorded, endpoint = True, dtype = float)
print(range(len(theta)))
row_number = 0
for row in csvreader:
    # for column_number in range(3):
    trajectory_human[row_number, 0] = float(row[2])
    trajectory_human[row_number, 1] = float(row[0])
    trajectory_human[row_number, 2] = float(row[1])

    radius = (trajectory_human[row_number, 0]**2+trajectory_human[row_number, 1]**2)**0.5
    for angle in range(len(theta)):
        trajectory_rotated[angle*nTrajectoriesRecorded*100+row_number, :] = [radius*np.cos(theta[angle]), radius*np.sin(theta[angle]), float(row[1])]
    row_number+=1
trans_human_training = []

for i in range(nTrajectoriesDesired):
    trans_human_training.append(trajectory_rotated[i*100:(i+1)*100, :])
# print(trans_human_training)


## Get testing locations
# human_data_file = open('/home/franka2/Kuka_Franka_GPS/src/real-franka-gps/gps/src/gps_agent_pkg/scripts/hand_data_test.csv')
human_data_file = open('/media/alap/OS/Users/alap_/CatkinWorkspaces/Kuka_Franka_GPS/src/gps/experiments/real_franka_badmm_example/plots/hand_data_test.csv')
csvreader = csv.reader(human_data_file)   
nTrajectoriesRecorded = 18 #Number of trajectories recorded, 4 or 18 depending on train or test
nTrajectoriesDesired = 90 #Number of trajectories desired, 8/12 or 90 depending on train with 8/12 or test

trajectory_human = np.zeros((nTrajectoriesRecorded*100,3))
trajectory_rotated = np.zeros((nTrajectoriesDesired*100,3)) #this includes all the trajectories of 100 timesteps each
theta = np.linspace(0,np.pi/4,num = nTrajectoriesDesired/nTrajectoriesRecorded, endpoint = True, dtype = float)
print(range(len(theta)))
row_number = 0
for row in csvreader:
    # for column_number in range(3):
    trajectory_human[row_number, 0] = float(row[2])
    trajectory_human[row_number, 1] = float(row[0])
    trajectory_human[row_number, 2] = float(row[1])

    radius = (trajectory_human[row_number, 0]**2+trajectory_human[row_number, 1]**2)**0.5
    for angle in range(len(theta)):
        trajectory_rotated[angle*nTrajectoriesRecorded*100+row_number, :] = [radius*np.cos(theta[angle]), radius*np.sin(theta[angle]), float(row[1])]
    row_number+=1
trans_human_testing = []

for i in range(nTrajectoriesDesired):
    trans_human_testing.append(trajectory_rotated[i*100:(i+1)*100, :])
average_testing_error = np.zeros([nConditions])
px_train = np.zeros([nTrainConditions])
py_train = np.zeros([nTrainConditions])
pz_train = np.zeros([nTrainConditions])

px_test = np.zeros([nConditions])
py_test = np.zeros([nConditions])
pz_test = np.zeros([nConditions])

pickle_object = pd.read_pickle(filepath+"pol_sample_itr_"+str(iteration)+".pkl")

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
    px_test[condition] = trans_human_testing[condition][-1,0]
    py_test[condition] = trans_human_testing[condition][-1,1]
    pz_test[condition] = trans_human_testing[condition][-1,2]

for condition in range(nTrainConditions):
    px_train[condition] = trans_human_training[condition][-1,0]
    py_train[condition] = trans_human_training[condition][-1,1]
    pz_train[condition] = trans_human_training[condition][-1,2]

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

plt.savefig("12_MovingTrainer_MovingTester_90con_100hz.png")
plt.show()
# # import pdb
# # pdb.set_trace()



