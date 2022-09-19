# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.13.8
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# %% [markdown]
# # Rosbag Sensor Data Inspection
#
# Analysis of sensor data in a single rosbag

# %%
import numpy as np
import rosbag

# %matplotlib inline
import matplotlib.pyplot as plt

figsize = (15,5)

# %%
# bag_path = "/data/datasets/uHumans2/office/uHumans2_office_s1_00h.bag"
# bag_path = "/data/datasets/uHumans2/office/uHumans2_office_s1_06h.bag"
# bag_path = "/data/datasets/misc/uHumans2_office_test.bag"
# bag_path = "/data/datasets/misc/uHumans2_office_sequential.bag"
# bag_path = "/data/datasets/misc/uHumans2_office_sequential2.bag"
# bag_path = "/data/datasets/misc/uHumans2_office_sequential3.bag"
# bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_sequential.bag"
# bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_sequential9.bag"
# bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_parallelshort2.bag"
bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_sequential_1loop.bag"
# bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_sequential_1loop_fast_9.bag"
# bag_path = "/data/datasets/Ford/tesse_ford/pill/windridge_pill_test_sequential_1loop_9.1x.bag"

imu_topic = "/tesse/imu/clean/imu"
image_topics = [
    "/tesse/left_cam/mono/image_raw",
    "/tesse/right_cam/mono/image_raw",
]

# %%
bag = rosbag.Bag(bag_path)

# %% [markdown]
# ## IMU Time Deltas
#
# Plot the time-delta between consecutive IMU messages

# %%
ts = []
ts_msg = []
for topic, msg, t in bag.read_messages(topics=[imu_topic]):
    ts.append(t.to_sec())
    ts_msg.append(msg.header.stamp.to_sec())
dts = [ts[i] - ts[i-1] for i in range(1, len(ts))]
dts_msg = [ts_msg[i] - ts_msg[i-1] for i in range(1, len(ts_msg))]

assert (np.all(np.array(dts) - np.array(dts_msg)) == 0)

# %%
print("Max Time-Delta (seconds):     ", np.max(dts))
print("Min Time-Delta (seconds):     ", np.min(dts))
print("Standard Deviation (seconds): ", np.std(dts))

plt.figure(figsize=figsize)
plt.plot(dts, label="IMU Time Deltas")
plt.xlabel("Message Count")
plt.ylabel("Time Delta (s)")
plt.title("IMU Timestamp Deltas")
plt.show()

# %% [markdown]
# ## IMU Raw Data
#
# Plot IMU acceleration and gyroscope data over the entire bag

# %%
accel = []
gyro = []
ts = []
for topic, msg, t in bag.read_messages(topics=[imu_topic]):
    ts.append(t.to_sec())
    accel.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
accel = np.array(accel)
gyro = np.array(gyro)

# %%
print("Max Acceleration Norm:     ", np.max(np.abs(np.linalg.norm(accel, axis=1))))
print("Min Acceleration Norm:     ", np.min(np.abs(np.linalg.norm(accel, axis=1))))

plt.figure(figsize=figsize)
plt.plot(ts, accel[:,0], label="x")
plt.plot(ts, accel[:,1], label="y")
plt.plot(ts, accel[:,2], label="z")
plt.title("IMU Acceleration Data")
plt.xlabel("TimeStamp (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.ylim(-1,1)
plt.legend()
plt.show()

# %%
print("Max Angular Velocity Norm:     ", np.max(np.abs(np.linalg.norm(gyro, axis=1))))
print("Min Angular Velocity Norm:     ", np.min(np.abs(np.linalg.norm(gyro, axis=1))))

plt.figure(figsize=figsize)
plt.plot(ts, gyro[:,0], label="x")
plt.plot(ts, gyro[:,1], label="y")
plt.plot(ts, gyro[:,2], label="z")
plt.title("IMU Angular Velocity (Gyro) Data")
plt.xlabel("TimeStamp (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend()
plt.show()

# %% [markdown]
# ## Image Time Deltas

# %%
ts = {topic : [] for topic in image_topics}
ts_msg = {topic : [] for topic in image_topics}

for topic, msg, t in bag.read_messages(topics=image_topics):
    ts[topic].append(t.to_sec())
    ts_msg[topic].append(msg.header.stamp.to_sec())

dts = {
    topic: [ts[topic][i] - ts[topic][i-1] for i in range(1, len(ts[topic]))] for topic in image_topics
}
dts_msg = {
    topic: [ts_msg[topic][i] - ts_msg[topic][i-1] for i in range(1, len(ts_msg[topic]))] for topic in image_topics
}

for topic in image_topics:
    assert (np.all(np.array(dts[topic]) - np.array(dts_msg[topic])) == 0)

# %%
for topic in image_topics:
    print("Max Time-Delta (seconds) for topic:     ", topic, ": ", np.max(dts[topic]))
    print("Min Time-Delta (seconds) for topic:     ", topic, ": ", np.min(dts[topic]))
    print("Avg Time-Delta (seconds) for topic:     ", topic, ": ", np.mean(dts[topic]))
    print("Standard Deviation (seconds) for topic: ", topic, ": ", np.std(dts[topic]), "\n")

plt.figure(figsize=figsize)
for topic in image_topics:
    plt.plot(dts[topic], label=topic)
plt.xlabel("Message Count")
plt.ylabel("Time Delta (s)")
plt.title("Image Timestamp Deltas")
plt.legend()
plt.show()

# %% [markdown]
# # Rosbag Comparison
#
# Compare two rosbags together. Intended mainly for use with simulator datasets (i.e. uHumans2) to ensure that timing and IMU data are identical for different runs on the same trajectory.

# %%
# bag_paths = [
# #     "/data/datasets/uHumans2/office/uHumans2_office_s1_00h.bag",
# #     "/data/datasets/uHumans2/office/uHumans2_office_s1_06h.bag",
# #     "/data/datasets/misc/uHumans2_office_sequential.bag",
# #     "/data/datasets/misc/uHumans2_office_sequential3.bag",
# #     "/data/datasets/misc/uHumans2_office_sequential4.bag",
# #     "/data/datasets/misc/uHumans2_office_sequential2.bag",
# #     "/data/datasets/misc/test_old3.bag",
# #     "/data/datasets/misc/uHumans2_office_test.bag",
# #     "/data/datasets/misc/uHumans2_office_sequentialreal_test1.bag",
    
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential_1loop_5.1.8.bag",
    
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_parallel_1loop_fast.bag",
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential_1loop_fast_5.1.8.bag",
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential_1loop_fast_5.1.8-2.bag",
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential_1loop_fast_5.1.8-3.bag",
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential_1loop_fast_9.bag",
    
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_sequential9.bag",
# #     "/data/datasets/Ford/tesse_ford/pill/windridge_pill_short_parallel.bag",
    
#     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_parallel.bag",
# #     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_sequential_5_1_8.bag",
#     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_sequential_5_1.bag",
# #     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_sequential_5_3.bag",
# #     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_sequential_5_5.bag",
# #     "/data/datasets/Ford/tesse_ford/windridge_car_2loop_sequential_20fps.bag",
# ]

# assert len(bag_paths) == 2

# imu_topic = "/tesse/imu/clean/imu"
# image_topics = [
#     "/tesse/left_cam/mono/image_raw",
#     "/tesse/right_cam/mono/image_raw",
# ]

# %%
bag_paths = [
    "/data/datasets/Ford/LCTest/bags/undist_ford_Rec20210709134157_split_000.bag",
    "/data/datasets/Ford/LCTest/bags/undist_nomultfactor_Rec20210709134157_split_000.bag",
]

assert len(bag_paths) == 2

imu_topic = "/RT3000/imu"
image_topics = [
    "/RightCameraUndistorted/image_raw",
]

# %%
bag1 = rosbag.Bag(bag_paths[0])
bag2 = rosbag.Bag(bag_paths[1])

# %% [markdown]
# ## IMU Timing Comparision
#
# Compare raw timestamp difference between two rosbags.
#
# It's unlikely that the dt's will be zero, but a flat line indicates constant IMU rate relative to the other bag. Oscillations indicate the rate is changing over time.

# %%
ts1 = []
ts2 = []
for topic, msg, t in bag1.read_messages(topics=[imu_topic]):
    ts1.append(t.to_sec())
for topic, msg, t in bag2.read_messages(topics=[imu_topic]):
    ts2.append(t.to_sec())

if (len(ts1) < len(ts2)):
    dts = [ts1[i] - ts2[i] for i in range(len(ts1))]
else:
    dts = [ts1[i] - ts2[i] for i in range(len(ts2))]

# %%
print("Max Time-Delta (seconds):     ", np.max(dts))
print("Min Time-Delta (seconds):     ", np.min(dts))
print("Standard Deviation (seconds): ", np.std(dts))

plt.figure(figsize=figsize)
plt.plot(dts, label="IMU Time Deltas")
plt.xlabel("Message Count")
plt.ylabel("Time Delta (s)")
plt.title("IMU Message Timestamp Difference Between Two Rosbags")
plt.show()

# %% [markdown]
# ## IMU Raw Data Comparison
#
# Plot the acceleration and angular velocity data of two bags directly against each other

# %%
accel1 = []
gyro1 = []
ts1 = []
for topic, msg, t in bag1.read_messages(topics=[imu_topic]):
    ts1.append(t.to_sec())
    accel1.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    gyro1.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
accel1 = np.array(accel1)
gyro1 = np.array(gyro1)

accel2 = []
gyro2 = []
ts2 = []
for topic, msg, t in bag2.read_messages(topics=[imu_topic]):
    ts2.append(t.to_sec())
    accel2.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    gyro2.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
accel2 = np.array(accel2)
gyro2 = np.array(gyro2)

ts1 = np.array(ts1) - ts1[0]
ts2 = np.array(ts2) - ts2[0]

# print("Max accel diff: ", np.max(accel2 - accel1))
# print("Max gyro diff: ", np.max(gyro2 - gyro1))

# %%
print("Max Acceleration Norm (bag 1):     ", np.max(np.abs(np.linalg.norm(accel1, axis=1))))
print("Min Acceleration Norm (bag 1):     ", np.min(np.abs(np.linalg.norm(accel1, axis=1))))

print("Max Acceleration Norm (bag 2):     ", np.max(np.abs(np.linalg.norm(accel2, axis=1))))
print("Min Acceleration Norm (bag 2):     ", np.min(np.abs(np.linalg.norm(accel2, axis=1))))

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize, sharey=False)


ax1.plot(ts1, accel1[:,0], label="bag 1")
ax1.plot(ts2, accel2[:,0], label="bag 2")
ax1.set_xlabel("TimeStamp (s)")
ax1.set_ylabel("x-Acceleration (m/s^2)")
ax1.legend()
ax1.set_ylim(-15,15)

ax2.plot(ts1, accel1[:,1], label="bag 1")
ax2.plot(ts2, accel2[:,1], label="bag 2")
ax2.set_xlabel("TimeStamp (s)")
ax2.set_ylabel("y-Acceleration (m/s^2)")
ax2.legend()
ax2.set_ylim(-15,15)

ax3.plot(ts1, accel1[:,2], label="bag 1")
ax3.plot(ts2, accel2[:,2], label="bag 2")
ax3.set_xlabel("TimeStamp (s)")
ax3.set_ylabel("z-Acceleration (m/s^2)")
ax3.legend()
ax3.set_ylim(7,12)

fig.suptitle("IMU Acceleration Data (Difference between Two Rosbags)")
plt.show()

# %%
print("Max Angular Velocity Norm (bag 1):     ", np.max(np.abs(np.linalg.norm(gyro1, axis=1))))
print("Min Angular Velocity Norm (bag 1):     ", np.min(np.abs(np.linalg.norm(gyro1, axis=1))))

print("Max Angular Velocity Norm (bag 2):     ", np.max(np.abs(np.linalg.norm(gyro2, axis=1))))
print("Min Angular Velocity Norm (bag 2):     ", np.min(np.abs(np.linalg.norm(gyro2, axis=1))))

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize, sharey=False)


ax1.plot(ts1, gyro1[:,0], label="bag 1")
ax1.plot(ts2, gyro2[:,0], label="bag 2")
ax1.set_xlabel("TimeStamp (s)")
ax1.set_ylabel("x-Angular Velocity (m/s^2)")
ax1.legend()
ax1.set_ylim(-1,1)

ax2.plot(ts1, gyro1[:,1], label="bag 1")
ax2.plot(ts2, gyro2[:,1], label="bag 2")
ax2.set_xlabel("TimeStamp (s)")
ax2.set_ylabel("y-Angular Velocity (m/s^2)")
ax2.legend()
ax2.set_ylim(-1,1)

ax3.plot(ts1, gyro1[:,2], label="bag 1")
ax3.plot(ts2, gyro2[:,2], label="bag 2")
ax3.set_xlabel("TimeStamp (s)")
ax3.set_ylabel("z-Angular Velocity (m/s^2)")
ax3.legend()
# ax3.set_ylim(9.7,9.9)

fig.suptitle("IMU Angular Velocity Data (Difference between Two Rosbags)")
plt.show()

# %% [markdown]
# ## Image Timing Comparison
#
# As with IMU, it's unlikely that the dt's will be zero, but a flat line indicates constant image rate relative to the other bag. Oscillations indicate the rate is changing over time.

# %%
ts1 = {topic : [] for topic in image_topics}
ts2 = {topic : [] for topic in image_topics}

for topic, msg, t in bag1.read_messages(topics=image_topics):
    ts1[topic].append(t.to_sec())
for topic, msg, t in bag2.read_messages(topics=image_topics):
    ts2[topic].append(t.to_sec())
    
dts = {topic: [] for topic in image_topics}

for topic in image_topics:
    if len(ts1[topic]) < len(ts2[topic]):
        dts[topic] = [ts1[topic][i] - ts2[topic][i] for i in range(len(ts1[topic]))]
    else:
        dts[topic] = [ts1[topic][i] - ts2[topic][i] for i in range(len(ts2[topic]))]

# %%
plt.figure(figsize=figsize)
for topic in image_topics:
    plt.plot(np.array(ts1[topic]) - ts1[topic][0], label="bag1: "+topic)
    plt.plot(np.array(ts2[topic]) - ts2[topic][0], label="bag2: "+topic)
plt.xlabel("Message Count")
plt.ylabel("Time (s)")
plt.title("Image Timestamps (Starting at Zero)")
plt.legend()
plt.show()

# %%
for topic in image_topics:
    print("Max Time-Delta (seconds) for topic:     ", topic, ": ", np.max(dts[topic]))
    print("Min Time-Delta (seconds) for topic:     ", topic, ": ", np.min(dts[topic]))
    print("Avg Time-Delta (seconds) for topic:     ", topic, ": ", np.mean(dts[topic]))
    print("Standard Deviation (seconds) for topic: ", topic, ": ", np.std(dts[topic]), "\n")

plt.figure(figsize=figsize)
for topic in image_topics:
    plt.plot(dts[topic], label=topic)
plt.xlabel("Message Count")
plt.ylabel("Time Delta (s)")
plt.title("Difference in Image Timestamps Between 2 Rosbags")
plt.legend()
plt.show()
