import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import re


# Load the log file
file_path = '/home/moro/test_log/sync/sync.txt'  # Update this with the path to your log file

# Initialize lists to store the parsed data
time_received = []
lidar_time = []
camera_time = []

# Regular expression to extract times
regex_pattern = r"\[INFO\] \[(\d+\.\d+)\] \[sync_node\]: Exact Sync - Lidar: \[Lidar count: \d+ Time: (\d+)\], Camera: \[Camera count: \d+ Time: (\d+)\]"

# Open the file and parse the data using regex
with open(file_path, 'r') as file:
    for line in file:
        match = re.search(regex_pattern, line)
        if match:
            time_received.append(float(match.group(1)))  # Time received
            lidar_time.append(int(match.group(2)))       # Lidar timestamp
            camera_time.append(int(match.group(3)))      # Camera timestamp
        else:
            print(f"Failed to parse line: {line}")

# Create a DataFrame from the parsed data
df = pd.DataFrame({
    'Time Received': pd.to_datetime(time_received, unit='s'),
    'Lidar Time': pd.to_datetime(lidar_time, unit='ns'),
    'Camera Time': pd.to_datetime(camera_time, unit='ns')
})

# Calculate differences
df['Lidar Delta'] = (df['Time Received'] - df['Lidar Time']).dt.microseconds*0.001
df['Camera Delta'] = (df['Time Received'] - df['Camera Time']).dt.microseconds*0.001
df['Time Difference'] = (df['Lidar Time'] - df['Camera Time']).dt.microseconds.abs()*0.001
df['Consecutive sync time difference'] = df['Time Received'].diff().abs().dt.total_seconds()
# Calculate the time since the first message
df['Time Since First Received'] = (df['Time Received'] - df['Time Received'].min()).dt.total_seconds()
# Plotting
plt.figure(figsize=(12, 8))

# Plot the delta times
plt.scatter(df['Time Since First Received'][1:],df['Consecutive sync time difference'][1:], label='Delta Time', color='blue')
plt.title('Time Difference Between sync')
plt.xlabel('Time Since First Received')
plt.ylabel('Consecutive sync time difference')
plt.legend()
plt.grid(True)

# Plotting
plt.figure(figsize=(12, 8))

# Plot the delta times
plt.subplot(2, 1, 1)
plt.scatter(df['Time Received'], df['Lidar Delta'], label='Delta Time (Lidar)', color='blue')
plt.scatter(df['Time Received'], df['Camera Delta'], label='Delta Time (Camera)', color='green')
plt.title('Time Difference Between Receipt and Message Times')
plt.xlabel('Time Received')
plt.ylabel('Delta Time (ms)')
plt.legend()
plt.grid(True)

# Plot the difference between Lidar and Camera times
plt.subplot(2, 1, 2)
plt.scatter(df['Time Received'], df['Time Difference'], label='Time Difference Between Lidar and Camera', color='red')
plt.title('Difference Between Lidar and Camera Timestamps')
plt.xlabel('Time Received')
plt.ylabel('Time Difference (ms)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.scatter(df['Lidar Delta'], df['Camera Delta'], alpha=0.5)
plt.title('Scatter Plot of Lidar vs. Camera Time Differences')
plt.xlabel('Lidar Delta (ms)')
plt.ylabel('Camera Delta (ms)')
plt.grid(True)
plt.show()

# To explore the relationship between Lidar and Camera time differences
plt.figure(figsize=(10, 6))
df[['Lidar Delta', 'Camera Delta']].boxplot()
plt.title('Box Plot of Time Differences for Lidar and Camera')
plt.ylabel('Time Difference (ms)')
plt.grid(True)
plt.show()

from mpl_toolkits.mplot3d import Axes3D

# Create the figure and a 3D subplot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Convert 'Time Received' to a float for plotting (number of seconds since the epoch)
time_received_seconds = (df['Time Received'] - pd.Timestamp("1970-01-01")) // pd.Timedelta('1s')

# Plotting
scatter = ax.scatter(df['Lidar Delta'], df['Camera Delta'], time_received_seconds,
                     c=time_received_seconds, cmap='viridis', marker='o')

# Create color bar
cbar = fig.colorbar(scatter, ax=ax, pad=0.1)
cbar.set_label('Time Received (seconds since epoch)')

# Set labels
ax.set_xlabel('Lidar Delta (ms)')
ax.set_ylabel('Camera Delta (ms)')
ax.set_zlabel('Time Received (Epoch Seconds)')

# Set title
ax.set_title('3D Scatter Plot of Time Deltas vs. Time Received')

# Show the plot
plt.show()

# Calculate the time since the first message
df['Time Since First Received'] = (df['Time Received'] - df['Time Received'].min()).dt.total_seconds()

# Create the figure and a 3D subplot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plotting
scatter = ax.scatter(df['Lidar Delta'], df['Camera Delta'], df['Time Since First Received'],
                     c=df['Time Since First Received'], cmap='viridis', marker='o')  # Color by time since first received

# Create color bar
cbar = fig.colorbar(scatter, ax=ax, pad=0.1)
cbar.set_label('Seconds Since First Message')

# Set labels
ax.set_xlabel('Lidar Delta (ms)')
ax.set_ylabel('Camera Delta (ms)')
ax.set_zlabel('Seconds Since First Message')

# Set title
ax.set_title('3D Scatter Plot of Time Deltas vs. Time Since First Message Received')

plt.show()