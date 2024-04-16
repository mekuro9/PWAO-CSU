
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import pytz

ros_times = []
sent_times = []
counts = []

with open('/home/moro/test_log/rosbridge/reliable_3_pub.txt', 'r') as file:
    
    
    for line in file:
        if "Time sent:" in line:
            # Extract ROS time
            ros_time_str = line.split('[')[2].split(']')[0]
            ros_time = float(ros_time_str)

            # Extract time sent
            time_sent_str = line.split("'")[1]
            time_sent = datetime.fromisoformat(time_sent_str).replace(tzinfo= None)

        elif "Count:" in line:
            # Extract count
            count_str = line.split("'")[1]
            count = int(count_str)

            # Append to lists
            ros_times.append(ros_time)
            sent_times.append(time_sent)
            counts.append(count)

df = pd.DataFrame({
    'ROS Time': ros_times,
    'Time Sent': sent_times,
    'Count': counts
})

cest = pytz.timezone('Europe/Berlin')

df['ROS Time'] = pd.to_datetime(df['ROS Time'], unit='s', utc=True).dt.tz_convert(cest)

df['Time Sent'] = df['Time Sent'].dt.tz_localize('Europe/Brussels').dt.tz_convert(cest)

# Calculate latency as the difference in seconds
df['Latency'] = (df['ROS Time'] - df['Time Sent']).dt.total_seconds()
df['Latency ms'] = df['Latency']*1000
print(df[['ROS Time', 'Time Sent', 'Latency', 'Count']])  # Display for verification


# Display DataFrame for verification
print(df)

df['Packet Loss'] = df['Count'].diff() - 1
df['Packet Loss'] = df['Packet Loss'].fillna(0).astype(int)

plt.figure(figsize=(10, 6))
plt.plot(df['Count'], df['Latency ms'], marker='o')
plt.xlabel('Message Count')
plt.ylabel('Latency (ms)')
plt.title('Latency of Rosbridge ')
plt.show()

total_packet_loss = df['Packet Loss'].sum()
print(f"Total Packet Loss: {total_packet_loss}")

# Calculate jitter as the absolute difference in latency between successive packets
df['Jitter'] = df['Latency'].diff().abs()
df['Jitter ms'] = df['Jitter']*1000

# Set up the plot with two subplots
fig, ax = plt.subplots(2, 1, figsize=(12, 8))

# Plot Latency
ax[0].plot(df['Count'], df['Latency ms'], marker='o', color='blue')
ax[0].set_title('Latency (Integration Sevice)')
ax[0].set_xlabel('Message Count')
ax[0].set_ylabel('Latency (ms)')
ax[0].grid(True)

# Plot Jitter
ax[1].plot(df['Count'][1:], df['Jitter ms'][1:], marker='o', color='red') # Skipping the first NaN value
ax[1].set_title('Jitter (Integration Service)')
ax[1].set_xlabel('Message Count')
ax[1].set_ylabel('Jitter (s)')
ax[1].grid(True)

plt.tight_layout()
plt.show()

