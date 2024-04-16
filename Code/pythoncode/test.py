import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import pytz

ros_times = []
sent_times = []
counts = []

# remember to change time

with open('/home/moro/test_log/rosbridge/reliable_3_pub_phone.txt', 'r') as file:
    
    
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
df['Latency'] = df['Latency']*1000
print(df[['ROS Time', 'Time Sent', 'Latency', 'Count']])  # Display for verification

# Define start time and frequency intervals
test_start_time = datetime.fromisoformat('2024-04-15T02:35:21.025562+02:00')  # First message time
frequency_durations = [3.0, 2.0, 1.0, 0.5, 0.3, 0.1]  # in seconds
frequency_change_times = [test_start_time + timedelta(minutes=i) for i in range(len(frequency_durations))]


# Assign frequencies to each message based on the time sent
def assign_frequency(time_sent):
    for i, change_time in enumerate(frequency_change_times):
        if time_sent < change_time:
            return frequency_durations[i - 1] if i > 0 else frequency_durations[0]
    return frequency_durations[-1]

df['Frequency'] = df['Time Sent'].apply(assign_frequency)
print(df[['ROS Time', 'Time Sent', 'Latency', 'Count', 'Frequency']])  # Display for verification

# Step 2: Plotting in Separate Windows

# Assuming 'df' is your DataFrame and it includes 'Count', 'Latency', and 'Jitter' columns

# Plot Latency vs. Message Count using only dots
plt.figure(figsize=(10, 6))
plt.scatter(df['Count'], df['Latency'], color='blue', s=10)  # `s` controls the size of the dots
plt.title('Latency Over Messages')
plt.xlabel('Message Count')
plt.ylabel('Latency (ms)')
plt.grid(True)
plt.show()


# Assuming jitter and packet loss are calculated
df['Jitter'] = df['Latency'].diff().abs()  # Sample calculation for jitter
df['Packet Loss'] = df['Count'].diff() - 1  # Sample calculation for packet loss

# Plotting Jitter in a new window
# Plot Jitter vs. Message Count using only dots
plt.figure(figsize=(10, 6))
plt.scatter(df['Count'][1:], df['Jitter'][1:], color='green', s=10)  # Skip the first point if it's NaN
plt.title('Jitter Over Messages')
plt.xlabel('Message Count')
plt.ylabel('Jitter (ms)')
plt.grid(True)
plt.show()


# Group by frequency and calculate mean for each metric
frequency_group = df.groupby('Frequency').mean()  # Using mean; you can also use median or other statistics

# Reset index to make 'Frequency' a column again if it's set as index after grouping
frequency_group = frequency_group.reset_index()

# Step 2: Plotting Metrics vs. Frequency
# Plot Latency vs. Frequency
plt.figure(figsize=(10, 6))
plt.plot(frequency_group['Frequency'], frequency_group['Latency'], marker='o', linestyle='-', color='blue')
plt.title('Average Latency vs. Frequency')
plt.xlabel('Frequency (seconds)')
plt.ylabel('Average Latency (ms)')
plt.grid(True)
plt.gca().invert_xaxis()  # Invert x-axis to show decreasing frequency from left to right
plt.show()

# Plot Jitter vs. Frequency
plt.figure(figsize=(10, 6))
plt.plot(frequency_group['Frequency'], frequency_group['Jitter'], marker='o', linestyle='-', color='green')
plt.title('Average Jitter vs. Frequency')
plt.xlabel('Frequency (seconds)')
plt.ylabel('Average Jitter (ms)')
plt.grid(True)
plt.gca().invert_xaxis()  # Often more logical to show higher frequency (lower interval) on the right
plt.show()

# Assuming 'df' has a 'Frequency' column

plt.figure(figsize=(12, 6))
plt.hist(df['Latency'], bins=50, color='skyblue', log=False)  # 'log=True' enables the logarithmic scale
plt.title('Histogram of Latency')
plt.xlabel('Latency (ms)')
plt.ylabel('Frequency(no of messages)')
plt.grid(True)
plt.show()


plt.figure(figsize=(12, 6))
plt.hist(df['Jitter'], bins=50, color='red', log=False)  # 'log=True' enables the logarithmic scale
plt.title('Histogram of Jitter')
plt.xlabel('Jitter (ms)')
plt.ylabel('Frequency(no of messages)')
plt.grid(True)
plt.show()


import numpy as np
import scipy.stats as stats

# Assuming 'df' is your DataFrame and it includes a 'Latency' column

# Calculate mean and standard deviation for the latency data
latency_mean = df['Latency'].mean()
latency_std = df['Latency'].std()

# Set the number of bins for the histogram
num_bins = 50

# Plot the histogram
plt.figure(figsize=(12, 6))
n, bins, patches = plt.hist(df['Latency'], num_bins, density=True, alpha=0.6, color='skyblue')

# Add a 'best fit' line for the normal PDF (Probability Density Function)
y = ((1 / (np.sqrt(2 * np.pi) * latency_std)) *
     np.exp(-0.5 * (1 / latency_std * (bins - latency_mean))**2))
plt.plot(bins, y, '--', color='black')

plt.title('Normalized Histogram of Latency with Bell Curve')
plt.xlabel('Latency (ms)')
plt.ylabel('Density')

# Add percentage format to the y-axis
plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: '{:.0%}'.format(y)))

plt.grid(True)
plt.show()


# Determine the number of bins for the histogram
bins = range(0, int(max(df['Latency'])), 20)  # adjust the bin range and width as needed

# Plot the histogram
plt.figure(figsize=(12, 6))

# Calculate the histogram data to get the bin counts
counts, bin_edges = np.histogram(df['Latency'], bins=bins)

# Convert the counts to a percentage of the total
counts_percentage = (counts / counts.sum()) * 100

# Plot the percentages
plt.bar(bin_edges[:-1], counts_percentage, width=np.diff(bin_edges), edgecolor='black', align='edge')

plt.title('Histogram of Latency Percentages')
plt.xlabel('Latency (ms)')
plt.ylabel('Percentage of Total Messages (%)')

# Set the y-axis to display percentages
plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: '{:.0%}'.format(y / 100)))

plt.grid(True)
plt.show()