import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Load the log file
file_path = '/home/moro/test_log/rosbridge/Test1/test1.txt'  # Update this with the path to your log file
with open(file_path, 'r') as file:
    log_lines = file.readlines()
    
# Initialize lists to hold parsed data
send_times = []
receive_times = []

# Read and parse the log file
with open(file_path, 'r') as file:
    for line in file.readlines():
        if 'Sending count:' in line:
            parts = line.split()
            count = int(parts[2])  # Get the count number
            timestamp = float(parts[4])  # Get the timestamp after "at"
            send_times.append({'Count': count, 'Timestamp': timestamp})
        elif 'Received echo:' in line:
            parts = line.split()
            count = int(parts[4])  # Get the count number after "ROS received:"
            timestamp = float(parts[6])  # Get the timestamp after "at"
            receive_times.append({'Count': count, 'Timestamp': timestamp})

# Create DataFrames
df_send = pd.DataFrame(send_times)
df_receive = pd.DataFrame(receive_times)

# Calculate RTT
df_rtt = pd.merge(df_send, df_receive, on='Count', suffixes=('_send', '_receive'))
df_rtt['RTT'] = df_rtt['Timestamp_receive'] - df_rtt['Timestamp_send']

# Detect packet loss
all_counts = set(df_send['Count'])
received_counts = set(df_receive['Count'])
lost_counts = all_counts - received_counts
packet_loss = len(lost_counts)
print(packet_loss)
# Calculate Jitter (difference in RTT between successive packets)
df_rtt['Jitter'] = df_rtt['RTT'].diff().abs()

# Display DataFrame for verification
print(df_rtt)

# Define bin width
bin_width = 5  # Set your desired bin width here

# Calculate number of bins for each dataset
num_bins1 = int(np.ceil((df_rtt['RTT'].max()*1000 - df_rtt['RTT'].min()*1000)/ bin_width))
print(df_rtt['RTT'].max()*1000 - df_rtt['RTT'].min()*1000)
# Define bins
bins1 = np.linspace(df_rtt['RTT'].min()*1000, df_rtt['RTT'].max()*1000, num_bins1 + 1)

plt.figure(figsize=(12, 6))
sns.histplot(df_rtt['RTT']*1000, bins=bins1, color='#0ab',  stat='percent')   
plt.title('Round trip time: Reliable QOS using Rosbridge')
plt.xlabel('Round Trip Time latency (ms)')
plt.ylabel('Percent of total messages')
plt.xlim(0, 400)
plt.grid(True)
plt.show()

num_bins1 = int(np.ceil((df_rtt['Jitter'].max()*1000 - df_rtt['Jitter'].min()*1000)/ bin_width))
print(df_rtt['Jitter'].max()*1000 - df_rtt['Jitter'].min()*1000)
# Define bins
bins1 = np.linspace(df_rtt['Jitter'].min()*1000, df_rtt['Jitter'].max()*1000, num_bins1 + 1)

plt.figure(figsize=(12, 6)) 
sns.histplot(df_rtt['Jitter']*1000, bins=bins1, color='goldenrod',  stat='percent')   
plt.title('Jitter in round trip time: Reliable QOS using Rosbridge')
plt.xlabel('Jitter (ms)')
plt.xlim(0, 400)
plt.ylabel('Percent of total messages')
plt.grid(True)
plt.show()