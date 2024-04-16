import pandas as pd
import matplotlib.pyplot as plt

# Load the log file
file_path = '/home/moro/test_log/rosbridge/RTT_28byte_RE.txt'  # Update this with the path to your log file
with open(file_path, 'r') as file:
    log_lines = file.readlines()
    
# Parse the logs
send_times = []
receive_times = []
for line in log_lines:
    if 'Sending count:' in line:
        parts = line.split()
        count = int(parts[3].strip(':'))
        # Assuming the timestamp is right before the IP:port part
        timestamp = float(parts[5])
        send_times.append({'Count': count, 'Timestamp': timestamp})
    elif 'Received echo:' in line:
        parts = line.split()
        count = int(parts[5].strip(':'))
        # Assuming the timestamp is right before the IP:port part
        timestamp = float(parts[7])
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

# Plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 15))
# RTT plot
axs[0].plot(df_rtt['Count'], df_rtt['RTT'], marker='o')
axs[0].set_title('Round Trip Time (RTT)')
axs[0].set_xlabel('Count')
axs[0].set_ylabel('RTT (seconds)')

# Jitter plot
axs[1].plot(df_rtt['Count'][1:], df_rtt['Jitter'][1:], marker='o', color='r')  # skip first NaN jitter
axs[1].set_title('Jitter')
axs[1].set_xlabel('Count')
axs[1].set_ylabel('Jitter (seconds)')

# Packet Loss
axs[2].bar(['Packet Loss'], [packet_loss])
axs[2].set_title('Packet Loss')
axs[2].set_ylabel('Lost Packets')

plt.tight_layout()
plt.show()


plt.figure(figsize=(12, 6))
plt.hist(df_rtt['RTT']*1000, bins=50, color='skyblue', log=False)  # 'log=True' enables the logarithmic scale
plt.title('Histogram of Round Trip Time using rosbridge')
plt.xlabel('Round Trip Time latency (ms)')
plt.ylabel('Frequency(no of messages)')
plt.grid(True)
plt.show()


plt.figure(figsize=(12, 6))
plt.hist(df_rtt['Jitter']*1000, bins=50, color='red', log=False)  # 'log=True' enables the logarithmic scale
plt.title('Histogram of round trip Jitter using rosbridge')
plt.xlabel('Jitter (ms)')
plt.ylabel('Frequency(no of messages)')
plt.grid(True)
plt.show()
