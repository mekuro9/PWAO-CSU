import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Load the log file
file_path = '/home/moro/test_log/rosbridge/Test2/test2.txt'  # Update this with the path to your log file
with open(file_path, 'r') as file:
    log_lines = file.readlines()


# Parse the logs
data = []
for line in log_lines:
    if 'Received message' in line:
        parts = line.split()
        message_number = int(parts[2].strip(':'))
        size = int(parts[4])
        sent_at_index = parts.index('at') + 1
        received_at_index = parts.index('received') + 2
        latency_index = parts.index('latency:') + 1
        sent_timestamp = float(parts[sent_at_index].strip('ms,'))
        received_timestamp = float(parts[received_at_index].strip('ms'))
        latency = float(parts[latency_index].strip('ms'))
        data.append({
            'Message Number': message_number,
            'Size': size,
            'Sent Timestamp': sent_timestamp,
            'Received Timestamp': received_timestamp,
            'Latency': latency
        })

# Create DataFrame
df = pd.DataFrame(data)


# Remove entries for the message size of 1000000 bytes
#{30, 100, 317, 1000, 3162, 10000, 31623, 100000, 316228, 1000000}; // Byte sizes
df = df[df['Size'] != 1000000]
#   df = df[df['Size'] != 316228]

#print(df)
# Calculate Jitter (difference in Latency between successive packets)
df['Jitter'] = df['Latency'].diff().abs()

sns.set_theme(style="whitegrid")
peacock_palette = ['#16eaff',  # Purple
                   '#00e6fc',  # Cyan
                   '#00d7ec',  # Green
                   '#00c8dc',  # Light blue
                   '#00b9cc',  # Dark cyan
                   '#0ab',
                   '#009cab',
                   '#007e8a',
                   '#005961']  # Light green

# Set the custom palette
# Box plot for Latency vs. Message Size
plt.figure(figsize=(8, 6))
sns.boxplot(y=df['Latency'], x = df['Size'], hue = df['Size'],log_scale = 10,  data=df, palette=peacock_palette,flierprops=dict(marker='*', markersize=3),whis=1.5, legend=False)
plt.title('Latency(one way) vs. Message Size: Reliable QoS with Rosbridge')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Latency (ms)')
plt.show()

gold_palette     = ['#eeb127',  # Purple
                   '#eaa812',  # Cyan
                   '#d39710',  # Green
                   '#bd870e',  # Light blue
                   '#ad7c0d',  # Dark cyan
                   '#9e710c',
                   '#976c0b',
                   '#8f670b',
                   '#5a4107']  # Light green
# Box plot for Jitter vs. Message Size
plt.figure(figsize=(8, 6))
sns.boxplot(y=df['Jitter'], x = df['Size'], hue = df['Size'],log_scale = 10,  data=df, palette=gold_palette,flierprops=dict(marker='*', markersize=3),whis=1.5, legend=False)
plt.title('Jitter vs. Message Size: Reliable QoS with Rosbridge')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Jitter (ms)')
plt.show()