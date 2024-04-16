import pandas as pd
import matplotlib.pyplot as plt

# Load the log file
file_path = '/home/moro/test_log/rosbridge/latency_vary_msg_size.txt'  # Update this with the path to your log file
with open(file_path, 'r') as file:
    log_lines = file.readlines()


# Parse the logs
data = []
for line in log_lines:
    if 'Received message' in line:
        parts = line.split()
        message_number = int(parts[2].strip(':'))
        size = int(parts[4])
        
        # Extract numerical data safely by stripping and splitting correctly
        sent_at_index = parts.index('at') + 1  # Find the index of 'at' and take the next element
        received_at_index = parts.index('received') + 2  # Find the index of 'received' and take the element two places after
        latency_index = parts.index('latency:') + 1  # Find the index of 'latency:' and take the next element
        
        sent_timestamp = float(parts[sent_at_index].strip('ms,'))
        received_timestamp = float(parts[received_at_index].strip('ms,'))
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

# Calculate Jitter (difference in Latency between successive packets)
df['Jitter'] = df['Latency'].diff().abs()

# Box plot for Latency vs. Message Size
plt.figure(figsize=(8, 6))
boxplot_data = df.groupby('Size')['Latency'].apply(list).reset_index(name='Latencies')
plt.boxplot(boxplot_data['Latencies'], labels=boxplot_data['Size'].astype(str))
plt.title('Latency vs. Message Size')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Latency (ms)')
plt.show()

# Box plot for Jitter vs. Message Size
plt.figure(figsize=(8, 6))
jitter_data = df.groupby('Size')['Jitter'].apply(list).reset_index(name='Jitters')
plt.boxplot(jitter_data['Jitters'], labels=jitter_data['Size'].astype(str))
plt.title('Jitter vs. Message Size')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Jitter (ms)')
plt.show()

# Optional Plot: Histogram of Latencies
plt.figure(figsize=(8, 6))
plt.hist(df['Latency'], bins=20, color='blue', alpha=0.7)
plt.title('Histogram of Latencies')
plt.xlabel('Latency (ms)')
plt.ylabel('Frequency')
plt.show()

plt.figure(figsize=(8, 6))
boxplot_data = df.groupby('Size')['Jitter'].apply(list).reset_index(name='Jitters')
bp = plt.boxplot(
    boxplot_data['Jitters'],
    labels=boxplot_data['Size'].astype(str),
    patch_artist=True  # This is needed to fill the boxes with color
)

# Set colors for each box
colors = ['skyblue', 'lightgreen', 'lightcoral', 'wheat', 'lightpink', 'khaki', 'grey']
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)

plt.title('Jitter vs. Message Size')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Jitter (ms)')
plt.show()
