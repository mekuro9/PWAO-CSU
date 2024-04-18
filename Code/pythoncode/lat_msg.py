import pandas as pd
import matplotlib.pyplot as plt

# Load the log file
file_path = '/home/moro/test_log/is/test4_re/test4_re.txt'  # Update this with the path to your log file
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


# Remove entries for the message size of 1000000 bytes
#df = df[df['Size'] != 1000000]

#print(df)
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

# Box plotprint(da)
#for Jitter vs. Message Size
plt.figure(figsize=(8, 6))
jitter_data = df.groupby('Size')['Jitter'].apply(list).reset_index(name='Jitters')
plt.boxplot(jitter_data['Jitters'], labels=jitter_data['Size'].astype(str))
plt.title('Jitter vs. Message Size')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Jitter (ms)')
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


# Define color palettes
blue_shades = ['#003f5c', '#2f4b7c', '#665191', '#a05195', '#d45087', '#f95d6a']
golden_shades = ['#ffdd57', '#ffcc33', '#ffbb00', '#e6ac00', '#cca900', '#b38f00']

# Box plot for Latency vs. Message Size
plt.figure(figsize=(8, 6))
boxplot_data = df.groupby('Size')['Latency'].apply(list).reset_index(name='Latencies')
bp_latency = plt.boxplot(
    boxplot_data['Latencies'], labels=boxplot_data['Size'].astype(str),
    patch_artist=True  # This is needed to fill the boxes with color
)
# Set colors for latency boxplot
for patch, color in zip(bp_latency['boxes'], blue_shades[:len(bp_latency['boxes'])]):
    patch.set_facecolor(color)
# Set y-axis to logarithmic scale
plt.yscale('log')
plt.title('Latency vs. Message Size (Log Scale)')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Latency (ms, log scale)')
plt.show()

print(df)
# Sort the DataFrame by 'Message Number' to ensure order

# Analyze each group
grouped = df.groupby('Size')
missing_summary = []

for size, group in grouped:
    # Sort each group by 'Message Number'
    group = group.sort_values('Message Number')

    # Find missing messages within each group
    all_messages = set(range(group['Message Number'].min(), group['Message Number'].max() + 1))
    received_messages = set(group['Message Number'])
    missing_messages = all_messages - received_messages

    # Store the results
    missing_summary.append({
        'Size': size,
        'Missing Messages': sorted(missing_messages),
        'Number of Missing Messages': len(missing_messages),
        'Percentage Missing': 100 * len(missing_messages) / (group['Message Number'].max() - group['Message Number'].min() + 1)
    })

# Convert summary to DataFrame for better visualization
missing_summary_df = pd.DataFrame(missing_summary)
print(missing_summary_df)

plt.figure(figsize=(10, 5))
plt.bar(missing_summary_df['Size'].astype(str), missing_summary_df['Percentage Missing'], color='blue')
plt.xlabel('Message Size (bytes)')
plt.ylabel('Percentage of Messages Missing')
plt.title('Packet Loss by Message Size')
plt.xticks(rotation=45)  # Rotate labels for better readability
plt.grid(True)
plt.show()