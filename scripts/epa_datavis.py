from dataclasses import dataclass
from typing import List, Mapping, TypedDict
from wpiutil.log import *
import matplotlib.pyplot as plt
import numpy as np
import re
import struct

wpilog = DataLogReader(
    # "/home/matt/Documents/GitHub/gtsam-playground/data/factor_graph_reference_1.wpilog"
    # "/home/matt/Downloads/harry-gtsam-data/Log_24-04-20_08-56-21_e2_sim.wpilog"
    "/mnt/d/Documents/matt logs ebr 20240516/logs_matt/matt_1715909200111.wpilog"
)


@dataclass
class TopicData:
    start: StartRecordData
    timestamps: List[int]


# map of topic ID to TopicData
topicMap: Mapping[int, TopicData] = {}

# (expanding) list of regexes to test topics against
ignoredTopics = [
    "/gtsam_meme/Camera[1-4]/input/cam_intrinsics",
    "/.schema.*",
    ".*robotTcam",
]

# list of twists (hacked in)
twistTimes = []
twists = []

for msg in wpilog:

    if msg.isStart():
        start: StartRecordData = msg.getStartData()

        # print("Got start for " + start.name)

        if any([re.match(it, start.name) for it in ignoredTopics]):
            # print("ignoring " + start.name)
            pass
        else:
            topicMap[start.entry] = TopicData(start, [])

            if (
                start.type.endswith("[]")
                or start.type.startswith("struct:")
                or start.type.startswith("proto:")
            ):
                ignoredTopics.append(start.name)
        continue

    if msg.isControl():
        continue

    if msg.isSetMetadata():
        continue

    if msg.isFinish():
        topicMap.pop(msg.getFinishEntry())
        continue

    entry = msg.getEntry()
    if entry in topicMap:
        # print name and time
        print(f"{topicMap[entry].start.name} @ {msg.getTimestamp()}")

        # topic = topicMap[entry].name
        topicMap[entry].timestamps.append(msg.getTimestamp())

        if topicMap[entry].start.name == "/gtsam_meme/input/odom_twist":
            twistTimes.append(msg.getTimestamp())
            twists.append(struct.unpack("<dddddd", msg.getRaw()))


# graph pose twist

twists = np.array(twists)
plt.figure(2)
for i in range(6):
    if i == 0:
        plt.subplot(211)
    if i == 3:
        plt.subplot(212)
    plt.plot(twistTimes, twists[:, i])

plt.subplot(211)
plt.legend(["dx", "dy", "dz"])
plt.title("Robot twists vs time (uS)")
plt.subplot(212)
plt.legend(["rx", "ry", "rz"])

# Plot stuff!

plt.figure(1)

legend = []

print("Stats")
row_format = "{:>40} | {:<10} | {:<10}"
print(row_format.format("Topic Name", "Count", "Frez [Hz]"))

for [topicID, data] in topicMap.items():
    legend.append(data.start.name)

    times = data.timestamps
    yval = np.full((len(times), 1), topicID)
    plt.scatter(times, yval)

    if len(times) > 1:
        mean_dt = round(1 / (np.mean(np.diff(times)) / 1e6), 1)
    else:
        mean_dt = "N/A"

    print(row_format.format(data.start.name, len(times), mean_dt))

plt.legend(legend)
plt.tight_layout()
plt.show()
