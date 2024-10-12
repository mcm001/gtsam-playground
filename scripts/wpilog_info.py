from collections import namedtuple
from dataclasses import dataclass
from wpiutil.log import DataLogReader, DataLogRecord, StartRecordData
from typing import Dict, List
import os
import struct

@dataclass
class Schema:
    unpack: str
    tuple: namedtuple

for file in os.listdir("logs"):
    reader = DataLogReader(f"logs/{file}")

    startRecordsByTopic: Dict[int, StartRecordData] = {}

    messagesByTopic: Dict[int, list] = {}

    schemasByTypename: Dict[str, Schema] = {}

    i = -1
    for entry in reader:
        i = i + 1
        if entry.isStart():
            data = entry.getStartData()

            # print(f"Start record on topic {data.entry} : {data.name}")

            startRecordsByTopic[data.entry] = data
            messagesByTopic[data.entry] = []
        elif entry.isFinish():
            data = entry.getFinishEntry()
            # print(f"Finish record on topic {data}")
            pass
        elif entry.isControl():
            # print(f"Control record on topic {data}")
            pass
        elif entry.isSetMetadata():
            # print(f"Set metadata record on topic {data}")
            pass
        else:
            # print(f"Normal message published on topic ID {entry.getEntry()}: len {entry.getSize()}")

            if entry.getEntry() in startRecordsByTopic.keys():
                messagesByTopic[entry.getEntry()].append(entry)
            else:
                print(f"Unknown message encountered")
                continue

            startData = startRecordsByTopic[entry.getEntry()]

            if ".schema/struct:" in startData.name:
                # huge hack
                if "TagDetection" in startData.name:
                    schemasByTypename[startData.name[startData.name.index("struct:") + len("struct:"):]] = Schema(
                        "<Ldddddddd",
                        namedtuple('TagDetection', 'id cx1 cy1 cx2 cy2 cx3 cy3 cx4 cy4')
                    )
                if "Twist3d" in startData.name:
                    schemasByTypename[startData.name[startData.name.index("struct:") + len("struct:"):]] = Schema(
                        "<dddddd",
                        namedtuple('Twist3d', 'dx dy dz rx ry rz')
                    )

            # apparently saving the record for later is a Bad Idea? so process it immediately
            if "struct:" in startData.type:
                topic_typestring = startData.type

                if topic_typestring.endswith("[]"):
                    schema = schemasByTypename[topic_typestring[len("struct:"):-2]]

                    inner = []
                    data = entry.getRaw()
                    inner_len = struct.calcsize(schema.unpack)
                    for i in range(0, entry.getSize(), inner_len):
                        decoded = schema.tuple._make(struct.unpack(schema.unpack, data[i:i+inner_len]))
                        inner.append((entry.getTimestamp(), decoded))
                    messagesByTopic[entry.getEntry()].append(inner)
                    print(inner)
                else:
                    schema = schemasByTypename[topic_typestring[len("struct:"):]]

                    decoded = schema.tuple._make(struct.unpack(schema.unpack, entry.getRaw()))
                    messagesByTopic[entry.getEntry()].append((entry.getTimestamp(), decoded))

                    print(decoded)


    print("")
    print(f"Topics in {file}:")
    for (key, topic) in startRecordsByTopic.items():
        # filter
        if "gtsam" not in topic.name:
            continue

        print("  " + topic.name + f": {len(messagesByTopic[key])} messages")

    print("=========")
