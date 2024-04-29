import re
from time import sleep
from typing import List
from wpiutil.log import *
import struct

wpilog = DataLogReader(
    "/home/matt/Documents/GitHub/gtsam-playground/data/factor_graph_reference_1.wpilog"
)

from dataclasses import dataclass
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from photonlibpy.packet import Packet
from wpimath.geometry import Translation2d
from ntcore import GenericPublisher, NetworkTableInstance, Value, _now


@dataclass
class SwerveModuleState:
    speedMps: float
    angleRad: float


@dataclass
class TagDetection:
    id: int
    corner: List[Translation2d]


def decodeStates(raw: bytes):
    schema = "<dddddddd"
    state = struct.unpack(schema, raw)
    return [SwerveModuleState(state[i], state[i + 1]) for i in range(0, 8, 2)]


def decodeTagId(raw: bytes) -> TagDetection:
    schema = "<idddddddd"
    data = struct.unpack(schema, raw)
    return TagDetection(
        data[0], [Translation2d(data[i], data[i + 1]) for i in range(1, 9, 2)]
    )


def decodePacket(raw: bytes):
    if len(raw) < 1:
        return None

    result = PhotonPipelineResult().populateFromPacket(Packet(raw))
    print(result)
    return result


NetworkTableInstance.getDefault().stopClient()
NetworkTableInstance.getDefault().startServer()


def recordToNt(record: DataLogRecord, entry: GenericPublisher):
    type = entry.getTopic().getTypeString()
    print("publishing a " + type)

    # deal with proto up front
    if (
        type.startswith("proto:")
        or type.startswith("struct:")
        or type == "structschema"
    ):
        entry.setRaw(record.getRaw())
        return

    match type:
        case "int64":
            entry.setInteger(record.getInteger())
        case "double[]":
            entry.setDoubleArray(record.getDoubleArray())
        case "double":
            entry.setDouble(record.getDouble())
        case "boolean":
            entry.setBoolean(record.getBoolean())
        case "string":
            entry.setString(record.getString())
        case "string[]":
            entry.setStringArray(record.getStringArray())
        case "raw" | "rawBytes":
            entry.setRaw(record.getRaw())
        case _:
            raise Exception("Unknown type string " + type)


def topicNameToPublisher(start: StartRecordData):
    print(f"Adding new publisher: {start.name} of type {start.type}")
    return (
        NetworkTableInstance.getDefault()
        .getTopic(start.name)
        .genericPublish(start.type)
    )


topicMap = {}
ignoredTopics = ["NT:/cam/tags/.*", "NT:/photonvision/YOUR CAMERA NAME/rawBytes/.*"]

sysToNtOffset = None

for msg in wpilog:
    if msg.isStart():
        start: StartRecordData = msg.getStartData()

        if any([re.match(it, start.name) for it in ignoredTopics]):
            print("ignoring " + start.name)
        else:
            topicMap[start.entry] = topicNameToPublisher(start)

            if (
                start.type.endswith("[]")
                or start.type.startswith("struct:")
                or start.type.startswith("proto:")
            ):
                # print("adding to ignore: " + start.name)
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

        if sysToNtOffset is None:
            sysToNtOffset = _now() - msg.getTimestamp()

        while (msg.getTimestamp() + sysToNtOffset) > _now():
            dt = msg.getTimestamp() + sysToNtOffset - _now()
            print("Busywaiting for ms: " + str(dt / 1e3))
            sleep(dt / 1e6)

        pub: GenericPublisher = topicMap[entry]

        recordToNt(msg, pub)

        # print(pub.getTopic().getType())
        # ret = Value.makeValue(msg.getRaw())
        # print(f"{pub.getTopic().getName()} <- {ret}")
        # pub.set(ret)
