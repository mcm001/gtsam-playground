import re
from time import sleep
from typing import List
from wpiutil.log import *
import struct

wpilog = DataLogReader(
    # "/home/matt/Documents/GitHub/gtsam-playground/data/factor_graph_reference_1.wpilog"
    # "/home/matt/Downloads/harry-gtsam-data/Log_24-04-20_08-56-21_e2_sim.wpilog"
    "/mnt/d/Documents/matt logs ebr 20240516/logs_matt/matt_1715909200111.wpilog"
)

from dataclasses import dataclass
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from photonlibpy.packet import Packet
from wpimath.geometry import Translation2d
from ntcore import GenericPublisher, NetworkTableInstance, Value, _now

inst = NetworkTableInstance.getDefault()
inst.stopClient()
inst.startServer()

while not any([it.remote_id.startswith("gtsam-meme") for it in inst.getConnections()]):
    print("Waiting for gtsam-meme: " + str([it.remote_id for it in inst.getConnections()]))
    sleep(1)

def recordToNt(record: DataLogRecord, entry: GenericPublisher):
    """
    Publish a record to NT
    """
    type = entry.getTopic().getTypeString()
    # print("publishing a " + type)

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
        case "float[]":
            entry.setFloatArray(record.getFloatArray())
        case "boolean":
            entry.setBoolean(record.getBoolean())
        case "string":
            entry.setString(record.getString())
        case "string[]":
            entry.setStringArray(record.getStringArray())
        case "int64[]":
            entry.setIntegerArray(record.getIntegerArray())
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


# map of topic ID to publisher
topicMap = {}
# (expanding) list of regexes to test topics against
ignoredTopics = ["NT:/cam/tags/.*", "NT:/photonvision/YOUR CAMERA NAME/rawBytes/.*"]

sysToNtOffset = None

for msg in wpilog:
    
    if msg.isStart():
        start: StartRecordData = msg.getStartData()

        print("Got start for " + start.name)

        if any([re.match(it, start.name) for it in ignoredTopics]):
            # print("ignoring " + start.name)
            pass
        else:
            topicMap[start.entry] = topicNameToPublisher(start)

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

        if sysToNtOffset is None:
            sysToNtOffset = _now() - msg.getTimestamp()

        # wait to sort-of-sync-clocks up
        while (msg.getTimestamp() + sysToNtOffset) > _now():
            dt = msg.getTimestamp() + sysToNtOffset - _now()
            # print("Busywaiting for ms: " + str(dt / 1e3))
            if dt > 0:
                sleep(dt / 1e6)

        pub: GenericPublisher = topicMap[entry]
        recordToNt(msg, pub)
