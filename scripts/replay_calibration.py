from datetime import date
import json
import re
from time import sleep
from typing import List
from wpilib import SmartDashboard
from wpiutil.log import *
import struct
from dataclasses import dataclass
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from photonlibpy.packet import Packet
from wpimath.geometry import Translation2d
from ntcore import FloatArrayPublisher, FloatArrayTopic, GenericPublisher, NetworkTableInstance, PubSubOptions, Value, _now

inst = NetworkTableInstance.getDefault()
inst.stopServer()
inst.setServer("127.0.0.1")
inst.startClient4("cal-replay")

pubMap = {}


def getPublisher(path) -> FloatArrayPublisher:
    if path not in pubMap:
        pubMap[path] = inst.getFloatArrayTopic(path).publish(PubSubOptions())

    return pubMap[path]


# order is front left, front right, back left, back right
# NT names are Camera1, Camera2, Camera3, Camera4
def publish():
    for [ntName, jsonName] in zip(
        ["Camera1", "Camera2", "Camera3", "Camera4"],
        ["front-left.json", "front-right.json", "back-left.json", "back-right.json"],
    ):
        data = json.load(open("data/" + jsonName))

        intrinsics = data["cameraIntrinsics"]["data"]

        # print("Publishing for camera " + ntName)
        getPublisher(f"/gtsam_meme/{ntName}/input/cam_intrinsics").setDefault(
            [intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]]
        )
        getPublisher(f"/gtsam_meme/{ntName}/input/cam_distortion").setDefault(
            data["distCoeffs"]["data"]
        )
        getPublisher(f"/gtsam_meme/{ntName}/input/cam_intrinsics").set(
            [intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]]
        )
        getPublisher(f"/gtsam_meme/{ntName}/input/cam_distortion").set(
            data["distCoeffs"]["data"]
        )


while True:
    publish()
    sleep(5)
