#!/usr/bin/env python3

import time
import os
from networktables import NetworkTablesInstance

team = 9999
audioFolder = "/home/pi/audio/"
audioExtension = "wav"
playbackKey = "/quacker/playback"
statusKey = "/quacker/status"


def listener(fromobj, key, value, isNew):
    if isinstance(value, float):
        if value >= 0 and value <= 99 and value % 1 == 0:
            NetworkTablesInstance.getDefault().getEntry(playbackKey).forceSetDouble(-1)
            audioStr = str(int(value)).zfill(2)
            os.system("aplay " + audioFolder + audioStr + "." + audioExtension)


if __name__ == "__main__":
    # Start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(team)
    ntinst.startDSClient()

    # Bind playback listener
    ntinst.getEntry(playbackKey).forceSetDouble(-1)
    ntinst.getEntry(playbackKey).addListener(
        listener, NetworkTablesInstance.NotifyFlags.UPDATE)

    # Update status entry
    i = 0
    while True:
        time.sleep(0.1)
        i += 1
        if i > 9:
            i = 0
        ntinst.getEntry(statusKey).forceSetDouble(i)
