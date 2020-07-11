
from platformio.util import get_serial_ports

import time
import sys
import serial
import subprocess

Import("env")
Import("projenv")

DEVID_DFU = "vid:pid,0483:df11"

env.Replace(UPLOADERFLAGS=[
        "-d", DEVID_DFU,
        "-a", "0", "-R", "-D"
    ])

UPLOADER = env["UPLOADER"]

def before_upload(source, target, env):
    port = None
    for p in get_serial_ports():
        if "0483:5740" in p.get("hwid", ""):
            if port is not None:
                sys.stderr.write("\nToo many RBCX ports!\n\n")
                sys.stderr.flush()
                sys.exit(1)
            port = p

    if port is None:
        return

    print("Sending magic RBCX sequence to %s for DFU reset..." % port["port"])
    try:
        with serial.Serial(port["port"], baudrate=12345, timeout=0.1, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO):
            time.sleep(0.01)
    except Exception as e:
        print("    Got exeption, might be okay anyway: %s" % e)

    print("Waiting for USB re-enumeration...")
    for i in range(30):
        out = subprocess.check_output([UPLOADER, "-d", DEVID_DFU, "-l"])
        if b"Found DFU" in out:
            return
        time.sleep(0.5)

    sys.stderr.write("\nTimed out while waiting for DFU!\n\n")
    sys.stderr.flush()
    sys.exit(1)

env.AddPreAction("upload", before_upload)
