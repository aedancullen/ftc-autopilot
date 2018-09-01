
# Extremely quick+basic tool for drawing the robot location graphically.
# Ensure adb is in the PATH.

import turtle as t
import math
import subprocess
from subprocess import PIPE

TITLE = "Autopilot Visualizer: "
TAG = "AutopilotVisualizerBroadcast"
PIXELS_PER_INCH = 3.00


def itop(i):
    return i * PIXELS_PER_INCH

def seth(heading):
    t.seth((heading * 180 / math.pi) + 90)

t.delay(0)
t.speed(0)
t.tracer(0,0)
seth(0)
rdim = itop(18) / 2
t.addshape("robot", ((-rdim,-rdim),(0,-rdim/1.5),(rdim,-rdim),(0,rdim)))
t.shape("robot")
t.pensize(3)
t.fillcolor("lightgrey")
t.pencolor("orange")
t.title(TITLE + "waiting...")
t.update()

last_status = "running"
def update(status, x, y, h):
    global last_status

    if last_status == "stopped" and status == "running":
        t.clear()

    if last_status == "running":
        t.pd()
    else:
        t.pu()

    seth(h)
    t.setpos(x, y)

    t.title(TITLE + status.upper())

    last_status = status
    t.update()

logcat = subprocess.Popen(["adb", "logcat"], stdout=PIPE)
while True:
    line = logcat.stdout.readline()
    if not line:
        break
    if TAG in line:
        line = line[line.find(TAG):]
        line = line.split(" ")[1]
        status, x, y, h = line.split(",")
        update(status, float(x),float(y),float(h))
