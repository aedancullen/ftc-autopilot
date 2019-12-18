
# Extremely quick+basic tool for drawing the robot location graphically.
# Ensure adb is in the PATH.

import turtle as t
import math
import subprocess

FIELD_FN = "field-grid-basic.gif"
FIELD_X = 500
FIELD_Y = 500
FIELD_SCALE = 500
TITLE = b"Autopilot Visualizer: "
TAG = b"AutopilotVisBcast"
PIXELS_PER_INCH = FIELD_SCALE/144


def itop(i):
    return i * PIXELS_PER_INCH

def seth(heading):
    t.seth((heading * 180 / math.pi) + 90)

t.delay(0)
t.speed(0)
t.tracer(0,0)
seth(0)
rdim = itop(18) / 2
#t.addshape("robot", ((-rdim,-rdim),(0,-rdim/1.5),(rdim,-rdim),(0,rdim)))
t.addshape("robot", (
    (-rdim,-rdim),
    (-rdim, rdim),
    (-rdim/4,rdim),
    (0,rdim*1.25),
    (rdim/4,rdim),
    (rdim,rdim),
    (rdim,-rdim)
    )
)
t.shape("robot")
t.pensize(3)
t.fillcolor("lightgray")
t.pencolor("orange")
t.title(TITLE + b"waiting...")
t.setup(width=FIELD_X+50, height=FIELD_Y+50)
t.bgpic(FIELD_FN)
t.update()

last_status = b"running"
def update(status, x, y, h):
    global last_status

    if last_status == b"stopped" and (status == b"running" or status == b"orienting"):
        t.clear()

    if last_status == b"running" or last_status == b"orienting":
        t.pd()
    else:
        t.pu()

    seth(h)
    t.setpos(itop(x), itop(y))

    t.title(TITLE + b"nav " + status)

    last_status = status
    t.update()

subprocess.call(["adb", "logcat", "-c"])
logcat = subprocess.Popen(["adb", "logcat"], stdout=subprocess.PIPE,stderr=subprocess.STDOUT).stdout

def check_logcat():
    global logcat

    line = logcat.readline()
    if not line:
        return
    if TAG in line:
        line = line[line.find(TAG):]
        line = line.rstrip(b" \r\n")
        line = line.split(b" ")[1]
        status, x, y, h = line.split(b",")
        update(status, float(x),float(y),float(h))

    t.ontimer(check_logcat, 15)

print("Connected to logcat, press Ctrl-C to quit")
t.ontimer(check_logcat, 15)
t.mainloop()
