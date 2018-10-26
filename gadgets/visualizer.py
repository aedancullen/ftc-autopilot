
# Extremely quick+basic tool for drawing the robot location graphically.
# Ensure adb is in the PATH.

import turtle as t
import math
import subprocess
import os
import fcntl

FIELD_FN = "field-grid.gif"
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

last_status = "running"
def update(status, x, y, h):
    global last_status

    if last_status == "stopped" and (status == "running" or status == "orienting"):
        t.clear()

    if last_status == "running" or last_status == "orienting":
        t.pd()
    else:
        t.pu()

    seth(h)
    t.setpos(x, y)

    t.title(TITLE + status.upper())

    last_status = status
    t.update()


logcat = subprocess.Popen(["adb", "logcat"], stdout=subprocess.PIPE,stderr=subprocess.STDOUT).stdout
flag = fcntl.fcntl(logcat, fcntl.F_GETFL)
fcntl.fcntl(logcat, fcntl.F_SETFL, flag | os.O_NONBLOCK)

def check_logcat():
    global logcat

    while True:
        try:
            line = os.read(logcat.fileno(), 1024)
        except Exception as e:
            t.ontimer(check_logcat, 15)
            return
        if TAG in line:
            line = line[line.find(TAG):]
            line = line.split(b" ")[1]
            line = line.split(b"\r\n")[0]
            status, x, y, h = line.split(b",")
            update(status, float(x),float(y),float(h))

    t.ontimer(check_logcat, 15)


t.ontimer(check_logcat, 15)
t.mainloop()
