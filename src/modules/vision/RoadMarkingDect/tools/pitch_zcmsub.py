from zerocm import ZCM
import pdb
import sys
import time
sys.path.insert(0, '/home/neousys/Desktop/tiev2019/src/modules/Visual/tools/')
from structNAVINFO import structNAVINFO

pitch = 0
def handler(channel, msg):
    global pitch
    #pdb.set_trace()
    pitch  = msg.mPitch
    print(pitch)

zcm = ZCM("ipc")

if not zcm.good():

    print("Unable to initialize zcm")

    exit()

zcm.start()

subs = zcm.subscribe("NAVINFO", structNAVINFO, handler)

while 1:
    #zcm.handle()

    time.sleep(0.1)

zcm.stop()

zcm.unsubscribe(subs)



