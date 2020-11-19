import time
from zerocm import ZCM
from MsgTrafficLightSignal import MsgTrafficLightSignal

class ZcmTransceiver(object):
    def __init__(self):
        self.transceiver = ZCM('')
        if not self.transceiver.good():
            print("Unable to initialize ZeroCM")
            exit(-1)

    def __call__(self, left: bool = True, forward: bool = True, right: bool = True):
        msg_traffic_light_signal = MsgTrafficLightSignal()
        msg_traffic_light_signal.timestamp = int(time.time())
        msg_traffic_light_signal.left = left
        msg_traffic_light_signal.forward = forward
        msg_traffic_light_signal.right = right
        self.transceiver.publish("msgTrafficLightSignal", msg_traffic_light_signal)
        # print(str(msg_traffic_light_signal.left)+" "+str(msg_traffic_light_signal.forward)+" "+str(msg_traffic_light_signal.right))
