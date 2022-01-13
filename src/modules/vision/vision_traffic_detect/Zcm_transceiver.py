import time
from zerocm import ZCM
from MsgTrafficLightSignal import MsgTrafficLightSignal

class ZcmTransceiver(object):
    def __init__(self):
        #self.transceiver = ZCM('ipc')
        self.transceiver = ZCM('udpm://239.255.76.67:7667?ttl=1')
        if not self.transceiver.good():
            print("Unable to initialize ZeroCM")
            exit(-1)

    def __call__(self, left: bool = True, forward: bool = True, right: bool = True):
        msg_traffic_light_signal = MsgTrafficLightSignal()
        msg_traffic_light_signal.timestamp = int(time.time())
        msg_traffic_light_signal.left = left
        msg_traffic_light_signal.forward = forward
        msg_traffic_light_signal.right = right
        self.transceiver.publish("MsgTrafficLightSignal", msg_traffic_light_signal)
