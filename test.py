import sys
import socket
from struct import *
import zlib
import time
import pandas as pd

def flatten_tuple(nested_tuple):
    flat_tuple = ()
    for element in nested_tuple:
        if isinstance(element, tuple):
            flat_tuple += flatten_tuple(element)
        else:
            flat_tuple += (element,)
    return flat_tuple

class ReqMsgUnpack:
    def __init__(self, msg):
        self.msg_id = 0
        self.request_id = 0
        self.time = 0
        self.time_ns = 0
        self.data_size_bytes = 0
        self.service_msg_id = 0
        self.service_request_id = 0
        self.service_accomplished = 0
        self.service_processing_time_us = 1
        self.msg = msg

    def header_unpack(self, prnt=True):
        unpacked_header = unpack('=2B3I3Bf3x', self.msg[0:24])
        self.msg_id = unpacked_header[0]
        self.request_id = unpacked_header[1]
        self.time = unpacked_header[2]
        self.time_ns = unpacked_header[3]
        self.data_size_bytes = unpacked_header[4]
        self.service_msg_id = unpacked_header[5]
        self.service_request_id = unpacked_header[6]
        self.service_accomplished = unpacked_header[7]
        self.service_processing_time_us = unpacked_header[8]
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y in zip(list(vars(self).keys())[:-1],
                                                             list(vars(self).values())[:-1])))
        else:
            print(unpacked_header[8])

    def unpack(self, prnt=True):
            self.header_unpack(prnt)
            
            data_r = unpack('=iHB', self.msg[44:52])
            header = ['closest_global_waypoint_id', 'current_velocity', 'DBW_enabled:']
            print("\n".join("{} : {}".format(x, y) for x, y in zip(list(header), list(data_r))))
            
            num_waypoints =self.msg[51]
            print("\n num_waypoints", num_waypoints)
            columns = ['wp_id', 'x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
            table = []
            idx = 52
            for _ in range(num_waypoints):
                table.append(list(unpack("ifffffi", self.msg[idx:idx+28])))
                idx = idx+28
            df = pd.DataFrame(table, columns=columns)
            print(df[:5])

            





socket_address = ('192.168.222.1', 1552)


if __name__ == '__main__':
    print('hi')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.bind(socket_address)
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            ReqMsgUnpack(data).unpack()
        except KeyboardInterrupt:
            s.close()
            s.close()
            s.close()