import sys
import socket
from struct import *
import zlib
import time
import pandas as pd

waypoint_only = True
ip = '127.0.0.1'
port = 1552
commands = [arg for arg in sys.argv[1:] if "-" in arg]
for arg in commands:
    if not arg in ['--wpOnly', '-wp', '--ip', '--port']:
        print("bad argument")
        exit(1)
for idx, arg in enumerate(sys.argv):
    if arg in ['--wpOnly', '-wp']:
        waypoint_only = True
        del sys.argv[idx]

for idx, arg in enumerate(sys.argv):
    if arg in ['--ip']:
        ip = str(sys.argv[idx+1])
        del sys.argv[idx]
        del sys.argv[idx]
for idx, arg in enumerate(sys.argv):
    if arg in ['--port']:
        port = int(sys.argv[idx+1])
        del sys.argv[idx]
        del sys.argv[idx]

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
        self.content = []
        self.msg = msg

    def header_unpack(self, prnt=True):
        unpacked_header = unpack('=B2I10B', self.msg[0:19])
        self.msg_id = unpacked_header[0]
        self.time = unpacked_header[1]
        self.time_ns = unpacked_header[2]
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y in zip(list(vars(self).keys())[:-1],
                                                             list(vars(self).values())[:-1])))
        return 19
   
    def replies_unpack(self, cb, prnt=True):
        msg_list = ['id', 'req', 't_rec', 't_rec_ns', 't_rep', 't_rep_ns']
        unpacked = unpack('=2B4i?', self.msg[cb:cb+19])
        idx += 19
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(msg_list, unpacked)))
        return cb + 19        
        
    def status_unpack(self, cb, prnt=True):
        unpacked = unpack('=b7d?', self.msg[cb:cb+57])
        msg_list = ['detected', 'glat', 'glng', 'X', 'Y', 'rlng', 'V', 'V_r']
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(msg_list, unpacked)))
        return cb+57
    
    def wp_unpack(self, cb, prnt=True):
        unpacked = unpack('=iBf?', self.msg[cb:cb+9])
        num_waypoints = unpacked[1]
        msg_list = ['closest', '# of wp', 'score']
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(msg_list, unpacked)))

        columns = ['x', 'y', 'z', 'yaw', 'velocity']
        table = []
        idx = cb+1
        for _ in range(num_waypoints):
            table.append(list(unpack("5f", self.msg[idx:idx+20])))
            idx = idx+20
        df = pd.DataFrame(table, columns=columns)
        if prnt: 
            print(df[:5])
        return cb + 2009
    
    def wp_only_unpack(self, prnt=True):
        idx = 19
        columns = ['route_id', 'total', 'nwp', 'length', 'duration', 'base_duration']
        unpacked = unpack("2B4I", self.msg[idx:idx+20])
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(columns, unpacked)))
        columns = ['lat', 'lng', 'elv', 'speed', 'base speed', 'action']
        table = []
        idx = 40
        for _ in range(2665):
            table.append(list(unpack("6f", self.msg[idx:idx+24])))
            idx = idx+24
        df = pd.DataFrame(table, columns=columns)
        if prnt: 
            print(df)
            # print(df.to_string())
        
    def unpack(self, prnt=True):
            cb = self.header_unpack(prnt)
            cb = self.replies_unpack(cb, prnt)
            cb = self.wp_unpack(cb, prnt)
            cb = self.status_unpack(cb, prnt)
    
    def unpack_wpOnly(self, prnt=True):
            self.header_unpack(prnt)
            self.wp_only_unpack(prnt)

            
if __name__ == '__main__':
    socket_address = (ip, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(socket_address)
    print(f'Socket has been opened:{ip}/{port}')
    if not waypoint_only:
        while True:
            try:
                data, addr = s.recvfrom(4096)
                ReqMsgUnpack(data).unpack()
            except KeyboardInterrupt:
                s.close()
    else:
        while True:
            try:
                data, addr = s.recvfrom(64000)
                ReqMsgUnpack(data).unpack_wpOnly()
            except KeyboardInterrupt:
                s.close()

