import sys
import socket
from struct import *
import zlib
import time
import pandas as pd

waypoint_only = False
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
        self.content = unpacked_header[3]
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y in zip(list(vars(self).keys())[:-1],
                                                             list(vars(self).values())[:-1])))
            content = [self.content >> i & 1 for i in range(8)]
            msg_list = ['replies','waypoint', 'status', 'lat/long offset', '', '', '', '']
            print('content:', [msg_list[i] for i in range(8) if content[i]])

        return 19
   
    def replies_unpack(self, cb, prnt=True):
        replies_num = int(self.msg[cb])
        idx = cb + 1
        msg_list = ['id', 'req', 't_rec', 't_rec_ns', 't_rep', 't_rep_ns']
        for _ in range(replies_num):
            unpacked = unpack('=2B4i?', self.msg[idx:idx+19])
            idx += 19
            if prnt:
                print("\n".join("{} : {}".format(x, y) for x, y
                                 in zip(msg_list, unpacked)))
        return cb + 191        
        
    def status_unpack(self, cb, prnt=True):
        unpacked = unpack('=iH?', self.msg[cb:cb+7])
        msg_list = ['closest_wp_id', 'vel', 'db_engaged']
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(msg_list, unpacked)))
        return cb+7
    
    def wp_unpack(self, cb, prnt=True):
        num_waypoints = self.msg[cb]
        columns = ['wp_id', 'x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
        table = []
        idx = cb+1
        for _ in range(num_waypoints):
            table.append(list(unpack("i5fi", self.msg[idx:idx+28])))
            idx = idx+28
        df = pd.DataFrame(table, columns=columns)
        if prnt: 
            print("\n num_waypoints", num_waypoints)
            print(df[:5])
        return cb + 2801
    
    def wp_only_unpack(self, cb, prnt=True):
        unpacked = unpack('=H?', self.msg[cb:cb+3])
        msg_list = ['start_id', 'end_of_data']
        if prnt:
            print("\n".join("{} : {}".format(x, y) for x, y
                                in zip(msg_list, unpacked)))
        columns = ['wp_id', 'velocity', 'z']
        table = []
        idx = cb+3
        for _ in range(1361):
            table.append(list(unpack("H2h", self.msg[idx:idx+6])))
            idx = idx+6
        df = pd.DataFrame(table, columns=columns)
        if prnt: 
            print(df[:5])
        return cb + 8169
        
    def unpack(self, prnt=True):
            cb = self.header_unpack(prnt)
            cb = self.replies_unpack(cb, prnt)
            cb = self.status_unpack(cb, prnt)
            cb = self.wp_unpack(cb, prnt)
    
    def unpack_wpOnly(self, prnt=True):
            cb = self.header_unpack(prnt)
            cb = self.wp_only_unpack(cb, prnt)

            
if __name__ == '__main__':
    socket_address = (ip, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(socket_address)
    print(f'Socket has been opened:{ip}/{port}', ip, port)
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
                data, addr = s.recvfrom(8192)
                ReqMsgUnpack(data).unpack_wpOnly()
            except KeyboardInterrupt:
                s.close()

