import time
import socket
import pickle
import numpy as np
from pyvicon import pyvicon as pv


IP = "199.98.21.246"
STREAM_MODE = pv.StreamMode.ClientPull #ServerPush is an alternative (lower latency)

# instantiate client
client = pv.PyVicon()

# try to connect to Vicon running on IP
print("Connecting to DataStream",end="")
while not client.is_connected():
    client.connect(IP)
    print(".")
    time.sleep(.25)
print("Connected to DataStream on IP:",IP)

# enable data types
client.enable_segment_data()
client.enable_marker_data()
client.enable_unlabeled_marker_data()
client.enable_device_data()

# set stream mode
print("Setting StreamMode to",str(STREAM_MODE))
client.set_stream_mode(STREAM_MODE)

# set axis mapping for z in upwards direction
client.set_axis_mapping(pv.Direction.Forward,
                        pv.Direction.Left,
                        pv.Direction.Up)
print("Axis Mapping:\n\tX - %s\n\tY - %s\n\tZ - %s" % client.get_axis_mapping())


# start numpy socket to pi
s = socket.socket()
s.connect(("localhost",8123))
footer = b'XXX'

# Since the STREAM_MODE is ClientPull, this client has to initiate all data transfer.
# We will get "continuous" data by using a while loop to constantly pull data from the Vicon stream.

counter = 0
while True:
    print("",end="")
    while client.get_frame() != pv.Result.Success:
        print(".")

    r_person =  client.get_segment_global_rotation_matrix("person","person")
    r_uparm = client.get_segment_global_rotation_matrix("uparm","uparm")
    r_arm = client.get_segment_global_rotation_matrix("arm","arm")
    if any([type(x) == type(None) for x in [r_person,r_uparm,r_arm]]):
        continue
    seg1 = -1*np.matmul(r_uparm.T,r_person)[:,0]
    seg2 = -1*np.matmul(r_arm.T,r_person)[:,0]
    seg1[0] = -1*seg1[0]
    seg2[0] = -1*seg2[0]
    s.sendall(pickle.dumps([seg1,seg2],protocol=2)+footer)
    time.sleep(3)
    counter += 1
