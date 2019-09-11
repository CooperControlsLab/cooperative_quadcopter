import socket
import numpy as np
import pickle
import time

FRAMES = [[0,1,0,0,1,0],
          [1,0,0,1,0,0],
          [0,-1,0,0,-1,0],
          [-1,0,0,-1,0,0]]

FRAMES = [[np.sin(x),np.cos(x),0,0,0,0] for x in np.linspace(0,2*np.pi,)]



s = socket.socket()
s.connect(("10.10.10.10",8123))
footer = b'XXX'

for f in FRAMES:
    arr = [np.array(f[:3]), np.array(f[3:])]
    s.sendall(pickle.dumps(arr,protocol=2)+footer)
    time.sleep(1)
    
