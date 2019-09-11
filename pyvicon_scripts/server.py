import cPickle as pickle
import socket
import numpy as np
from threading import Thread
from Queue import Queue
from niryo_one_python_api.niryo_one_api import *
import rospy
rospy.init_node('niryo_one_example_python_api')
import sys
#import math

###############
### Robot func
angle1 = 0
angle2 = 0
angle3 = 0
angle4 = np.pi/2
angle5 = 0
angle6 = 0

n = NiryoOne()
n.calibrate_auto()

def thread_move_joints(vectors):
    if vectors is None:
        return
    #angle 1
    angle1 = np.arccos(np.dot(np.array([vectors[0][0], vectors[0][1], 0]), np.array([0,1,0]))/np.linalg.norm(np.array([vectors[0][0], vectors[0][1], 0])))
    if vectors[0][0] < 0:
        angle1 = max(-3.053, angle1*-1)
    else:
        angle1 = min(3.053, angle1)

    #alt1
    #angle1 = atan2(vectors[0][1],vectors[0][0])
    #angle 2
    angle2 = 0

    #angle 3
    angle3 = np.arccos(np.dot(vectors[0], np.array([0,0,-1]))/np.linalg.norm(vectors[0])) - np.pi/2
    if angle3 < 0:
        angle3 = max(-1.396, angle3)
    else:
        angle3 = min(1.57, angle3)
    #alt3
    #angle3 = atan2(vectors[0][2]/(np.linalg.norm(np.array([vectors[0][0],vectors[0][1],0])))
    
    #angle 4
    angle4 = np.arccos(np.dot(np.cross(np.array([1,0,0]), vectors[0]), (vectors[1] - np.dot(vectors[0], vectors[1])/np.linalg.norm(vectors[0])**2*vectors[0]))/np.linalg.norm(np.cross(np.array([1,0,0]), vectors[0]))/np.linalg.norm(vectors[1] - np.dot(vectors[0], vectors[1])/np.linalg.norm(vectors[0])**2*vectors[0]))
    if (vectors[1] - np.dot(vectors[0], vectors[1])/np.linalg.norm(vectors[0])**2*vectors[0])[0] < 0:
        angle4 = max(-3.053, -1*angle4)
    else:
        angle4 = min(3.053, angle4)

    #angle 5
    angle5 = np.arccos(np.dot(vectors[0], vectors[1])/np.linalg.norm(vectors[0])/np.linalg.norm(vectors[1]))
    if (vectors[1] - np.dot(vectors[0], vectors[1])/np.linalg.norm(vectors[0])**2*vectors[0])[0] < 0:
        angle5 = max(-1.744, -angle5)
    else:
        angle5 = min(1.919, angle5)
    #alt5
    #angle5 = asin(np.linalg.norm(np.cross(vectors[0,:],vectors[1,:]))/np.linalg.norm(vectors[0,:])/np.linalg.norm(vectors[1,:]))
    #angle 6
    angle6 = 0

    n.move_joints(np.array([angle1, angle2, angle3, angle4, angle5, angle6]))
    #q.task_done()
###############
##### socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((b'',8123))
def thread_socket():
    global s
    s.listen(1)
    data = b''
    footer = b'XXX'
    while True:
        try:
            c,a = s.accept()
        except AttributeError:

            return
        while True:
            block = c.recv(4096)
            if not block: break
            data += block
            s = data.split(footer,1)
            if len(s)==2:
                out,data = s
                arr = pickle.loads(out)
                q.put(arr)
                c.close()

q = Queue()
worker = Thread(target=thread_socket)
worker.setDaemon(True)
worker.start()
while True:
    try:
        arr = q.get(timeout=10)
    except Empty:
        s.close()
        sys.exit()
    print arr
    try:
        thread_move_joints(arr)
    except NiryoOneException:
        pass
    

    
