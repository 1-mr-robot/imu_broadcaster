#!/usr/bin/env python

import roslib; roslib.load_manifest('imu_broadcaster')
import math
import rospy
import serial
import string
import numpy as np
from geometry_msgs.msg import Quaternion
import tf

# Buffer length
addresses = ['C0:82:26:31:22:48','C0:82:38:31:5E:48','C0:82:1D:30:25:4D']
add_len = 6
N_devices = len(addresses)
count_imu = 1
# x=1:500000000;
# N_samples = length(x);
# R10 = eye(3,3);
# R20 = eye(3,3);
# R30 = eye(3,3);
buffLen = 34
pckLen = 27
e = 0
ePrev = 0
quat1 = [0]*4
quat2 = [0]*4
quat_total = []
quat = [0]*4
for i in range(0, N_devices):
    quat_total.append(quat)
        

def parse_data(data):
    global quat_total
    showing = 0
    showing_index = 0
    RxBuff = [0]*buffLen
    RxBuff_prev = []
    ij_prev = 0
    counter = 1
    
    # if(cval==1)
    #     count_imu = 1;
    # end
    for i in range(0, buffLen):
        # print ord(data[i])
        # print hex(ord(data[i]))
        # RxBuff[i] = ord(data[i])
        RxBuff[i] = (data[i])
        # if(RxBuff[i]==0xC0):
        #     showing = 1
        # if(showing == 1):
        #     print(hex(RxBuff[i]))
        #     showing_index = showing_index +1
        #     if(showing_index == 6):
        #         showing = 0
    # # RxBuff = data
    ij = 0
    if(ij_prev>(buffLen-pckLen)):
        RxBuff = RxBuff_prev[ij_prev:len(RxBuff_prev)].extend(RxBuff)
    buffLen_new = len(RxBuff)
    
    while ij <= (buffLen_new-pckLen):
        found_device = 0
        address = ''
        for i in range(0, add_len):
            address = address  + "{:02X}".format(RxBuff[ij+add_len-1-i])
            if(i<add_len-1):
                address = address +':'
        # address = str(hex(RxBuff[ij+5]))+':'+str(hex(RxBuff[ij+4]))+':'+str(hex(RxBuff[ij+3]))+':'+str(hex(RxBuff[ij+2]))+':'+str(hex(RxBuff[ij+1]))+':'+str(hex(RxBuff[ij]))
        isMIMU = RxBuff[ij+6]
        for j in range(0,N_devices):
            if(address==addresses[j]):
                found_device = 1
                dev_num = j
        quat = []
        if(found_device == 1):
            if(isMIMU == 1):
                print(dev_num)
                print("FOUND")
                timestamp = np.array([RxBuff[ij+7], RxBuff[ij+8], RxBuff[ij+9], RxBuff[ij+10]], dtype=np.int8)
                timestamp.dtype = np.uint32
                timestamp = float(timestamp)
                # print(timestamp)
                time16bit = np.array([RxBuff[ij+7], RxBuff[ij+8]], dtype=np.int8)
                time16bit.dtype = np.uint16
                time16bit = float(time16bit)
                # print(time16bit)
                for ind_quat in range(0, 4):
                    quat_comp = np.array([RxBuff[ij+11+4*ind_quat], RxBuff[ij+12+4*ind_quat], RxBuff[ij+13+4*ind_quat], RxBuff[ij+14+4*ind_quat]], dtype=np.int8)
                    quat_comp.dtype = np.int32
                    quat_comp = float(quat_comp)/1000000.0
                    quat.append(quat_comp)
                # print(quat)
                if dev_num == 0:
                    quat_total[0] = quat
                elif dev_num == 1:
                    quat_total[1] = quat
                elif dev_num == 2:
                    quat_total[2] = quat
            ij = ij + pckLen
            
        elif(found_device == 0):
            ij = ij + 1

    ij_prev = ij
    RxBuff_prev = RxBuff

    counter = counter+1
     
    ePrev = e
    print(quat_total)

class read_wrist(object):
    def __init__(self):
        rospy.init_node('read_serial')
        # Initialize the serial class
        ser = serial.Serial()
        # Define the serial port
        ser.port = "/dev/ttyACM0"
        # Set the baudrate
        ser.baudrate = 115200
        # Set the timeout limit
        ser.timeout = 1
        # Open the serial
        ser.open()

        quat_upper_pub = rospy.Publisher('upperarm_quaternion', Quaternion, queue_size = 10)
        quat_should_pub = rospy.Publisher('shoulder_quaternion', Quaternion, queue_size = 10)
        quat_fore_pub = rospy.Publisher('forearm_quaternion', Quaternion, queue_size = 10)
        
        # Initialize the flag used to check the availability of the received serial data
        flag_sent = 0
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            try:
                # Read data from the Serial
                data = ser.read(buffLen)
                # data = ser.readline()
                # print "len:"
                # print len(data)
                #print(data[2])
                # print(len(data))
                if(len(data)>=buffLen):
                    parse_data(data)
                
                    quat_sent = Quaternion()
                    quat_sent.w = quat_total[0][0]
                    quat_sent.x = quat_total[0][1]
                    quat_sent.y = quat_total[0][2]
                    quat_sent.z = quat_total[0][3]
                    quat_fore_pub.publish(quat_sent)
                    quat_sent = Quaternion()
                    quat_sent.w = quat_total[1][0]
                    quat_sent.x = quat_total[1][1]
                    quat_sent.y = quat_total[1][2]
                    quat_sent.z = quat_total[1][3]
                    quat_should_pub.publish(quat_sent)
                    quat_sent = Quaternion()
                    quat_sent.w = quat_total[2][0]
                    quat_sent.x = quat_total[2][1]
                    quat_sent.y = quat_total[2][2]
                    quat_sent.z = quat_total[2][3]
                    quat_upper_pub.publish(quat_sent)
                # # Split the serial data into several words separated by space
                # words = data.split()
                # # Get the length of the data array
                # word_len = len(words)
                # rate.sleep()
            except KeyboardInterrupt:
                ser.close()
                print("Bye")

if __name__ == '__main__':
    try:
        read_wrist()
    except rospy.ROSInterruptException: pass