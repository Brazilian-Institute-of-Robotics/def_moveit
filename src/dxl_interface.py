#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Interface for trajectory following for dynamixel pro

import os, sys, time, rospy
import tty, termios
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
from control_msgs.msg import FollowJointTrajectoryActionGoal

# Control table address
ADDR_PRO_OP_MODE             = 11  # 1 byte
ADDR_PRO_TORQUE_ENABLE       = 562 # 1 byte                         # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 596 # 4 byte 
ADDR_PRO_PRESENT_POSITION    = 611 # 4 byte
ADDR_PRO_GOAL_VELOCITY       = 600 # 4 byte
ADDR_PRO_GOAL_TORQUE         = 604 # 2 byte

# Firmware 2.0, check control table
# http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
ADDR_106_OP_MODE             = 11   # 1 byte
ADDR_106_MAXPOS_LIMIT        = 48   # 4 byte
ADDR_106_MINPOS_LIMIT        = 52   # 4 byte
ADDR_106_TORQUE_ENABLE       = 64   # 1 byte
ADDR_106_GOAL_POSITION       = 116  # 4 byte 
ADDR_106_PRESENT_POSITION    = 132  # 4 byte
ADDR_106_GOAL_VELOCITY       = 104  # 4 byte
ADDR_106_VEL_LIMIT           = 44   # 4 byte

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

DXL_PRO_ID                  = 2
DXL_106_ID                  = 1
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')

PRO_OPMODE_VEL = 1
PRO_OPMODE_POS = 3
TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_PRO_MINIMUM_POSITION_VALUE  = -150000                       # Dynamixel will rotate between this value
DXL_PRO_MAXIMUM_POSITION_VALUE  = 150000                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_PRO_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)



index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result

dxl_error = 0                                               # Dynamixel error
dxl_present_position = 0                                    # Present position


def init():
    # Initialize PacketHandler Structs
    dynamixel.packetHandler()
    # Open port
    if dynamixel.openPort(port_num):
        print("Succeeded to open the port!")
    else:
        print("Failed to open the port!")
        quit()

    # Set port baudrate
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeeded to change the baudrate!")
    else:
        print("Failed to change the baudrate!")
        quit()


def initDxl106():
    # set opmode to velocity control
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_OP_MODE, PRO_OPMODE_VEL)
    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_MAXPOS_LIMIT, 4090)
    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_MINPOS_LIMIT, 10)
    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_VEL_LIMIT, 500)
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_ENABLE)

def initDxlPro():
        # set opmode to velocity control
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_OP_MODE, PRO_OPMODE_VEL)

    # Enable Dynamixel Torque
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    resulttx = dynamixel.getTxRxResult(port_num, PROTOCOL_VERSION)
    if resulttx != COMM_SUCCESS:
        pass
    elif dynamixel.getRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getRxPacketError(port_num, PROTOCOL_VERSION))
    else:
        print("Dynamixel has been successfully connected")



# Set new Trajectory as goal
def setNewTrajectory(trajMsg):
    pass


def control():

    actual_vel = 0
    upgoing = True
    rospy.init_node('dynamixelpronode', anonymous=True)
    #rospy.Subscriber("/dyn_ef_robot/dyn_ef_robot_controller/follow_joint_trajectory/goal", String, setNewTrajectory)
    rate = rospy.Rate(20) # 10hz

    while not rospy.is_shutdown():
        # print("Press any key to continue! (or press ESC to quit!)")
        # if getch() == chr(ESC_ASCII_VALUE):
        #     break
        # print("hallo")

        if upgoing:
            actual_vel += 100
            if actual_vel >= 4000:
                upgoing = False
        else:
            actual_vel -= 100
            if actual_vel <= -4000:
                upgoing = True

        # Write goal position
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_GOAL_VELOCITY, actual_vel)
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_GOAL_VELOCITY, actual_vel/16)
        # if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        #     print("not successful")
        #     dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        # elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        #     dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

        # while 1:
        #     # Read present position
        #     print("read present position")
        #     dxl_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        #     if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        #         dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        #     elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        #         dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

        #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

        #     if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
        #         break

        # Change goal position
        # if index == 0:
        #     index = 1
        # else:
        #     index = 0
        
        rate.sleep()

    # Disable Dynamixel Torque
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_DISABLE)




# Close port
dynamixel.closePort(port_num)

if __name__ == '__main__':
    try:
        init()
        initDxl106()
        initDxlPro()
        control()
    except rospy.ROSInterruptException:
        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
