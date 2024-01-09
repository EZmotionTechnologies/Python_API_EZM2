from .EZMotionMMS2 import *
import time

#Create motor object with slave address 1 and port COM24
Motor1= Ezm("COM24", 1, 115200)

Motor1.Shutdown()

# Set modes of operation to Torque mode
Motor1.Set_Op_Mode("TORQUE")

#Set target motor torque
Motor1.Set_Target_Torque(-80)

Motor1.Enable_Motor()

#read and print actual motor torque
print(Motor1.Read_Actual_Torque())
time.sleep(15)

#read and print motor actual velocity
print("Motor Speed = ", Motor1.Read_Actual_Velocity())

#read and print actual motor torque
print(Motor1.Read_Actual_Torque())

#print selected modes of operation
print(Motor1.Read_Op_Mode())

Motor1.Disable_Motor()