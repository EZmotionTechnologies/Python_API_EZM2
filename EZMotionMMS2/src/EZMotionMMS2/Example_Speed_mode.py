import time
from .EZMotionMMS2 import *
#Create motor object with slave address 1 and port COM24
Motor1= Ezm("COM24", 1, 115200)
Motor1.Shutdown()

# Set modes of operation to Speed mode
Motor1.Set_Op_Mode("SPEED")
Motor1.Shutdown()

Motor1.Set_Acceleration(500)
Motor1.Set_Deceleration(500)

# Set target velocity to 1000rpm in CW direction
Motor1.Set_Target_Velocity(-1000)

Motor1.Enable_Motor()
time.sleep(5)
#read and print motor actual velocity
print("Motor Speed = ", Motor1.Read_Actual_Velocity())

#print selected modes of operation
print(Motor1.Read_Op_Mode())

# Set target velocity to 1000rpm in CCW direction
Motor1.Set_Target_Velocity(1000)

time.sleep(5)
Motor1.Disable_Motor()