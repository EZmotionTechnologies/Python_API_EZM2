from .EZMotionMMS2 import *
import time
Motor1= Ezm("COM24", 2, 115200)
Motor1._Shutdown()
# Motor2= EZMotionMMS2.EZM("COM24", 2, 115200)
# print("Slave Address for Motor2= ", Motor2._Read_16bit_Reg(0x3050))
Motor1._Set_Op_Mode("TORQUE")
Motor1._Set_Target_Torque(-80)
# Motor1._Set_Acceleration(500)
# Motor1._Set_Deceleration(500)
# Motor1._Set_Target_Velocity(-1000)
Motor1._Enable_Motor()
print(Motor1._Read_Actual_Torque())
time.sleep(15)
print("Motor Speed = ", Motor1._Read_Actual_Velocity())
print(Motor1._Read_Actual_Torque())
print(Motor1._Read_Op_Mode())

# time.sleep(5)
#
Motor1._Disable_Motor()