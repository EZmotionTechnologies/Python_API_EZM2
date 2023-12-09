from .EZMotionMMS2 import *
import time
Motor1= Ezm( "COM24", 2, 115200)
Motor1._Shutdown()
# Motor2= EZMotionMMS2.EZM("COM24", 2, 115200)
# print("Slave Address for Motor2= ", Motor2._Read_16bit_Reg(0x3050))
Motor1._Set_Op_Mode("HOMING")
Motor1._Shutdown()
Motor1._Homing_Method(-3)
Motor1._Homing_Torque(200)
Motor1._Update()


Motor1._Enable_Motor()
Motor1._Update()
time.sleep(5)
print("Motor Speed = ", Motor1._Read_Actual_Velocity())
print(Motor1._Read_Op_Mode())
# Motor1._Set_Target_Velocity(1000)
time.sleep(5)

Motor1._Disable_Motor()