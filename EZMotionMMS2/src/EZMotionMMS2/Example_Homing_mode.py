from .EZMotionMMS2 import *
import time
Motor1= Ezm( "COM24", 1, 115200)
Motor1.Shutdown()
Motor1.Set_Op_Mode("HOMING")
Motor1.Shutdown()
Motor1.Homing_Method(-3)
Motor1.Homing_Torque(200)
Motor1.Update()


Motor1.Enable_Motor()
time.sleep(5)
print("Motor Speed = ", Motor1.Read_Actual_Velocity())
print(Motor1.Read_Op_Mode())
# Motor1.Set_Target_Velocity(1000)
time.sleep(5)

Motor1.Disable_Motor()