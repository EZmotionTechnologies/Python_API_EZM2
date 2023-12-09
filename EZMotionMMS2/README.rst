
EZMotionMMS2
========================

Python wrapper built around minimalmodbus for controlling EZMotion MMP/ MMS 740 and 760 series servo motors and driver modules via serial-to-RS485 converter


Example: Position Mode (Single motor)
--------------
::

    from .EZMotionMMS2 import *
    Motor1= Ezm("COM24", 2, 115200)
    Motor1._Shutdown()                  # Shutdown and Enable the motor

    # Set modes of operation to Relative Position mode
    Motor1._Set_Op_Mode("POSITION REL")

    #print selected modes of operation
    print(Motor1._Read_Op_Mode())

    Motor1._Enable_Motor()

    #send new target position (10 turns + 180 degrees in CCW direction) to the motor and update
    Motor1._Set_Target_Position(10,180,"CCW")
    Motor1._Update()

    # wait for the motor to reach target position
    while (Motor1._Pos_Target_Check_Flag() == False):
        pass

    # Read and print motors current position
    turns,angle = Motor1._Read_Actual_Position()
    print("Turns = ", turns)
    print("Angle = ", angle)

    #send new target position to the motor and update
    Motor1._Set_Target_Position(5,180,"CW")
    Motor1._Update()

    # wait for the motor to reach target position
    while (Motor1._Pos_Target_Check_Flag() == False):
        pass

    Motor1._Disable_Motor()


Example: Speed Mode (Single motor)
--------------
::

    import time
    from .EZMotionMMS2 import *
    Motor1= Ezm("COM24", 2, 115200)
    Motor1._Shutdown()
    Motor1._Set_Op_Mode("SPEED")
    Motor1._Shutdown()
    Motor1._Set_Acceleration(500)
    Motor1._Set_Deceleration(500)
    Motor1._Set_Target_Velocity(-1000)
    Motor1._Enable_Motor()
    time.sleep(5)
    print("Motor Speed = ", Motor1._Read_Actual_Velocity())
    print(Motor1._Read_Op_Mode())
    Motor1._Set_Target_Velocity(1000)
    time.sleep(5)
    Motor1._Disable_Motor()