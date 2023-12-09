import minimalmodbus
import serial

# Register Address for EZmotion MMS760 and MMS740 Modbus series servo motors


_CONTROL_WORD_REG = 0x6400
_STATUS_WORD_REG = 0x6410
_OPERATION_MODE_REG = 0x6600
_SHOW_OPERATION_MODE_REG = 0x6610

# Profile Torque Control Registers      *************************************
_TARGET_TORQUE_REG = 0x6710
_MAX_TORQUE_REG = 0x6720
_MAX_CURRENT_REG = 0x6730
_TORQUE_DEMAND_REG = 0x6740
_TORQUE_ACTUAL_REG = 0x6770
_CURRENT_ACTUAL_REG = 0x6780
_TORQUE_SLOPE_REG = 0x6870
_TORQUE_PROFILE_REG = 0x6880

# Profile Speed Control Registers       *************************************
_VELOCITY_DEMAND_REG = 0x66B0
_VELOCITY_ACTUAL_REG = 0x66C0
_VELOCITY_WINDOW_REG = 0x66D0
_VELOCITY_WINDOW_TIME_REG = 0x66E0
_VELOCITY_THRESHOLD_REG = 0x66F0
_PROFILE_ACCELERATION_REG = 0x6830
_PROFILE_DECELERATION_REG = 0x6840
_MAX_ACCELERATION_REG = 0x6C50
_MAX_DECELERATION_REG = 0x6C60
_TARGET_VELOCITY_REG = 0x6FF0

# Profile Position Control Registers    *************************************
_POSITION_DEMAND_REG = 0x6620
_POSITION_WINDOW_REG = 0x6670
_POSITION_WINDOW_TIME_REG = 0x6680
_TARGET_POSITION_REG = 0x67A0
_PROFILE_VELOCITY_REG = 0x6810
_MOTION_PROFILE_TYPE_REG = 0x6860
_FOLLOWING_ERROR_ACTUAL_REG = 0x6F40
_POSITION_ACTUAL_REG = 0x6640

# Homing Mode Registers                 **************************************
_HOMING_TORQUE_REG = 0x2700
_HOMING_TIME_REG = 0x2701
_HOME_OFFSET_REG = 0x67C0
_HOMING_METHOD_REG = 0x6980
_HOMING_SPEED_SWITCH_REG = 0x6990
_HOMING_SPEED_ZERO_REG = 0x6991
_HOMING_ACCELERATION_REG = 0x69A0

# Flag Bits
_TARGET_REACHED_BIT = 10
_NEW_SET_POINT_BIT = 4
_ABSOLUTE_RELATIVE_BIT = 6
_FAULT_RESET_BIT = 7


class Ezm:

    def __init__(self, COM_port, slave_add, baudrate: int= 115200, debugger: bool = False):
        '''
        Initialize servo motor object
        :param COM_port: serial COM port, for example "COM24"
        :param slave_add: slave address of the servo motor
        :param baudrate: serial baudrate, default- 115200
        :param debugger: enable/disable minimalmodbus debugger, default- disabled
        '''
        self.Ezm = minimalmodbus.Instrument(COM_port, slave_add, debug=debugger)
        self.Ezm.serial.baudrate = baudrate
        self.Ezm.serial.parity = serial.PARITY_EVEN
        self.Ezm.serial.timeout = 0.2
        self.Ezm.serial.bytesize = 8
        self.Ezm.mode = minimalmodbus.MODE_RTU

    def _Read_32bit_Reg(self, reg_address, sign: bool = False):
        '''
        Read a 32bit register value
        :param reg_address: register address to be read
        :param sign: True for signed and False (default) for unsigned
        :return: register value
        '''
        return self.Ezm.read_long(reg_address, signed=sign, byteorder=3)

    def _Read_16bit_Reg(self, reg_address, sign: bool = False):
        '''
        Read a 16bit register value
        :param reg_address: register address to be read
        :param sign: True for signed and False (default) for unsigned
        :return: register value
        '''
        return self.Ezm.read_register(reg_address, signed=sign)

    def _Write_16bit_Reg(self, reg_address, value, sign: bool = False):
        '''
        Write value to a 16 bit register
        :param reg_address: register address to be written
        :param value: value to be written
        :param sign: True for signed and False (default) for unsigned
        :return: None
        '''
        self.Ezm.write_register(reg_address, value, signed=sign)

    def _Write_32bit_Reg(self, reg_address, value, sign: bool = False):
        '''
        Write value to a 32 bit register
        :param reg_address: register address to be written
        :param value: value to be written
        :param sign: True for signed and False (default) for unsigned
        :return: None
        '''
        self.Ezm.write_long(reg_address, value, signed=sign, byteorder=3)

    def _INC_to_RPM(self, value: int):
        '''
        Takes encoder increments per sec and returns RPM
        :param value: encoder increments
        :return: RPM (revolution per minute)
        '''
        return int((value / 2 ** 16) * 60)

    def _RPM_to_INC(self, value: int):
        '''
        Takes RPM and returns encoder increments per sec
        :param value: RPM (revolution per minute)
        :return: Encoder increments per sec
        '''
        return int((value * 2 ** 16) / 60)

    def _INC_to_ANGLE(self, value: int):
        '''
        Takes encoder increments and returns angle in degrees
        :param value: encoder increments
        :return: Angle in degrees
        '''
        return int((value / 2 ** 16) * 360)

    def _ANGLE_to_INC(self, value: int):
        '''
        Takes angle in degree and returns encoder increments
        :param value: angle in degrees
        :return: encoder increments
        '''
        return int((value / 360) * 2 ** 16)


    # ___________________________________________________________________________________________________________________
    def _Set_Op_Mode(self, mode: str):
        ''' Sets the mode of operation for the servo motor
        :param mode: Uppercase operation name (HOMING, SPEED, TORQUE, POSITION ABS, and POSITION REL)
        :return:None
        '''
        if mode == 'POSITION ABS':
            self._Write_16bit_Reg(_OPERATION_MODE_REG, 0x0001)
            temp_value = (self._Read_Control_Word() & ~ (1 << _ABSOLUTE_RELATIVE_BIT))
            self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)
        elif mode == 'POSITION REL':
            self._Write_16bit_Reg(_OPERATION_MODE_REG, 0x0001)
            temp_value = (self._Read_Control_Word() | (1 << _ABSOLUTE_RELATIVE_BIT))
            self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)
        elif mode == 'SPEED':
            self._Write_16bit_Reg(_OPERATION_MODE_REG, 0x0003)
        elif mode == 'TORQUE':
            self._Write_16bit_Reg(_OPERATION_MODE_REG, 0x0004)
        elif mode == 'HOMING':
            self._Write_16bit_Reg(_OPERATION_MODE_REG, 0x0006)

    # READ and Print OPERATION MODE (Position, Speed, Torque and Homing)            ************************************
    # __________________________________________________________________________________________________________________
    def _Read_Op_Mode(self):
        '''
        Simply returns current mode of operation for the servo motor
        :return: Uppercase operation name (HOMING, SPEED, TORQUE, POSITION ABS, and POSITION REL)
        '''
        temp = self._Read_16bit_Reg(_OPERATION_MODE_REG)
        if temp == 1:
            if (self._Read_Control_Word() & (1 << 6)) == 0:
                return "Operation Mode Selected = 1 (POSITION ABSOLUTE)"
            else:
                return "Operation Mode Selected = 1 (POSITION RELATIVE)"
        elif temp == 3:
            return "Operation Mode Selected= 3 (SPEED)"
        elif temp == 4:
            return "Operation Mode Selected = 4 (TORQUE)"
        elif temp == 6:
            return "Operation Mode Selected = 6 (HOMING)"
        else:
            return "Operation Mode Selected = "

    def _Read_Actual_Velocity(self):
        '''
        Read and return actual velocity (rpm) of the servo motor
        :return: actual velocity (rpm)
        '''
        temp = self._Read_32bit_Reg(_VELOCITY_ACTUAL_REG, True)
        return self._INC_to_RPM(temp)

    def _Set_Profile_Velocity(self, rpm):
        '''
        Sets profile velocity in profile position mode
        :param rpm: velocity in rpm
        :return: None
        '''
        rpm = abs(rpm)
        temp = self._RPM_to_INC(rpm)
        self._Write_32bit_Reg(_PROFILE_VELOCITY_REG, temp, False)
        # print(self._Read_32bit_Reg(_PROFILE_VELOCITY_REG, True))

    def _Set_Acceleration(self, rpm_per_sec):
        '''
        Sets the profile acceleration of the servo motor
        :param rpm_per_sec: rpm/s
        :return: None
        '''
        rpm_per_sec = abs(rpm_per_sec)
        temp_value = self._RPM_to_INC(rpm_per_sec)
        self._Write_32bit_Reg(_PROFILE_ACCELERATION_REG, temp_value, False)

    def _Set_Deceleration(self, rpm_per_sec):
        '''
           Sets the profile deceleration of the servo motor
           :param rpm_per_sec: rpm/s
           :return: None
           '''
        rpm_per_sec = abs(rpm_per_sec)
        temp_value = self._RPM_to_INC(rpm_per_sec)
        self._Write_32bit_Reg(_PROFILE_DECELERATION_REG, temp_value, False)

    def _Set_Target_Velocity(self, rpm: int):
        '''
        Sets target velocity of the servo motor in Speed mode
        :param rpm: speed in rpm
        :return: None
        '''
        temp = self._RPM_to_INC(rpm)
        self._Write_32bit_Reg(_TARGET_VELOCITY_REG, temp, True)

    def _Read_Status_Word(self):
        '''
        Read status word register
        :return: register value
        '''
        temp = self._Read_16bit_Reg(_STATUS_WORD_REG)
        return temp

    def _Read_Control_Word(self):
        '''
        Read control word register
        :return: register value
        '''
        return self._Read_16bit_Reg(_CONTROL_WORD_REG)

    def _Enable_Motor(self):
        '''
        Enable the servo motor
        :return: None
        '''
        temp_value = (self._Read_Control_Word() & 0xFFF0) + 0x000F
        self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)

    def _Disable_Motor(self):
        '''
        Disable the servo motor
        :return: None
        '''
        temp_value = (self._Read_Control_Word() & 0xFFF0) + 0x0006
        self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)

    def _Shutdown(self):
        '''
        This command allows the servo to advance to "Ready to Switch On" state.
        See servo user guide for state machine diagram
        :return: None
        '''
        # temp_value = (self._Read_Control_Word() | (1 << 4))
        self._Write_16bit_Reg(_CONTROL_WORD_REG, 0x0006)

    def _Update(self):
        '''
        Update servo motor with new target parameter for control loops
        :return: None
        '''
        temp_value = (self._Read_Control_Word() & ~ (1 << _NEW_SET_POINT_BIT))
        self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)
        temp_value = (self._Read_Control_Word() | (1 << _NEW_SET_POINT_BIT))
        self._Write_16bit_Reg(_CONTROL_WORD_REG, temp_value)

    def _Set_Target_Position(self, turns: int, angle: int, dir: str) -> object:
        '''
        Sets servo motors target position using revs and angle information
        :param turns: number of full turns to rotate
        :param angle: 0-360 degree angle
        :param dir: CW for clockwise direction, CCW for counter clockwise
        :return: None
        '''
        if dir == "CW":
            turns = -turns
            angle = -angle

        temp_value = (turns << 16) + self._ANGLE_to_INC(angle)
        # print("temp_value = ", hex(temp_value))
        self._Write_32bit_Reg(_TARGET_POSITION_REG, temp_value, True)

    def _Pos_Target_Check_Flag(self):
        '''
        Checks if servo has reached its target
        :return: True if target reached, otherwise returns False
        '''
        if (self._Read_Status_Word() & (1 << _TARGET_REACHED_BIT)) == (1 << _TARGET_REACHED_BIT):
            return True
        else:
            return False

    def _Read_Actual_Position(self):
        '''
        Read actual position of the servo motor
        :return: number of full turns in absolute + angle between 0-360
        '''
        temp_value = self._Read_32bit_Reg(_POSITION_ACTUAL_REG, True)
        # print(hex(temp_value))
        turns = ((temp_value & 0xFFFF0000) >> 16)
        angle = self._INC_to_ANGLE(temp_value & 0x0000FFFF)
        # print(hex(turns),"  =  ", hex(angle))
        return turns, angle

    def _Set_Target_Torque(self, target: int, limit: bool = True):
        '''
        Sets target torque in thousandth
        Will throw a warning message if target is set to more than 100%
        :param target: target torque in thousandth unit (110 = 10.1% of servo rated torque)
        :param limit: if True then limits the settable torque to 100% of the motor's rated torque
        :return: None
        '''
        if limit == True:
            if 1000 >= target >= -1000:
                self._Write_16bit_Reg(_TARGET_TORQUE_REG, target, True)
            else:
                print("     WARNING: Target torque > Motor's rated torque.................................."
                      "\n       ....Are you sure? If yes, then pass (limit= False) to _Set_Target_Torque method")
                exit(1)
        else:
            self._Write_16bit_Reg(_TARGET_TORQUE_REG, target)

    def _Read_Actual_Torque(self):
        '''
        Read actual servo motor torque in thousandth unit
        :return: torque between (-3000 to 3000)
        '''
        return self._Read_16bit_Reg(_TORQUE_ACTUAL_REG, True)

    def _Set_Torque_Slope(self, slope: int):
        '''
        Sets the slope of the torque in torque mode
        :param slope: slope of the torque
        :return:None
        '''
        slope = abs(slope)
        self._Write_32bit_Reg(_TORQUE_SLOPE_REG, slope)

    def _Homing_Method(self, method: int):
        '''
        Sets the homing method type
        :param method: signed integer (see user guide)
        :return: None
        '''
        self._Write_16bit_Reg(_HOMING_METHOD_REG, method, True)

    def _Homing_Torque(self, torque: int):
        '''
        Sets torque limit when homing with torque limit is implemented
        :param torque: torque in thousandth unit
        :return: None
        '''
        torque = abs(torque)  # in thousandth ( 550 = 0.55% of motor's rated torque)
        self._Write_16bit_Reg(_HOMING_TORQUE_REG, torque)


    def _Homing_Switch_Speed(self, rpm):
        '''
        homing switch search speed to search for the homing switch
        :param rpm: speed in rpm
        :return: None
        '''
        rpm = abs(rpm)
        temp = self._RPM_to_INC(rpm)
        self._Write_32bit_Reg(_HOMING_SPEED_SWITCH_REG, temp)


    def _Homing_Zero_Speed(self, rpm):
        '''
        homing zero search speed to search for the actual homing position
        :param rpm: speed in rpm
        :return: None
        '''
        rpm = abs(rpm)
        temp = self._RPM_to_INC(rpm)
        self._Write_32bit_Reg(_HOMING_SPEED_ZERO_REG, temp)

    def _Homing_Acceleration(self, rpm_per_sec):
        '''
        acceleration in homing mode
        :param rpm_per_sec: acceleration in rpm/s
        :return: None
        '''
        rpm_per_sec = abs(rpm_per_sec)
        temp_value = self._RPM_to_INC(rpm_per_sec)
        self._Write_32bit_Reg(_HOMING_ACCELERATION_REG, temp_value, False)


