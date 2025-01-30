import serial
import struct
import time

class MotorBoardMD49:

    # Constants
    GET_SPEED_1 = b'\x21'
    GET_SPEED_2 = b'\x22'
    GET_ENCODER_1 = b'\x23'
    GET_ENCODER_2 = b'\x24'
    GET_ENCODERS = b'\x25'
    GET_VOLTS = b'\x26'
    GET_CURRENT_1 = b'\x27'
    GET_CURRENT_2 = b'\x28'
    GET_VERSION = b'\x29'
    GET_ACCELERATION = b'\x2A'
    GET_MODE = b'\x2B'
    GET_VI = b'\x2C'
    GET_ERROR = b'\x2D'

    SET_SPEED_1 = b'\x31'
    SET_SPEED_2_TURN = b'\x32'
    SET_ACCELERATION = b'\x33'
    SET_MODE = b'\x34'
    RESET_ENCODERS = b'\x35'
    DISABLE_REGULATOR = b'\x36'
    ENABLE_REGULATOR = b'\x37'
    DISABLE_TIMEOUT = b'\x38'
    ENABLE_TIMEOUT = b'\x39'

    def __init__(self, port, baudrate=38400):
        print("MotorBoardMD49 inited")
        self._uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        self._mode = 0

    def _txCmd(self, cmdByte, valueByte=None):
        if cmdByte is not None and len(cmdByte) == 1 and \
                (valueByte is None or len(valueByte) == 1):
            buf = b'\x00' + cmdByte + (valueByte if valueByte else b'')
            for _ in range(100):
                self._uart.write(buf)
                time.sleep(0.01)
                return True
        return False

    def _rxRet(self, retSize):
        ret = b''
        for _ in range(100):
            buf = self._uart.read(retSize - len(ret))
            if buf:
                ret += buf
                if len(ret) == retSize:
                    return ret
            time.sleep(0.01)
        return None

    def GetSpeed1(self):
        if self._txCmd(self.GET_SPEED_1):
            ret = self._rxRet(1)
            if ret:
                fmt = 'B' if self._mode in [0, 2] else 'b'
                return struct.unpack(fmt, ret)[0]
        return None

    def GetSpeed2(self):
        if self._txCmd(self.GET_SPEED_2):
            ret = self._rxRet(1)
            if ret:
                fmt = 'B' if self._mode in [0, 2] else 'b'
                return struct.unpack(fmt, ret)[0]
        return None

    def GetEncoder1(self):
        if self._txCmd(self.GET_ENCODER_1):
            ret = self._rxRet(4)
            if ret:
                return struct.unpack('>i', ret)[0]
        return None

    def GetEncoders(self):
        if self._txCmd(self.GET_ENCODERS):
            ret = self._rxRet(8)
            if ret:
                return struct.unpack('>ii', ret)
        return None

    def GetVolts(self):
        if self._txCmd(self.GET_VOLTS):
            ret = self._rxRet(1)
            if ret:
                return struct.unpack('B', ret)[0]
        return None

    def SetSpeed1(self, value):
        fmt = 'B' if self._mode in [0, 2] else 'b'
        if (self._mode in [0, 2] and 0 <= value <= 255) or (-128 <= value <= 127):
            return self._txCmd(self.SET_SPEED_1, struct.pack(fmt, value))
        return False
    
    def set_speed_2_turn(self, value: int) -> bool:
        if self._mode in [0, 2]:
            if 0 <= value <= 255:
                fmt = 'B'
            else:
                print("Value out of range (0-255) for mode 0 or 2")
                return False
        else:
            if -128 <= value <= 127:
                fmt = 'b'
            else:
                print("Value out of range (-128 to 127) for mode other than 0 or 2")
                return False

        return self._txCmd(MotorBoardMD49.SET_SPEED_2_TURN, struct.pack(fmt, value))

    def set_acceleration(self, value: int) -> bool:
        if 1 <= value <= 10:
            return self._txCmd(MotorBoardMD49.SET_ACCELERATION, struct.pack('B', value))
        
        print("Acceleration value must be between 1 and 10")
        return False

    def SetMode(self, mode):
        if 0 <= mode <= 3:
            if self._txCmd(self.SET_MODE, struct.pack('B', mode)):
                self._mode = mode
                return True
        return False

    def Close(self):
        self._uart.close()