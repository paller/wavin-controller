from datetime import datetime
from enum import IntEnum
from typing import List

import serial


class _WavinAHC9000Modbus:
    """Docs
    Communication setup:
        Baud rate:  38.400 bps
        Data bits:  8
        Start bits: 1
        Stop bits:  1
        Parity:     None
    Proprietary Modbus Commands:
        Read:  0x43
        Write: 0x44
    """
    _crc_length = 2
    _header_length = 3

    # TODO: May or may not be used in the future.
    class ResponseErrors(IntEnum):
        read_register_from_address = 0xC1
        write_register_to_address = 0xC2
        read_register_from_index = 0xC3
        write_register_to_index = 0xC4

    class Commands(IntEnum):
        read = 0x43
        write = 0x44

    def __init__(self, tty: str, id: int):
        self._id = id

        try:
            self._dialout = serial.Serial(port=tty,
                                          baudrate=38400,
                                          bytesize=serial.EIGHTBITS,
                                          parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE,
                                          timeout=5,
                                          write_timeout=5)

        except serial.SerialException as e:
            print('Could not open: {}'.format(tty))
            raise e

    def read_register(self, category: int, index: int, page: int, length: int):
        """TODO: Describe inputs """

        # Sanitize just the most fundamental.
        if length < 1:
            # TODO: Make an explosion
            return None

        modbus_cmd = bytes([self._id, self.Commands.read, category, index, page, length])
        modbus_cmd += self._crc(modbus_cmd)

        # Request registers.
        self._dialout.reset_input_buffer()
        self._dialout.reset_output_buffer()
        self._dialout.write(modbus_cmd)
        self._dialout.flush()

        # Read register response.
        modbus_response = self._dialout.read(self._header_length + 2 * length + self._crc_length)

        # Handle some basic failures.
        # First check if we have enough data for an error-response.
        if len(modbus_response) < 3:
            # TODO: Handle absurd case
            return 1
        # If an error-code was returned the command-field will not match.
        if modbus_response[1] != self.Commands.read:
            # TODO: An error occurred. Its error-code can be read from modbus_response[2]
            return 2
        # Make sure we receive the requested amount of data. Nothing more, nothing less.
        if (len(modbus_response) - self._header_length - self._crc_length) != 2 * length:
            # TODO: The response does not match our expectations
            return 3

        # Strip away the header and CRC and convert into an array of registers.
        registers = self._unpack(modbus_response[self._header_length:-self._crc_length])
        return registers

    def write_register(self, category: int, index: int, page: int, data: List[int]):
        """TODO: Describe inputs """

        # Number of registers to write.
        num_registers = len(data)

        # Convert the 16 bit registers into a byte stream used as payload in the Modbus write-command.
        payload = self._pack(data)

        # Construct the entire Modbus command including CRC.
        modbus_cmd = bytes([self._id, self.Commands.write, category, index, page, num_registers]) + payload
        modbus_cmd += self._crc(modbus_cmd)

        # Request registers.
        self._dialout.reset_input_buffer()
        self._dialout.reset_output_buffer()
        self._dialout.write(modbus_cmd)
        self._dialout.flush()

        # Read register response.
        status = self._dialout.read(self._header_length + num_registers + self._crc_length)

        # TODO: Handle errors on write
        return status

    @classmethod
    def _pack(cls, registers: List[int]) -> bytes:
        """Convert a register array into a byte array.

        Convert each of the 16 bit registers into two bytes in the correct sequence for Modbus and
        return it as a bytes().
        """
        length = len(registers)

        # Convert the 16 bit registers into a byte stream.
        stream = bytearray(2 * length)
        index = 0
        for register in registers:
            stream[index] = (register >> 8) & 0xFF
            index += 1
            stream[index] = register & 0xFF
            index += 1

        return bytes(stream)

    @classmethod
    def _unpack(cls, stream: bytes) -> List[int]:
        """Convert a Modbus byte stream into an array of 16 bit registers.

        Convert two bytes from a Modbus sequence into a single 16 bit register value and return all
        the bytes converted as a single array of registers.
        """
        length = int(len(stream) / 2)

        # Convert the byte stream into an array of 16 bit registers.
        registers = [0] * length
        for index in range(length):
            registers[index] = (stream[2 * index] << 8) | stream[2 * index + 1]

        return registers

    @classmethod
    def _crc(cls, data: iter) -> bytearray:
        """Calculate the Modbus CRC value for data.

        While the input data can be any iterator it is advised to use bytes() or bytearray()
        as it guarantees that all data is within range of a single byte.

        Returns the 16 bit CRC value in 2 bytes such that it is readily applied to the input data or compared
        with the received data for integrity checks.

        This is heavily inspired by github.com/cristianav/PyCRC
        """
        crc_table = [
            0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780, 0xc741, 0x0500,
            0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00, 0xcac1,
            0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81,
            0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540,
            0xd701, 0x17c0, 0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001,
            0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0,
            0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0, 0x3b80,
            0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
            0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700,
            0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0,
            0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480,
            0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01, 0x6ac0, 0x6b80, 0xab41,
            0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41, 0xbe01,
            0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1,
            0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181,
            0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
            0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901,
            0x59c0, 0x5880, 0x9841, 0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1,
            0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680,
            0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040]
        crc_16bit = 0xFFFF

        for elm in data:
            tmp = crc_16bit ^ elm
            rotated = crc_16bit >> 8
            crc_16bit = rotated ^ crc_table[tmp & 0x00FF]

        return bytearray([crc_16bit & 0xFF, crc_16bit >> 8])


class _ElementsCategory:

    _category = 0x01
    _pages = 48

    _invalid_value = 0x7FFF

    @property
    def not_a_number(self):
        return float('nan')

    def __init__(self, modbus: _WavinAHC9000Modbus):
        self.__modbus = modbus

    @classmethod
    def _valid_address(cls, address: int) -> bool:
        return address > 0

    def get_indexes(self):
        valid_indexes = list()

        for id in range(self._pages):
            if self._valid_address(self.address(id)):
                valid_indexes.append(id)

        return valid_indexes

    def address_low(self, channel: int) -> int:
        return self.__modbus.read_register(self._category, 0, channel, 1)[0]

    def address_high(self, channel: int) -> int:
        return self.__modbus.read_register(self._category, 1, channel, 1)[0]

    def address(self, channel: int) -> int:
        address_part = self.__modbus.read_register(self._category, 0, channel, 2)
        address = (address_part[1] << 16) | address_part[0]

        return address

    @classmethod
    def _valid_value(cls, value: int):
        return value < cls._invalid_value

    def _read_and_sanitize_value(self, channel: int, index: int):
        value = self.__modbus.read_register(self._category, index, channel, 1)[0]

        if self._valid_value(value):
            return value
        else:
            return self.not_a_number

    def temp_air(self, channel: int) -> float:
        return self._read_and_sanitize_value(channel, 4) / 10

    def temp_floor(self, channel: int) -> float:
        return self._read_and_sanitize_value(channel, 5) / 10

    def temp_dew(self, channel: int) -> float:
        return self._read_and_sanitize_value(channel, 6) / 10

    def humidity(self, channel: int):
        return self.__modbus.read_register(self._category, 7, channel, 1)[0]

    def status(self, channel: int):
        return self.__modbus.read_register(self._category, 8, channel, 1)[0]

    def _rssi(self, channel: int):
        return self.__modbus.read_register(self._category, 9, channel, 1)[0]

    def rssi_base(self, channel: int):
        return -74.0 + 0.5 * (self._rssi(channel) & 0xFF)

    def rssi_remote(self, channel: int):
        return -74.0 + 0.5 * (self._rssi(channel) >> 8)

    def battery(self, channel: int):
        return (self.__modbus.read_register(self._category, 10, channel, 1)[0] & 0x0F) * 10

    def sync_group(self, channel: int):
        return self.__modbus.read_register(self._category, 11, channel, 1)[0] & 0xFF


class _ClockCategory:

    _category = 5

    def __init__(self, modbus: _WavinAHC9000Modbus):
        self.__modbus = modbus

    def get(self) -> datetime:
        registers = self.__modbus.read_register(self._category, 0, 0, 7)

        year = registers[0]
        month = registers[1]
        day = registers[2]
        week = registers[3]
        hour = registers[4]
        minute = registers[5]
        second = registers[6]

        return datetime(year, month, day, hour, minute, second)

    def set(self, date: datetime):
        registers = [0] * 7

        registers[0] = date.year
        registers[1] = date.month
        registers[2] = date.day
        registers[3] = date.weekday()
        registers[4] = date.hour
        registers[5] = date.minute
        registers[6] = date.second

        self.__modbus.write_register(self._category, 0, 0, registers)


class WavinControl:

    def __init__(self, tty: str, id: int = 0x1):
        self._modbus = _WavinAHC9000Modbus(tty, id)

        self.clock = _ClockCategory(self._modbus)
        self.sensor = _ElementsCategory(self._modbus)

