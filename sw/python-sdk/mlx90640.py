import ctypes
from dataclasses import dataclass
from typing import List


class MLX90640_I2CDriver:
    NOTIFY_ROUTINE = ctypes.CFUNCTYPE(None, ctypes.c_ulong)
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    def __init__(self, device_index=0, dll_path="i2c_driver/dll/CH341DLLA64.DLL"):
        self.device_index = device_index
        self.dll = ctypes.WinDLL(dll_path)

        self.dll.CH341OpenDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341OpenDevice.restype = ctypes.c_void_p

        self.dll.CH341CloseDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341CloseDevice.restype = ctypes.c_void_p

        self.dll.CH341SetStream.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        self.dll.CH341SetStream.restype = ctypes.c_bool

        self.dll.CH341StreamI2C.argtypes = [
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_void_p,
        ]
        self.dll.CH341StreamI2C.restype = ctypes.c_bool

        self.handle = self.INVALID_HANDLE_VALUE
        self.notify_routine = self.NOTIFY_ROUTINE(self.notify_callback)

    def init(self):
        # Initialize the I2C bus and any necessary parameters.
        self.handle = self.dll.CH341OpenDevice(self.device_index)
        if self.handle == self.INVALID_HANDLE_VALUE:
            print("Failed to open device.")
            return -1

        # 位1-位0: I2C接口速度/SCL频率, 00=低速/20KHz,01=标准/100KHz(默认值),10=快速/400KHz,11=高速/750KHz
        result = self.dll.CH341SetStream(self.device_index, 0x03)
        if result != 0:
            print("Failed to set I2c.")
            return -2

        return self.handle

    def reset(self):
        # Perform a general reset of the I2C bus.
        result = self.dll.CH341ResetDevice(self.device_index)
        if result != 0:
            print("Failed to reset I2C.")

        return result

    def read_words(self, slave_addr, start_address, n_mem_address_read):
        """
        Reads a word list from the I2C bus.

        Args:
            slave_addr (int): The slave address of the device.
            start_address (int): The start address to read from.
            n_mem_address_read (int): The number of memory addresses to read.

        Returns:
            list: A list of read data.

        """

        # Prepare the write data
        # The first byte is the slave address with the write direction bit
        # The second and third bytes are the start address
        write_data = bytes([slave_addr << 1]) + start_address.to_bytes(2, "big")

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(write_data)

        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(n_mem_address_read * 2)

        # Call the DLL function
        result = self.dll.CH341StreamI2C(
            self.device_index,
            len(write_data),
            write_buffer,
            n_mem_address_read * 2,
            read_buffer,
        )

        if result == 0:
            # Convert the read data to a list of integers
            data = [
                int.from_bytes(read_buffer[i : i + 2], "big")
                for i in range(0, len(read_buffer), 2)
            ]
        else:
            data = []

        return data

    def write_word(self, slaveAddr, writeAddress, data):
        """
        Writes a word of data to the specified I2C slave device.

        Parameters:
        - slaveAddr (int): The address of the I2C slave device.
        - writeAddress (int): The address to write the data to.
        - data (int): The data to be written.

        Returns:
        - result (int): The result of the write operation. Returns 0 if successful, otherwise returns an error code.
        """

        # The first byte is the slave address with the write direction bit
        # The second and third bytes are the write address
        # The fourth and fifth bytes are the data
        write_data = (
            bytes([slaveAddr << 1])
            + writeAddress.to_bytes(2, "big")
            + data.to_bytes(2, "big")
        )

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(write_data)

        # Call the DLL function
        result = self.dll.CH341StreamI2C(
            self.device_index, len(write_data), write_buffer, 0, None
        )

        if result != 0:
            print("Failed to write word.")

        return result

    def set_freq(self, freq):
        # Set the frequency of the I2C bus.
        # 位1-位0: I2C接口速度/SCL频率, 00=低速/20KHz,01=标准/100KHz(默认值),10=快速/400KHz,11=高速/750KHz
        if freq > 3:
            freq = 3
        result = self.dll.CH341SetStream(self.device_index, freq)
        if result != 0:
            print("Failed to set I2c.")

        return result


# Parameter structure
@dataclass
class MLX90640Params:
    kVdd: int
    vdd25: int
    KvPTAT: float
    KtPTAT: float
    vPTAT25: int
    alphaPTAT: float
    gainEE: int
    tgc: float
    cpKv: float
    cpKta: float
    resolutionEE: int
    calibrationModeEE: int
    KsTa: float
    ksTo: List[float]
    ct: List[int]
    alpha: List[int]
    alphaScale: int
    offset: List[int]
    kta: List[int]
    ktaScale: int
    kv: List[int]
    kvScale: int
    cpAlpha: List[float]
    cpOffset: List[int]
    ilChessC: List[float]
    brokenPixels: List[int]
    outlierPixels: List[int]


# Constants
MLX90640_NO_ERROR = 0
MLX90640_I2C_NACK_ERROR = 1
MLX90640_I2C_WRITE_ERROR = 2
MLX90640_BROKEN_PIXELS_NUM_ERROR = 3
MLX90640_OUTLIER_PIXELS_NUM_ERROR = 4
MLX90640_BAD_PIXELS_NUM_ERROR = 5
MLX90640_ADJACENT_BAD_PIXELS_ERROR = 6
MLX90640_EEPROM_DATA_ERROR = 7
MLX90640_FRAME_DATA_ERROR = 8
MLX90640_MEAS_TRIGGER_ERROR = 9

MLX90640_EEPROM_START_ADDRESS = 0x2400
MLX90640_EEPROM_DUMP_NUM = 832
MLX90640_PIXEL_DATA_START_ADDRESS = 0x0400
MLX90640_PIXEL_NUM = 768
MLX90640_LINE_NUM = 24
MLX90640_COLUMN_NUM = 32
MLX90640_LINE_SIZE = 32
MLX90640_COLUMN_SIZE = 24
MLX90640_AUX_DATA_START_ADDRESS = 0x0700
MLX90640_AUX_NUM = 64
MLX90640_STATUS_REG = 0x8000
MLX90640_INIT_STATUS_VALUE = 0x0030
MLX90640_STAT_FRAME_MASK = 1 << 0
MLX90640_STAT_DATA_READY_MASK = 1 << 3
MLX90640_CTRL_REG = 0x800D
MLX90640_CTRL_TRIG_READY_MASK = 1 << 15
MLX90640_CTRL_REFRESH_SHIFT = 7
MLX90640_CTRL_RESOLUTION_SHIFT = 10
MLX90640_CTRL_MEAS_MODE_SHIFT = 12
MLX90640_MS_BYTE_SHIFT = 8
MLX90640_MS_BYTE_MASK = 0xFF00
MLX90640_LS_BYTE_MASK = 0x00FF
MLX90640_MSBITS_6_MASK = 0xFC00
MLX90640_LSBITS_10_MASK = 0x03FF
MLX90640_NIBBLE1_MASK = 0x000F
MLX90640_NIBBLE2_MASK = 0x00F0
MLX90640_NIBBLE3_MASK = 0x0F00
MLX90640_NIBBLE4_MASK = 0xF000
SCALEALPHA = 0.000001


class MLX90640:
    def __init__(self, address: int = 0x33, i2c_driver: MLX90640_I2CDriver = None):
        if i2c_driver is None:
            print("Please specific I2C driver!")
            return

        self.address = address
        self.i2c_driver = i2c_driver

        self.i2c_driver.init()

    def _validate_aux_data(self, aux_data):
        if aux_data[0] == 0x7FFF:
            return -MLX90640_FRAME_DATA_ERROR
    
        ranges_to_check = [(8, 19), (20, 23), (24, 33), (40, 51), (52, 55), (56, 64)]
    
        for start, end in ranges_to_check:
            if 0x7FFF in aux_data[start:end]:
                return -MLX90640_FRAME_DATA_ERROR
    
        return MLX90640_NO_ERROR
    
    def dump_ee(self):
        return self.i2c_driver.read_words(self.address,
            MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM
        )

    def synch_frame(self):
        result = self.i2c_driver.write_word(self.address,
            MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE
        )
        if result != 0:
            return result

        data_ready = 0
        while data_ready == 0:
            status_register = self.i2c_driver.read_words(self.address, MLX90640_STATUS_REG, 1)
            if not status_register:
                print("Failed to read status register.")
                return status_register
            data_ready = status_register & MLX90640_STAT_DATA_READY_MASK

        return MLX90640_NO_ERROR

    def trigger_measurement(self):
        ctrl_reg = self.i2c_driver.read_words(self.address, MLX90640_CTRL_REG, 1)
        if not ctrl_reg:
            print("Failed to read control register.")
            return ctrl_reg

        ctrl_reg |= MLX90640_CTRL_TRIG_READY_MASK
        result = self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, ctrl_reg)
        if result != MLX90640_NO_ERROR:
            return result

        result = self.i2c_driver.reset()
        if result != MLX90640_NO_ERROR:
            return result

        self.i2c_driver.init()
        ctrl_reg = self.i2c_driver.read_words(self.address, MLX90640_CTRL_REG, 1)
        if not ctrl_reg:
            return ctrl_reg

        if (ctrl_reg & MLX90640_CTRL_TRIG_READY_MASK) != 0:
            return -MLX90640_MEAS_TRIGGER_ERROR

        return MLX90640_NO_ERROR

    def get_frame_data(self):
        data_ready = 0
        while data_ready == 0:
            status_register = self.i2c_driver.read_words(self.address, MLX90640_STATUS_REG, 1)
            if not status_register:
                return status_register
            data_ready = status_register & MLX90640_STAT_DATA_READY_MASK

        result = self.i2c_driver.write_word(self.address, 
            MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE
        )
        if result != 0:
            return result

        frame_data = self.i2c_driver.read_words(self.address, 
            MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM
        )
        if not frame_data:
            return frame_data

        aux_data = self.i2c_driver.read_words(self.address, 
            MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM
        )
        if not aux_data:
            return aux_data

        control_register1 = self.i2c_driver.read_words(self.address, MLX90640_CTRL_REG, 1)
        frame_data[832] = control_register1
        frame_data[833] = status_register & MLX90640_STAT_FRAME_MASK

        if not control_register1:
            return control_register1

        error = self._validate_aux_data(aux_data)
        if error == MLX90640_NO_ERROR:
            frame_data[MLX90640_PIXEL_NUM:MLX90640_PIXEL_NUM+MLX90640_AUX_NUM] = aux_data[:MLX90640_AUX_NUM]

        error = validate_frame_data(frame_data)
        if error != MLX90640_NO_ERROR:
            return error

        return frame_data[833]

    def extract_parameters(self, eeData):
        pass

    def get_vdd(self, frameData, params):
        pass

    def get_ta(self, frameData, params):
        pass

    def get_image(self, frameData, params):
        pass

    def calculate_to(self, frameData, params, emissivity, tr):
        pass

    def set_resolution(self, resolution):
        pass

    def get_cur_resolution(self):
        pass

    def set_refresh_rate(self, refreshRate):
        pass

    def get_refresh_rate(self):
        pass

    def get_sub_page_number(self, frameData):
        pass

    def get_cur_mode(self):
        pass

    def set_interleaved_mode(self):
        pass

    def set_chess_mode(self):
        pass

    def bad_pixels_correction(self, pixels, to, mode, params):
        pass