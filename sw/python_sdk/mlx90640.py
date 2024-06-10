import ctypes
from dataclasses import dataclass
from typing import List
import numpy as np
import math


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

        self.dll.CH341ResetDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341ResetDevice.restype = ctypes.c_bool

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
        success = self.dll.CH341SetStream(self.device_index, 0x03)
        if not success:
            print("Failed to set I2c.")
            return -2

        return self.handle

    def reset(self):
        # Perform a general reset of the I2C bus.
        success = self.dll.CH341ResetDevice(self.device_index)
        if not success:
            print("Failed to reset I2C.")
            return False

        self.init()
        return success

    def read_words(self, slave_address, start_address, n_mem_address_read):
        """
        Reads a word list from the I2C bus.

        Args:
            slave_address (int): The slave address of the device.
            start_address (int): The start address to read from.
            n_mem_address_read (int): The number of memory addresses to read.

        Returns:
            list: A list of read data.

        """

        # Prepare the write data
        # The first byte is the slave address with the write direction bit
        # The second and third bytes are the start address
        write_data = bytes([slave_address << 1]) + start_address.to_bytes(2, "big")

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(write_data)

        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(n_mem_address_read * 2)

        # Call the DLL function
        success = self.dll.CH341StreamI2C(
            self.device_index,
            len(write_data),
            write_buffer,
            n_mem_address_read * 2,
            read_buffer,
        )

        if success:
            # if raw:
            #     # Convert the read data to a list of integers
            #     data = [
            #         int.from_bytes(read_buffer[i : i + 2], "big")
            #         for i in range(0, len(read_buffer), 2)
            #     ]
            # else:
            data = np.frombuffer(read_buffer, dtype=">i2").tolist()
        else:
            data = []

        return data

    def write_word(self, slave_address, write_address, data):
        """
        Writes a word of data to a specific address on the I2C bus.

        Args:
            slave_address (int): The slave address of the device.
            write_address (int): The address to write the data to.
            data (int): The data to be written.

        Returns:
            bool: True if the write operation was successful, False otherwise.
        """
        # The first byte is the slave address with the write direction bit
        # The second and third bytes are the write address
        # The fourth and fifth bytes are the data
        write_data = (
            bytes([slave_address << 1])
            + write_address.to_bytes(2, "big")
            + data.to_bytes(2, "big")
        )

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(write_data)

        # Call the DLL function
        success = self.dll.CH341StreamI2C(
            self.device_index, len(write_data), write_buffer, 0, None
        )

        if not success:
            print("Failed to write word.")

        return success

    def set_freq(self, freq):
        # Set the frequency of the I2C bus.
        # 位1-位0: I2C接口速度/SCL频率, 00=低速/20KHz,01=标准/100KHz(默认值),10=快速/400KHz,11=高速/750KHz
        if freq > 3:
            freq = 3
        success = self.dll.CH341SetStream(self.device_index, freq)
        if not success:
            print("Failed to set I2c.")

        return success

    def notify_callback(self):
        # Notify callback function.
        pass

    def close_device(self):
        if self.handle != self.INVALID_HANDLE_VALUE:
            self.dll.CH341CloseDevice(self.device_index)


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
MLX90640_CTRL_REFRESH_MASK = 0x07 << MLX90640_CTRL_REFRESH_SHIFT
MLX90640_CTRL_RESOLUTION_SHIFT = 10
MLX90640_CTRL_RESOLUTION_MASK = 0x03 << MLX90640_CTRL_RESOLUTION_SHIFT
MLX90640_CTRL_MEAS_MODE_SHIFT = 12
MLX90640_CTRL_MEAS_MODE_MASK = 0x01 << MLX90640_CTRL_MEAS_MODE_SHIFT
MLX90640_MS_BYTE_MASK = 0xFF00
MLX90640_LS_BYTE_MASK = 0x00FF
MLX90640_MSBITS_6_MASK = 0xFC00
MLX90640_LSBITS_10_MASK = 0x03FF
MLX90640_NIBBLE1_MASK = 0x000F
MLX90640_NIBBLE2_MASK = 0x00F0
MLX90640_NIBBLE3_MASK = 0x0F00
MLX90640_NIBBLE4_MASK = 0xF000

SCALEALPHA = 0.000001
OPENAIR_TA_SHIFT = 8


class MLX90640:
    def __init__(self, address: int = 0x33, i2c_driver: MLX90640_I2CDriver = None):
        if i2c_driver is None:
            i2c_driver = MLX90640_I2CDriver()

        self.address = address
        self.i2c_driver = i2c_driver

        self.i2c_driver.init()

        self.params = MLX90640Params(
            kVdd=0,
            vdd25=0,
            KvPTAT=0,
            KtPTAT=0,
            vPTAT25=0,
            alphaPTAT=0,
            gainEE=0,
            tgc=0,
            cpKv=0,
            cpKta=0,
            resolutionEE=0,
            calibrationModeEE=0,
            KsTa=0,
            ksTo=[0] * 5,
            ct=[0] * 5,
            alpha=[0] * 768,
            alphaScale=0,
            offset=[0] * 768,
            kta=[0] * 768,
            ktaScale=0,
            kv=[0] * 768,
            kvScale=0,
            cpAlpha=[0] * 2,
            cpOffset=[0] * 2,
            ilChessC=[0] * 3,
            brokenPixels=[0xFFFF] * 5,
            outlierPixels=[0xFFFF] * 5,
        )
        self.extract_parameters()

    def _validate_aux_data(self, aux_data):
        if aux_data[0] == 0x7FFF:
            return -MLX90640_FRAME_DATA_ERROR

        ranges_to_check = [(8, 19), (20, 23), (24, 33), (40, 51), (52, 55), (56, 64)]

        for start, end in ranges_to_check:
            if 0x7FFF in aux_data[start:end]:
                return -MLX90640_FRAME_DATA_ERROR

        return MLX90640_NO_ERROR

    def _validate_frame_data(self, frame_data):
        for i, data in enumerate(frame_data[0:MLX90640_PIXEL_NUM:MLX90640_LINE_SIZE]):
            if data == 0x7FFF and i % 2 == frame_data[833]:
                return -MLX90640_FRAME_DATA_ERROR

        return MLX90640_NO_ERROR

    def dump_ee(self):
        return self.i2c_driver.read_words(
            self.address, MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM
        )

    def sync_frame(self):
        if not self.i2c_driver.write_word(
            self.address, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE
        ):
            return -1

        data_ready = 0
        while data_ready == 0:
            status_register = self.i2c_driver.read_words(
                self.address, MLX90640_STATUS_REG, 1
            )[0]
            if not status_register:
                print("Failed to read status register.")
                return status_register
            data_ready = status_register & MLX90640_STAT_DATA_READY_MASK

        return MLX90640_NO_ERROR

    def trigger_measurement(self):
        ctrl_reg = self.i2c_driver.read_words(self.address, MLX90640_CTRL_REG, 1)[0]
        if not ctrl_reg:
            print("Failed to read control register.")
            return ctrl_reg

        ctrl_reg |= MLX90640_CTRL_TRIG_READY_MASK
        if not self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, ctrl_reg):
            print("Failed to write control register.")
            return -1

        if not self.i2c_driver.reset():
            print("Failed to reset I2C.")
            return -2

        self.i2c_driver.init()
        ctrl_reg = self.i2c_driver.read_words(self.address, MLX90640_CTRL_REG, 1)[0]
        print(ctrl_reg)
        if not ctrl_reg:
            return ctrl_reg

        if (ctrl_reg & MLX90640_CTRL_TRIG_READY_MASK) != 0:
            return -MLX90640_MEAS_TRIGGER_ERROR

        return MLX90640_NO_ERROR

    def get_frame_data(self):
        data_ready = 0
        while data_ready == 0:
            status_register = self.i2c_driver.read_words(
                self.address, MLX90640_STATUS_REG, 1
            )[0]
            if not status_register:
                print("Failed to read status register.")
                return None
            data_ready = status_register & MLX90640_STAT_DATA_READY_MASK

        if not self.i2c_driver.write_word(
            self.address, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE
        ):
            print("Failed to write status register.")
            return None

        frame_data = self.i2c_driver.read_words(
            self.address, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM
        )
        if not frame_data:
            print("Failed to read frame data.")
            return None

        aux_data = self.i2c_driver.read_words(
            self.address, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM
        )
        if not aux_data:
            print("Failed to read aux data.")
            return None

        control_register1 = self.i2c_driver.read_words(
            self.address, MLX90640_CTRL_REG, 1
        )[0]
        frame_data += [0] * MLX90640_AUX_NUM
        frame_data += [control_register1, status_register & MLX90640_STAT_FRAME_MASK]

        if not control_register1:
            print("Failed to read control register.")
            return None

        error = self._validate_aux_data(aux_data)
        if error != MLX90640_NO_ERROR:
            print("Error in aux data validation.")
            return None

        frame_data[MLX90640_PIXEL_NUM : MLX90640_PIXEL_NUM + MLX90640_AUX_NUM] = (
            aux_data[:MLX90640_AUX_NUM]
        )

        error = self._validate_frame_data(frame_data)
        if error != MLX90640_NO_ERROR:
            print("Error in frame data validation.")
            return None

        return frame_data

    def extract_parameters(self, ee_data=None):
        if ee_data is None:
            ee_data = self.dump_ee()

        self.params.kVdd = ee_data[51] >> 8
        if self.params.kVdd > 127:
            self.params.kVdd -= 256
        self.params.kVdd *= 32

        self.params.vdd25 = (((ee_data[51] & 0xFF) - 256) << 5) - 8192

        KvPTAT = (ee_data[50] & 0xFC00) >> 10
        if KvPTAT > 31:
            KvPTAT -= 64
        self.params.KvPTAT = KvPTAT / 4096

        KtPTAT = ee_data[50] & 0x03FF
        if KtPTAT > 511:
            KtPTAT -= 1024
        self.params.KtPTAT = KtPTAT / 8

        self.params.vPTAT25 = ee_data[49]

        self.params.alphaPTAT = (ee_data[16] & 0xF000) / pow(2, 14) + 8.0

        self.params.gainEE = ee_data[48]
        if self.params.gainEE > 32767:
            self.params.gainEE -= 65536

        self.params.tgc = ee_data[60] & 0xFF
        if self.params.tgc > 127:
            self.params.tgc -= 256
        self.params.tgc /= 32.0

        self.params.resolutionEE = (ee_data[56] & 0x3000) >> 12

        self.params.KsTa = (ee_data[60] & 0xFF00) >> 8
        if self.params.KsTa > 127:
            self.params.KsTa -= 256
        self.params.KsTa /= 8192.0

        step = ((ee_data[63] & 0x3000) >> 12) * 10
        self.params.ct[0] = -40
        self.params.ct[1] = 0
        self.params.ct[2] = (ee_data[63] & 0x00F0) >> 4
        self.params.ct[3] = (ee_data[63] & 0x0F00) >> 8
        self.params.ct[2] *= step
        self.params.ct[3] = self.params.ct[2] + self.params.ct[3] * step
        self.params.ct[4] = 400

        ks_to_scale = (ee_data[63] & 0x000F) + 8
        ks_to_scale = 1 << ks_to_scale

        self.params.ksTo[0] = ee_data[61] & 0xFF
        self.params.ksTo[1] = (ee_data[61] & 0xFF00) >> 8
        self.params.ksTo[2] = ee_data[62] & 0xFF
        self.params.ksTo[3] = (ee_data[62] & 0xFF00) >> 8
        for i in range(4):
            if self.params.ksTo[i] > 127:
                self.params.ksTo[i] -= 256
            self.params.ksTo[i] /= ks_to_scale
        self.params.ksTo[4] = -0.0002

        # self.params.alpha, self.params.alphaScale = self._extract_alpha_parameters(
        #     ee_data
        # )
        self._extract_alpha_parameters(ee_data)

        # self.params.offset = self._extract_offset_parameters(ee_data)
        self._extract_offset_parameters(ee_data)

        # self.params.kta, self.params.ktaScale = self._extract_kta_pixel_parameters(
        #     ee_data
        # )
        self._extract_kta_pixel_parameters(ee_data)

        # self.params.kv, self.params.kvScale = self._extract_kv_pixel_parameters(ee_data)
        self._extract_kv_pixel_parameters(ee_data)

        # (
        #     self.params.cpAlpha,
        #     self.params.cpOffset,
        #     self.params.cpKta,
        #     self.params.cpKv,
        # ) = self._extract_cp_parameters(ee_data)
        self._extract_cp_parameters(ee_data)

        # self.params.calibrationModeEE, self.params.ilChessC = (
        #     self._extract_cilc_parameters(ee_data)
        # )
        self._extract_cilc_parameters(ee_data)

        warn, broken_pixels, outlier_pixels = self._extract_deviating_pixels(ee_data)
        if not warn:
            self.params.brokenPixels = broken_pixels
            self.params.outlierPixels = outlier_pixels
        else:
            print("Error in deviating pixels extraction.")

    def _extract_alpha_parameters(self, ee_data):
        # extract alpha
        accRemScale = ee_data[32] & 0x000F
        accColumnScale = (ee_data[32] & 0x00F0) >> 4
        accRowScale = (ee_data[32] & 0x0F00) >> 8
        alphaScale = ((ee_data[32] & 0xF000) >> 12) + 30
        alphaRef = ee_data[33]
        accRow = [0] * 24
        accColumn = [0] * 32
        alphaTemp = [0] * 768

        for i in range(6):
            p = i * 4
            accRow[p + 0] = ee_data[34 + i] & 0x000F
            accRow[p + 1] = (ee_data[34 + i] & 0x00F0) >> 4
            accRow[p + 2] = (ee_data[34 + i] & 0x0F00) >> 8
            accRow[p + 3] = (ee_data[34 + i] & 0xF000) >> 12

        for i in range(24):
            if accRow[i] > 7:
                accRow[i] -= 16

        for i in range(8):
            p = i * 4
            accColumn[p + 0] = ee_data[40 + i] & 0x000F
            accColumn[p + 1] = (ee_data[40 + i] & 0x00F0) >> 4
            accColumn[p + 2] = (ee_data[40 + i] & 0x0F00) >> 8
            accColumn[p + 3] = (ee_data[40 + i] & 0xF000) >> 12

        for i in range(32):
            if accColumn[i] > 7:
                accColumn[i] -= 16

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                alphaTemp[p] = (ee_data[64 + p] & 0x03F0) >> 4
                if alphaTemp[p] > 31:
                    alphaTemp[p] -= 64
                alphaTemp[p] *= 1 << accRemScale
                alphaTemp[p] += (
                    alphaRef
                    + (accRow[i] << accRowScale)
                    + (accColumn[j] << accColumnScale)
                )
                alphaTemp[p] /= pow(2, alphaScale)
                alphaTemp[p] -= (
                    self.params.tgc
                    * (self.params.cpAlpha[0] + self.params.cpAlpha[1])
                    / 2
                )
                alphaTemp[p] = SCALEALPHA / alphaTemp[p]
        # print("alphaTemp: ", alphaTemp)

        temp = max(alphaTemp)
        # print("temp", temp)

        alphaScale = 0
        while temp < 32768:
            temp *= 2
            alphaScale += 1

        for i in range(768):
            temp = alphaTemp[i] * pow(2, alphaScale)
            self.params.alpha[i] = int(temp + 0.5)

        self.params.alphaScale = alphaScale

    def _extract_offset_parameters(self, ee_data):
        # extract offset
        occRow = [0] * 24
        occColumn = [0] * 32

        occRemScale = ee_data[16] & 0x000F
        occColumnScale = (ee_data[16] & 0x00F0) >> 4
        occRowScale = (ee_data[16] & 0x0F00) >> 8
        offsetRef = ee_data[17]
        if offsetRef > 32767:
            offsetRef -= 65536

        for i in range(6):
            p = i * 4
            occRow[p + 0] = ee_data[18 + i] & 0x000F
            occRow[p + 1] = (ee_data[18 + i] & 0x00F0) >> 4
            occRow[p + 2] = (ee_data[18 + i] & 0x0F00) >> 8
            occRow[p + 3] = (ee_data[18 + i] & 0xF000) >> 12

        for i in range(24):
            if occRow[i] > 7:
                occRow[i] -= 16

        for i in range(8):
            p = i * 4
            occColumn[p + 0] = ee_data[24 + i] & 0x000F
            occColumn[p + 1] = (ee_data[24 + i] & 0x00F0) >> 4
            occColumn[p + 2] = (ee_data[24 + i] & 0x0F00) >> 8
            occColumn[p + 3] = (ee_data[24 + i] & 0xF000) >> 12

        for i in range(32):
            if occColumn[i] > 7:
                occColumn[i] -= 16

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                self.params.offset[p] = (ee_data[64 + p] & 0xFC00) >> 10
                if self.params.offset[p] > 31:
                    self.params.offset[p] -= 64
                self.params.offset[p] *= 1 << occRemScale
                self.params.offset[p] += (
                    offsetRef
                    + (occRow[i] << occRowScale)
                    + (occColumn[j] << occColumnScale)
                )

    def _extract_kta_pixel_parameters(self, ee_data):
        # extract KtaPixel
        KtaRC = [0] * 4
        ktaTemp = [0] * 768

        KtaRoCo = (ee_data[54] & 0xFF00) >> 8
        if KtaRoCo > 127:
            KtaRoCo -= 256
        KtaRC[0] = KtaRoCo

        KtaReCo = ee_data[54] & 0x00FF
        if KtaReCo > 127:
            KtaReCo -= 256
        KtaRC[2] = KtaReCo

        KtaRoCe = (ee_data[55] & 0xFF00) >> 8
        if KtaRoCe > 127:
            KtaRoCe -= 256
        KtaRC[1] = KtaRoCe

        KtaReCe = ee_data[55] & 0x00FF
        if KtaReCe > 127:
            KtaReCe -= 256
        KtaRC[3] = KtaReCe

        ktaScale1 = ((ee_data[56] & 0x00F0) >> 4) + 8
        ktaScale2 = ee_data[56] & 0x000F

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = 2 * (p // 32 - (p // 64) * 2) + p % 2
                ktaTemp[p] = (ee_data[64 + p] & 0x000E) >> 1
                if ktaTemp[p] > 3:
                    ktaTemp[p] -= 8
                ktaTemp[p] *= 1 << ktaScale2
                ktaTemp[p] += KtaRC[split]
                ktaTemp[p] /= pow(2, ktaScale1)

        temp = abs(ktaTemp[0])
        for kta in ktaTemp:
            temp = max(temp, abs(kta))

        ktaScale1 = 0
        while temp < 64:
            temp *= 2
            ktaScale1 += 1

        for i in range(768):
            temp = ktaTemp[i] * pow(2, ktaScale1)
            if temp < 0:
                self.params.kta[i] = int(temp - 0.5)
            else:
                self.params.kta[i] = int(temp + 0.5)
        self.params.ktaScale = ktaScale1

    def _extract_kv_pixel_parameters(self, ee_data):
        KvT = [0] * 4
        kvTemp = [0] * 768

        KvRoCo = (ee_data[52] & 0xF000) >> 12
        if KvRoCo > 7:
            KvRoCo -= 16
        KvT[0] = KvRoCo

        KvReCo = (ee_data[52] & 0x0F00) >> 8
        if KvReCo > 7:
            KvReCo -= 16
        KvT[2] = KvReCo

        KvRoCe = (ee_data[52] & 0x00F0) >> 4
        if KvRoCe > 7:
            KvRoCe -= 16
        KvT[1] = KvRoCe

        KvReCe = ee_data[52] & 0x000F
        if KvReCe > 7:
            KvReCe -= 16
        KvT[3] = KvReCe

        kvScale = (ee_data[56] & 0x0F00) >> 8

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = 2 * (p // 32 - (p // 64) * 2) + p % 2
                kvTemp[p] = KvT[split]
                kvTemp[p] /= pow(2, kvScale)

        temp = abs(kvTemp[0])
        for kv in kvTemp:
            temp = max(temp, abs(kv))

        kvScale = 0
        while temp < 64:
            temp *= 2
            kvScale += 1

        for i in range(768):
            temp = kvTemp[i] * pow(2, kvScale)
            if temp < 0:
                self.params.kv[i] = int(temp - 0.5)
            else:
                self.params.kv[i] = int(temp + 0.5)
        self.params.kvScale = kvScale

    def _extract_cp_parameters(self, ee_data):
        offsetSP = [0] * 2
        alphaSP = [0] * 2

        alphaScale = ((ee_data[32] & 0xF000) >> 12) + 27

        offsetSP[0] = ee_data[58] & 0x03FF
        if offsetSP[0] > 511:
            offsetSP[0] -= 1024

        offsetSP[1] = (ee_data[58] & 0xFC00) >> 10
        if offsetSP[1] > 31:
            offsetSP[1] -= 64
        offsetSP[1] += offsetSP[0]

        alphaSP[0] = ee_data[57] & 0x03FF
        if alphaSP[0] > 511:
            alphaSP[0] -= 1024
        alphaSP[0] /= pow(2, alphaScale)

        alphaSP[1] = (ee_data[57] & 0xFC00) >> 10
        if alphaSP[1] > 31:
            alphaSP[1] -= 64
        alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0]

        cpKta = ee_data[59] & 0x00FF
        if cpKta > 127:
            cpKta -= 256
        ktaScale1 = ((ee_data[56] & 0x00F0) >> 4) + 8
        self.params.cpKta = cpKta / pow(2, ktaScale1)

        cpKv = (ee_data[59] & 0xFF00) >> 8
        if cpKv > 127:
            cpKv -= 256
        kvScale = (ee_data[56] & 0x0F00) >> 8
        self.params.cpKv = cpKv / pow(2, kvScale)

        self.params.cpAlpha[0] = alphaSP[0]
        self.params.cpAlpha[1] = alphaSP[1]
        self.params.cpOffset[0] = offsetSP[0]
        self.params.cpOffset[1] = offsetSP[1]

    def _extract_cilc_parameters(self, ee_data):
        ilChessC = [0] * 3

        self.params.calibrationModeEE = (ee_data[10] & 0x0800) >> 4
        self.params.calibrationModeEE = self.params.calibrationModeEE ^ 0x80

        ilChessC[0] = ee_data[53] & 0x003F
        if ilChessC[0] > 31:
            ilChessC[0] -= 64
        ilChessC[0] /= 16.0

        ilChessC[1] = (ee_data[53] & 0x07C0) >> 6
        if ilChessC[1] > 15:
            ilChessC[1] -= 32
        ilChessC[1] /= 2.0

        ilChessC[2] = (ee_data[53] & 0xF800) >> 11
        if ilChessC[2] > 15:
            ilChessC[2] -= 32
        ilChessC[2] /= 8.0

        self.params.ilChessC = ilChessC

    def _extract_deviating_pixels(self, ee_data):
        broken_pixels = [0xFFFF] * 5
        outlier_pixels = [0xFFFF] * 5
        broken_pix_cnt = 0
        outlier_pix_cnt = 0
        warn = 0

        pix_cnt = 0
        while (
            pix_cnt < MLX90640_PIXEL_NUM and broken_pix_cnt < 5 and outlier_pix_cnt < 5
        ):
            if ee_data[pix_cnt + 64] == 0:
                broken_pixels[broken_pix_cnt] = pix_cnt
                broken_pix_cnt += 1
            elif (ee_data[pix_cnt + 64] & 0x0001) != 0:
                outlier_pixels[outlier_pix_cnt] = pix_cnt
                outlier_pix_cnt += 1

            pix_cnt += 1

        if broken_pix_cnt > 4:
            warn = -MLX90640_BROKEN_PIXELS_NUM_ERROR
        elif outlier_pix_cnt > 4:
            warn = -MLX90640_OUTLIER_PIXELS_NUM_ERROR
        elif (broken_pix_cnt + outlier_pix_cnt) > 4:
            warn = -MLX90640_BAD_PIXELS_NUM_ERROR
        else:
            for i in range(broken_pix_cnt):
                for j in range(i + 1, broken_pix_cnt):
                    warn = self._check_adjacent_pixels(
                        broken_pixels[i], broken_pixels[j]
                    )
                    if warn != 0:
                        return warn

            for i in range(outlier_pix_cnt):
                for j in range(i + 1, outlier_pix_cnt):
                    warn = self._check_adjacent_pixels(
                        outlier_pixels[i], outlier_pixels[j]
                    )
                    if warn != 0:
                        return warn

            for i in range(broken_pix_cnt):
                for j in range(outlier_pix_cnt):
                    warn = self._check_adjacent_pixels(
                        broken_pixels[i], outlier_pixels[j]
                    )
                    if warn != 0:
                        return warn

        return warn, broken_pixels, outlier_pixels

    def _check_adjacent_pixels(self, pix1, pix2):
        lp1 = pix1 >> 5
        lp2 = pix2 >> 5
        cp1 = pix1 - (lp1 << 5)
        cp2 = pix2 - (lp2 << 5)

        pix_pos_dif = lp1 - lp2
        if -2 < pix_pos_dif < 2:
            pix_pos_dif = cp1 - cp2
            if -2 < pix_pos_dif < 2:
                return -6

        return 0

    def _get_median(self, values):
        values.sort()
        n = len(values)
        if n % 2 == 0:
            return (values[n // 2] + values[n // 2 - 1]) / 2.0
        else:
            return values[n // 2]

    def _is_pixel_bad(self, pixel):
        for i in range(5):
            if (
                pixel == self.params.outlierPixels[i]
                or pixel == self.params.brokenPixels[i]
            ):
                return True
        return False

    def get_vdd(self, frame_data=None):
        if frame_data is None:
            frame_data = self.get_frame_data()

        if not frame_data:
            return -1

        resolution_correction = pow(2, self.params.resolutionEE) / pow(
            2,
            (frame_data[832] & MLX90640_CTRL_RESOLUTION_MASK)
            >> MLX90640_CTRL_RESOLUTION_SHIFT,
        )
        vdd = (
            resolution_correction * frame_data[810] - self.params.vdd25
        ) / self.params.kVdd + 3.3
        return vdd

    def get_ta(self, frame_data=None):
        if frame_data is None:
            frame_data = self.get_frame_data()

        vdd = self.get_vdd(frame_data=frame_data)

        ptat = frame_data[800]
        ptat_art = (ptat / (ptat * self.params.alphaPTAT + frame_data[768])) * pow(
            2, 18
        )
        ta = ptat_art / (1 + self.params.KvPTAT * (vdd - 3.3)) - self.params.vPTAT25
        ta = ta / self.params.KtPTAT + 25
        return ta

    def get_image(self, frame_data=None):
        if frame_data is None:
            frame_data = self.get_frame_data()

        vdd = self.get_vdd(frame_data=frame_data)
        ta = self.get_ta(frame_data=frame_data)

        kta_scale = pow(2, self.params.ktaScale)
        kv_scale = pow(2, self.params.kvScale)

        gain = self.params.gainEE / frame_data[778]

        mode = (frame_data[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5

        ir_data_cp = [0, 0]
        ir_data_cp[0] = frame_data[776] * gain
        ir_data_cp[1] = frame_data[808] * gain

        ir_data_cp[0] -= (
            self.params.cpOffset[0]
            * (1 + self.params.cpKta * (ta - 25))
            * (1 + self.params.cpKv * (vdd - 3.3))
        )
        if mode == self.params.calibrationModeEE:
            ir_data_cp[1] -= (
                self.params.cpOffset[1]
                * (1 + self.params.cpKta * (ta - 25))
                * (1 + self.params.cpKv * (vdd - 3.3))
            )
        else:
            ir_data_cp[1] -= (
                (self.params.cpOffset[1] + self.params.ilChessC[0])
                * (1 + self.params.cpKta * (ta - 25))
                * (1 + self.params.cpKv * (vdd - 3.3))
            )

        result = [0] * 768
        for pixel_number in range(768):
            il_pattern = pixel_number // 32 - (pixel_number // 64) * 2
            chess_pattern = il_pattern ^ (pixel_number - (pixel_number // 2) * 2)
            conversion_pattern = (
                (pixel_number + 2) // 4
                - (pixel_number + 3) // 4
                + (pixel_number + 1) // 4
                - pixel_number // 4
            ) * (1 - 2 * il_pattern)

            pattern = il_pattern if mode == 0 else chess_pattern

            if pattern == frame_data[833]:
                ir_data = frame_data[pixel_number] * gain

                kta = self.params.kta[pixel_number] / kta_scale
                kv = self.params.kv[pixel_number] / kv_scale
                ir_data -= (
                    self.params.offset[pixel_number]
                    * (1 + kta * (ta - 25))
                    * (1 + kv * (vdd - 3.3))
                )

                if mode != self.params.calibrationModeEE:
                    ir_data += (
                        self.params.ilChessC[2] * (2 * il_pattern - 1)
                        - self.params.ilChessC[1] * conversion_pattern
                    )

                ir_data -= self.params.tgc * ir_data_cp[frame_data[833]]

                alpha_compensated = self.params.alpha[pixel_number]

                image = ir_data * alpha_compensated

                result[pixel_number] = image

        return result

    def calculate_to(self, emissivity: float = 0.95, tr: float = None, frame_data=None):
        if frame_data is None:
            frame_data = self.get_frame_data()

        subPage = frame_data[833]
        alphaCorrR = [0] * 4
        irDataCP = [0, 0]

        vdd = self.get_vdd(frame_data)
        ta = self.get_ta(frame_data)
        if tr is None:
            tr = ta - OPENAIR_TA_SHIFT

        ta4 = ta + 273.15
        ta4 = ta4 * ta4
        ta4 = ta4 * ta4
        tr4 = tr + 273.15
        tr4 = tr4 * tr4
        tr4 = tr4 * tr4
        taTr = tr4 - (tr4 - ta4) / emissivity

        ktaScale = pow(2, self.params.ktaScale)
        kvScale = pow(2, self.params.kvScale)
        alphaScale = pow(2, self.params.alphaScale)

        alphaCorrR[0] = 1 / (1 + self.params.ksTo[0] * 40)
        alphaCorrR[1] = 1
        alphaCorrR[2] = 1 + self.params.ksTo[1] * self.params.ct[2]
        alphaCorrR[3] = alphaCorrR[2] * (
            1 + self.params.ksTo[2] * (self.params.ct[3] - self.params.ct[2])
        )

        # --------- Gain calculation -----------------------------------
        gain = frame_data[778]
        if gain > 32767:
            gain -= 65536
        gain = self.params.gainEE / gain

        # --------- To calculation -------------------------------------
        mode = (frame_data[832] & 0x1000) >> 5

        irDataCP[0] = frame_data[776]
        irDataCP[1] = frame_data[808]
        for i in range(2):
            if irDataCP[i] > 32767:
                irDataCP[i] -= 65536
            irDataCP[i] *= gain

        irDataCP[0] -= (
            self.params.cpOffset[0]
            * (1 + self.params.cpKta * (ta - 25))
            * (1 + self.params.cpKv * (vdd - 3.3))
        )
        if mode == self.params.calibrationModeEE:
            irDataCP[1] -= (
                self.params.cpOffset[1]
                * (1 + self.params.cpKta * (ta - 25))
                * (1 + self.params.cpKv * (vdd - 3.3))
            )
        else:
            irDataCP[1] -= (
                (self.params.cpOffset[1] + self.params.ilChessC[0])
                * (1 + self.params.cpKta * (ta - 25))
                * (1 + self.params.cpKv * (vdd - 3.3))
            )

        result = [0] * 768
        for pixelNumber in range(768):
            if self._is_pixel_bad(pixelNumber):
                # print("Fixing broken pixel %d" % pixelNumber)
                result[pixelNumber] = -273.15
                continue

            ilPattern = pixelNumber // 32 - (pixelNumber // 64) * 2
            chessPattern = ilPattern ^ (pixelNumber - (pixelNumber // 2) * 2)
            conversionPattern = (
                (pixelNumber + 2) // 4
                - (pixelNumber + 3) // 4
                + (pixelNumber + 1) // 4
                - pixelNumber // 4
            ) * (1 - 2 * ilPattern)

            if mode == 0:
                pattern = ilPattern
            else:
                pattern = chessPattern

            if pattern == frame_data[833]:
                irData = frame_data[pixelNumber]
                if irData > 32767:
                    irData -= 65536
                irData *= gain

                kta = self.params.kta[pixelNumber] / ktaScale
                kv = self.params.kv[pixelNumber] / kvScale
                irData -= (
                    self.params.offset[pixelNumber]
                    * (1 + kta * (ta - 25))
                    * (1 + kv * (vdd - 3.3))
                )

                if mode != self.params.calibrationModeEE:
                    irData += (
                        self.params.ilChessC[2] * (2 * ilPattern - 1)
                        - self.params.ilChessC[1] * conversionPattern
                    )

                irData = irData - self.params.tgc * irDataCP[subPage]
                irData /= emissivity

                alphaCompensated = (
                    SCALEALPHA * alphaScale / self.params.alpha[pixelNumber]
                )
                alphaCompensated *= 1 + self.params.KsTa * (ta - 25)

                Sx = (
                    alphaCompensated
                    * alphaCompensated
                    * alphaCompensated
                    * (irData + alphaCompensated * taTr)
                )
                Sx = math.sqrt(math.sqrt(Sx)) * self.params.ksTo[1]

                To = (
                    math.sqrt(
                        math.sqrt(
                            irData
                            / (
                                alphaCompensated * (1 - self.params.ksTo[1] * 273.15)
                                + Sx
                            )
                            + taTr
                        )
                    )
                    - 273.15
                )

                if To < self.params.ct[1]:
                    torange = 0
                elif To < self.params.ct[2]:
                    torange = 1
                elif To < self.params.ct[3]:
                    torange = 2
                else:
                    torange = 3

                To = (
                    math.sqrt(
                        math.sqrt(
                            irData
                            / (
                                alphaCompensated
                                * alphaCorrR[torange]
                                * (
                                    1
                                    + self.params.ksTo[torange]
                                    * (To - self.params.ct[torange])
                                )
                            )
                            + taTr
                        )
                    )
                    - 273.15
                )

                result[pixelNumber] = To

        return result

    def set_resolution(self, resolution):
        """
        Sets the resolution of the MLX90640 sensor.

        Args:
            resolution (int): The desired resolution value.
            - 0: ADC set to 16 bit resolution
            - 1: ADC set to 17 bit resolution
            - 2: ADC set to 18 bit resolution (default)
            - 3: ADC set to 19 bit resolution

        Returns:
            bool: True if the resolution was set successfully, False otherwise.
        """
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            value = controlRegister1 & ~MLX90640_CTRL_RESOLUTION_MASK
            value |= resolution << MLX90640_CTRL_RESOLUTION_SHIFT
            self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, value)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def get_cur_resolution(self):
        """
        Get the current resolution of the MLX90640 sensor.

        Returns:
            int: The current resolution of the sensor.
            - 0: ADC set to 16 bit resolution
            - 1: ADC set to 17 bit resolution
            - 2: ADC set to 18 bit resolution (default)
            - 3: ADC set to 19 bit resolution

        Raises:
            Exception: If there is an error reading the control register.
        """
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            resolutionRAM = (
                controlRegister1 & MLX90640_CTRL_RESOLUTION_MASK
            ) >> MLX90640_CTRL_RESOLUTION_SHIFT
            return resolutionRAM
        except Exception as e:
            print(f"Error: {e}")
            return None

    def set_refresh_rate(self, refresh_rate):
        """
        Set the refresh rate of the sensor.

        Args:
            refresh_rate (int): The refresh rate value to set. Valid values are:
            - 0: IR refresh rate = 0.5Hz
            - 1: IR refresh rate = 1Hz
            - 2: IR refresh rate = 2Hz (default)
            - 3: IR refresh rate = 4Hz
            - 4: IR refresh rate = 8Hz
            - 5: IR refresh rate = 16Hz
            - 6: IR refresh rate = 32Hz
            - 7: IR refresh rate = 64Hz

        Returns:
            bool: True if the refresh rate was set successfully, False otherwise.
        """
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            value = controlRegister1 & ~MLX90640_CTRL_REFRESH_MASK
            value |= refresh_rate << MLX90640_CTRL_REFRESH_SHIFT
            self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, value)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def get_refresh_rate(self):
        """
        Get the refresh rate of the MLX90640 sensor.

        Returns:
            int: The refresh rate value.
            - 0: IR refresh rate = 0.5Hz
            - 1: IR refresh rate = 1Hz
            - 2: IR refresh rate = 2Hz (default)
            - 3: IR refresh rate = 4Hz
            - 4: IR refresh rate = 8Hz
            - 5: IR refresh rate = 16Hz
            - 6: IR refresh rate = 32Hz
            - 7: IR refresh rate = 64Hz

        Raises:
            Exception: If there is an error reading the control register.
        """
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            refreshRate = (
                controlRegister1 & MLX90640_CTRL_REFRESH_MASK
            ) >> MLX90640_CTRL_REFRESH_SHIFT
            return refreshRate
        except Exception as e:
            print(f"Error: {e}")
            return None

    def get_sub_page_number(self, frame_data=None):
        if frame_data is None:
            frame_data = self.get_frame_data()

        return frame_data[833]

    def get_cur_mode(self):
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            modeRAM = (
                controlRegister1 & MLX90640_CTRL_MEAS_MODE_MASK
            ) >> MLX90640_CTRL_MEAS_MODE_SHIFT
            return modeRAM
        except Exception as e:
            print(f"Error: {e}")
            return None

    def set_interleaved_mode(self):
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            value = controlRegister1 & ~MLX90640_CTRL_MEAS_MODE_MASK
            self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, value)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def set_chess_mode(self):
        try:
            controlRegister1 = self.i2c_driver.read_words(
                self.address, MLX90640_CTRL_REG, 1
            )[0]
            value = controlRegister1 | MLX90640_CTRL_MEAS_MODE_MASK
            self.i2c_driver.write_word(self.address, MLX90640_CTRL_REG, value)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def bad_pixels_correction(self, pixels, to, mode):
        pix = 0
        while pixels[pix] != 0xFFFF:
            line = pixels[pix] >> 5
            column = pixels[pix] - (line << 5)

            if mode == 1:
                if line == 0:
                    if column == 0:
                        to[pixels[pix]] = to[33]
                    elif column == 31:
                        to[pixels[pix]] = to[62]
                    else:
                        to[pixels[pix]] = (
                            to[pixels[pix] + 31] + to[pixels[pix] + 33]
                        ) / 2.0
                elif line == 23:
                    if column == 0:
                        to[pixels[pix]] = to[705]
                    elif column == 31:
                        to[pixels[pix]] = to[734]
                    else:
                        to[pixels[pix]] = (
                            to[pixels[pix] - 33] + to[pixels[pix] - 31]
                        ) / 2.0
                elif column == 0:
                    to[pixels[pix]] = (
                        to[pixels[pix] - 31] + to[pixels[pix] + 33]
                    ) / 2.0
                elif column == 31:
                    to[pixels[pix]] = (
                        to[pixels[pix] - 33] + to[pixels[pix] + 31]
                    ) / 2.0
                else:
                    ap = [
                        to[pixels[pix] - 33],
                        to[pixels[pix] - 31],
                        to[pixels[pix] + 31],
                        to[pixels[pix] + 33],
                    ]
                    to[pixels[pix]] = np.median(ap)
            else:
                if column == 0:
                    to[pixels[pix]] = to[pixels[pix] + 1]
                elif column == 1 or column == 30:
                    to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0
                elif column == 31:
                    to[pixels[pix]] = to[pixels[pix] - 1]
                else:
                    if not self._is_pixel_bad(
                        pixels[pix] - 2
                    ) and not self._is_pixel_bad(pixels[pix] + 2):
                        ap = [
                            to[pixels[pix] + 1] - to[pixels[pix] + 2],
                            to[pixels[pix] - 1] - to[pixels[pix] - 2],
                        ]
                        if abs(ap[0]) > abs(ap[1]):
                            to[pixels[pix]] = to[pixels[pix] - 1] + ap[1]
                        else:
                            to[pixels[pix]] = to[pixels[pix] + 1] + ap[0]
                    else:
                        to[pixels[pix]] = (
                            to[pixels[pix] - 1] + to[pixels[pix] + 1]
                        ) / 2.0
            pix += 1

    def get_frame(
        self,
        emissivity: float = 0.95,
        tr: float = None,
        frame_data: list = None,
        frame_buffer: list = None,
    ):
        if frame_data is None:
            frame_data = self.get_frame_data()

        if tr is None:
            tr = self.get_ta(frame_data=frame_data) - OPENAIR_TA_SHIFT

        if frame_buffer is None:
            frame_buffer = [0] * 768

        sub_page = self.get_sub_page_number(frame_data)
        mode = self.get_cur_mode()
        temperature_frame = self.calculate_to(emissivity, tr, frame_data)
        for pixelNumber in range(768):
            ilPattern = pixelNumber // 32 - (pixelNumber // 64) * 2
            chessPattern = ilPattern ^ (pixelNumber - (pixelNumber // 2) * 2)

            if mode == 0:
                pattern = ilPattern
            else:
                pattern = chessPattern

            if pattern == sub_page:
                frame_buffer[pixelNumber] = temperature_frame[pixelNumber]

        return frame_buffer
