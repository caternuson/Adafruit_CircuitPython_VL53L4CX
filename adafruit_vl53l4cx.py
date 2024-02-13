# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_vl53l4cx`
================================================================================

CircuitPython helper library for the VL53L4CX ToF Sensor


* Author(s): Carter Nelson

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
import struct
from adafruit_bus_device import i2c_device
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CX.git"

# registers
_VL53L4CX_IDENTIFICATION_MODEL_ID = const(0x010F)
_VL53L4CX_GPIO_HV_MUX_CTRL = const(0x0030)
_VL53L4CX_GPIO_TIO_HV_STATUS = const(0x0031)
_VL53L4CX_FIRMWARE_SYSTEM_STATUS = const(0x00E5)
_VL53L4CX_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_VL53L4CX_SYSTEM_START = const(0x0087)
_VL53L4CX_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = const(0x0096)

# misc
_VL53L4CX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT = const(2011)

class VL53L4CX:
    """Driver for the VL53L4CX distance sensor."""

    def __init__(self, i2c, address=41):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        model_id, module_type = self.model_info
        if model_id != 0xEB or module_type != 0xAA:
            raise RuntimeError("Wrong sensor ID or type!")
        self._ranging = False
        self._sensor_init()

    def _sensor_init(self):
        # pylint: disable=line-too-long
        init_seq = (
            b"\x02"  # 0x02
            b"\x10"  # 0x03
            b"\x00"  # 0x04
            b"\x2B"  # 0x05
            b"\xBC"  # 0x06
            b"\xF5"  # 0x07
            b"\x81"  # 0x08
            b"\x80"  # 0x09
            b"\x07"  # 0x0A
            b"\x94"  # 0x0B
            b"\x00"  # 0x0C
            b"\xEF"  # 0x0D
            b"\xFF"  # 0x0E
            b"\xFE"  # 0x0F
            b"\xAF"  # 0x10
            b"\xFF"  # 0x11
            b"\x0F"  # 0x12
            b"\x00"  # 0x13
            b"\x0F"  # 0x14
            b"\x02"  # 0x15
            b"\x00"  # 0x16
            b"\x00"  # 0x17
            b"\x00"  # 0x18
            b"\x00"  # 0x19
            b"\x00"  # 0x1A
            b"\x00"  # 0x1B
            b"\x00"  # 0x1C
            b"\x00"  # 0x1D
            b"\x00"  # 0x1E
            b"\x00"  # 0x1F
            b"\x00"  # 0x20
            b"\x00"  # 0x21
            b"\x00"  # 0x22
            b"\x00"  # 0x23
            b"\x0A"  # 0x24
            b"\x00"  # 0x25
            b"\x00"  # 0x26
            b"\x00"  # 0x27
            b"\x00"  # 0x28
            b"\x00"  # 0x29
            b"\x00"  # 0x2A
            b"\x00"  # 0x2B
            b"\x00"  # 0x2C
            b"\x00"  # 0x2D
            b"\x00"  # 0x2E
            b"\x00"  # 0x2F
            b"\x11"  # 0x30
            b"\x02"  # 0x31
            b"\x00"  # 0x32
            b"\x02"  # 0x33
            b"\x08"  # 0x34
            b"\x00"  # 0x35
            b"\x07"  # 0x36
            b"\x11"  # 0x37
            b"\x22"  # 0x38
            b"\x10"  # 0x39
            b"\x12"  # 0x3A
            b"\x32"  # 0x3B
            b"\x07"  # 0x3C
            b"\x11"  # 0x3D
            b"\x22"  # 0x3E
            b"\x10"  # 0x3F
            b"\x02"  # 0x40
            b"\x21"  # 0x41
            b"\x03"  # 0x42
            b"\x00"  # 0x43
            b"\x00"  # 0x44
            b"\x00"  # 0x45
            b"\x20"  # 0x46
            b"\x05"  # 0x47
            b"\x00"  # 0x48
            b"\x00"  # 0x49
            b"\x02"  # 0x4A
            b"\xFF"  # 0x4B
            b"\x21"  # 0x4C
            b"\x00"  # 0x4D
            b"\x00"  # 0x4E
            b"\x01"  # 0x4F
            b"\x00"  # 0x50
            b"\x00"  # 0x51
            b"\x00"  # 0x52
            b"\x00"  # 0x53
            b"\x8C"  # 0x54
            b"\x00"  # 0x55
            b"\x00"  # 0x56
            b"\x38"  # 0x57
            b"\xFF"  # 0x58
            b"\x01"  # 0x59
            b"\x00"  # 0x5A
            b"\x36"  # 0x5B
            b"\x00"  # 0x5C
            b"\x28"  # 0x5D
            b"\x00"  # 0x5E
            b"\x8F"  # 0x5F
            b"\x05"  # 0x60
            b"\x00"  # 0x61
            b"\x6B"  # 0x62
            b"\x07"  # 0x63
            b"\x07"  # 0x64
            b"\x11"  # 0x65
            b"\x22"  # 0x66
            b"\x10"  # 0x67
            b"\x12"  # 0x68
            b"\x32"  # 0x69
            b"\x00"  # 0x6A
            b"\x00"  # 0x6B
            b"\x00"  # 0x6C
            b"\x00"  # 0x6D
            b"\x8C"  # 0x6E
            b"\xA0"  # 0x6F
            b"\x00"  # 0x70
            b"\x02"  # 0x71
            b"\xFF"  # 0x72
            b"\xFF"  # 0x73
            b"\xFF"  # 0x74
            b"\xFF"  # 0x75
            b"\x00"  # 0x76
            b"\x02"  # 0x77
            b"\x05"  # 0x78
            b"\x07"  # 0x79
            b"\x05"  # 0x7A
            b"\x06"  # 0x7B
            b"\x02"  # 0x7C
            b"\x00"  # 0x7D
            b"\x02"  # 0x7E
            b"\xC7"  # 0x7F
            b"\xFF"  # 0x80
            b"\x9B"  # 0x81
            b"\x02"  # 0x82
            b"\x01"  # 0x83
            b"\x00"  # 0x84
            b"\x01"  # 0x85
            b"\x01"  # 0x86
            b"\x26"  # 0x87

        )
        self._wait_for_boot()
        self._write_register(0x0002, init_seq)
        #self.clear_interrupt()
        #self.stop_ranging()

    @property
    def model_info(self):
        """A 2 tuple of Model ID and Module Type."""
        info = self._read_register(_VL53L4CX_IDENTIFICATION_MODEL_ID, 2)
        return info[0], info[1]  # Model ID, Module Type

    @property
    def distance(self):
        """The distance in units of centimeters."""

        #dist = self._read_register(_VL53L4CX_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 2)
        #dist = struct.unpack(">H", dist)[0]
        #dist *= _VL53L4CX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT
        #dist += 0x0400
        #dist /= 0x0800
        #return dist

        data = self._read_register(136, 83);



    def start_ranging(self):
        """Starts ranging operation."""
        self._write_register(_VL53L4CX_SYSTEM_START, b"\x26")
        # wait for data ready
        timed_out = True
        for _ in range(1000):
            if self.data_ready:
                timed_out = False
                break
            time.sleep(0.001)
        if timed_out:
            raise TimeoutError("Time out waiting for data ready.")

        self.clear_interrupt()
        self._ranging = True

    def stop_ranging(self):
        """Stops ranging operation."""
        self._write_register(_VL53L4CX_SYSTEM_START, b"\x00")
        self._ranging = False

    def clear_interrupt(self):
        """Clears new data interrupt."""
        self._write_register(_VL53L4CX_SYSTEM_INTERRUPT_CLEAR, b"\x01")

    @property
    def data_ready(self):
        """Returns true if new data is ready, otherwise false."""
        # https://github.com/stm32duino/VL53L4CX/blob/36487bb4872f102766bfc415089f62c0c626f8bb/src/vl53l4cx_wait.cpp#L225-L236
        # bit 0 of `VL53L4CX_GPIO__TIO_HV_STATUS` == interrupt polarity -> ready
        if (
            self._read_register(_VL53L4CX_GPIO_TIO_HV_STATUS)[0] & 0x01
            == self._interrupt_polarity
        ):
            return True
        return False

    @property
    def _interrupt_polarity(self):
        # https://github.com/stm32duino/VL53L4CX/blob/36487bb4872f102766bfc415089f62c0c626f8bb/src/vl53l4cx_wait.cpp#L213-L220
        # bit 5 of `VL53L4CX_GPIO_HV_MUX__CTRL` 0 is HIGH, 1 is LOW
        int_pol = self._read_register(_VL53L4CX_GPIO_HV_MUX_CTRL)[0] & 0x10
        int_pol = (int_pol >> 4) & 0x01
        return 0 if int_pol else 1

    def _wait_for_boot(self):
        # https://github.com/stm32duino/VL53L4CX/blob/36487bb4872f102766bfc415089f62c0c626f8bb/src/vl53l4cx_wait.cpp#L258
        # bit 0 of `VL53L4CX_FIRMWARE__SYSTEM_STATUS` is 1
        for _ in range(1000):
            if self._read_register(_VL53L4CX_FIRMWARE_SYSTEM_STATUS)[0] & 0x01 == 0x01:
                return
            time.sleep(0.001)
        raise TimeoutError("Time out waiting for system boot.")

    def _write_register(self, address, data, length=None):
        if length is None:
            length = len(data)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address) + data[:length])

    def _read_register(self, address, length=1):
        data = bytearray(length)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address))
            i2c.readinto(data)
        return data