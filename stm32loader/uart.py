# Author: Floris Lambrechts
# GitHub repository: https://github.com/florisla/stm32loader
#
# This file is part of stm32loader.
#
# stm32loader is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 3, or (at your option) any later
# version.
#
# stm32loader is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with stm32loader; see the file LICENSE.  If not see
# <http://www.gnu.org/licenses/>.

"""
Handle RS-232 serial communication through pyserial.

Offer support for toggling RESET and BOOT0.
"""

# not naming this file itself 'serial', becase that name-clashes in Python 2
import serial

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


class SerialConnection:
    """Wrap a serial.Serial connection and toggle reset and boot0."""

    PIN_MAPPING_BOARD = "board"
    PIN_MAPPING_BCM = "bcm"

    # pylint: disable=too-many-instance-attributes

    def __init__(self, serial_port, baud_rate=115200, parity="E"):
        """Construct a SerialConnection (not yet connected)."""
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.parity = parity

        # advertise reset / boot0 toggle capability
        self.can_toggle_reset = True
        self.can_toggle_boot0 = True

        self.swap_rts_dtr = False
        self.reset_active_high = False
        self.boot0_active_high = False

        self.can_use_custom_pins = GPIO is not None
        self._reset_pin = None
        self._boot0_pin = None
        self._pin_mapping = None

        # call connect() to establish connection
        self.serial_connection = None

    @property
    def pin_mapping(self):
        return self._reset_pin

    @pin_mapping.setter
    def pin_mapping(self, value):
        if self.can_use_custom_pins:
            if value == self.PIN_MAPPING_BOARD:
                GPIO.setmode(GPIO.BOARD)
            elif value == self.PIN_MAPPING_BCM:
                GPIO.setmode(GPIO.BCM)
            else:
                raise ValueError("{} is not a valid pin mapping".format(value))
            self._pin_mapping = value
        else:
            raise Exception("GPIO disabled")

    @property
    def reset_pin(self):
        return self._reset_pin

    @reset_pin.setter
    def reset_pin(self, value):
        if self.can_use_custom_pins:
            GPIO.setup(value, GPIO.OUT)
            self._reset_pin = value
        else:
            raise Exception("GPIO disabled")

    @property
    def boot0_pin(self):
        return self._boot0_pin

    @boot0_pin.setter
    def boot0_pin(self, value):
        if self.can_use_custom_pins:
            GPIO.setup(value, GPIO.OUT)
            self._boot0_pin = value
        else:
            raise Exception("GPIO disabled")

    def connect(self):
        """Connect to the RS-232 serial port."""
        self.serial_connection = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            # number of write_data bits
            bytesize=8,
            parity=self.parity,
            stopbits=1,
            # don't enable software flow control
            xonxoff=0,
            # don't enable RTS/CTS flow control
            rtscts=0,
            # set a timeout value, None for waiting forever
            timeout=5,
        )

    def write(self, *args, **kwargs):
        """Write the given data to the serial connection."""
        return self.serial_connection.write(*args, **kwargs)

    def read(self, *args, **kwargs):
        """Read the given amount of bytes from the serial connection."""
        return self.serial_connection.read(*args, **kwargs)

    def enable_reset(self, enable=True):
        """Enable or disable the reset IO line."""
        # reset on the STM32 is active low (0 Volt puts the MCU in reset)
        # but the RS-232 DTR signal is active low by itself, so it
        # inverts this (writing a logical 1 outputs a low voltage, i.e.
        # enables reset)
        level = int(enable)
        if self.reset_active_high:
            level = 1 - level

        if self.enable_pin is not None:
            GPIO.output(self.enable_pin, level)
        elif self.swap_rts_dtr:
            self.serial_connection.setRTS(level)
        else:
            self.serial_connection.setDTR(level)

    def enable_boot0(self, enable=True):
        """Enable or disable the boot0 IO line."""
        level = int(enable)

        # by default, this is active low
        if not self.boot0_active_high:
            level = 1 - level

        if self.boot0_pin is not None:
            GPIO.output(self.boot0_pin, level)
        elif self.swap_rts_dtr:
            self.serial_connection.setDTR(level)
        else:
            self.serial_connection.setRTS(level)
