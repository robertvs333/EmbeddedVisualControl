#!/usr/bin/env python2

from enum import IntEnum
import Jetson.GPIO as GPIO
import time

class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1

def current_milli_time():
    return int(round(time.time() * 1000))

class WheelEncoderDriver:
    """Class handling communication with a wheel encoder."""

    def __init__(self, gpio_pin):
        # valid gpio_pin
        if not 1 <= gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
            
        self._gpio_pin = gpio_pin
        
        current_mode = GPIO.getmode()
        if current_mode is None:
            # If nothing else set it, default to BOARD (since you use 12 and 35)
            GPIO.setmode(GPIO.BOARD)
        
        # setup pin
        GPIO.setup(gpio_pin, GPIO.IN)
        GPIO.add_event_detect(gpio_pin, GPIO.RISING, callback=self._cb)
        
        self.I = 0
        self.D = 0
        self._last_meas = 0
        self._last_time = current_milli_time() # Initialize with actual time
        self._ticks = 0
        self._direction = WheelDirection.FORWARD

    def get_direction(self):
        return self._direction

    def set_direction(self, direction):
        self._direction = direction

    def _cb(self, _):
        tim = current_milli_time()
        self._ticks += self._direction.value
        
        self.I += self._direction.value * (tim - self._last_time)

        self._last_time = tim

    def shutdown(self):
        GPIO.remove_event_detect(self._gpio_pin)
