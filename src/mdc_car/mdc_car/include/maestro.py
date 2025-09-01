import serial
from typing import Sequence
"""Module for controlling a Polulu Micro Maestro 6-Channel USB Server Controller.


This module communciates with the controller using the compact protocol. This
means that there must be exactly one controller connected via USB.


The controller should be configured to be in USB_DUAL_PORT mode so that two way
communication can happen over a single serial port.


Serial command reference can be found at https://www.pololu.com/docs/0J40/5.e
"""
class Maestro:
   def __init__(self, path: str, baud_rate: int):
       """Creates a new Maestro controller.


       Args:
           path: The serial path the controller is connected to (e.g. "/dev/ttyACM0").
           baud_rate: The baud rate that the controller expects.
       """
       self._serial = serial.Serial(path, baud_rate)

   def _send(self, cmd: Sequence[int]) -> None:
       self._serial.write(bytes(cmd))

   def _recv(self) -> int:
       lo = int(self._serial.read()[0])
       hi = int(self._serial.read()[0])
       return (hi << 8) + lo

   def _send_set_cmd(self, cmd_id: int, channel: int, target: int) -> None:
       """Shared helper method for all set_* commands."""
       channel = channel & 0xff
       target = target & 0x3fff
       cmd = [
       cmd_id,
       channel,
       target & 0x7f,
       (target >> 7) & 0x7f,
       ]
       self._send(cmd)

   def close(self) -> None:
       """Closes the connection to the controller."""
       self._serial.close()

   def set_target(self, channel: int, target: int) -> None:
       """Sets the target for a given channel.

       If the channel is configured as a servo, then the target represents the
       pulse width to transmit in units of quarter-microseconds. A target value of
       0 tells the Maestro to stop sending pulses to the servo.


       If the channel is configured as a digital output, values less than 6000 tell
       the Maestro to drive the line low, while values of 6000 or greater tell the
       Maestro to drive the line high.
       """
       self._send_set_cmd(0x84, channel, target)

   def set_speed(self, channel: int, target: int) -> None:
       """Sets the speed limit at which a servo channel's output value changes.


       The speed limit is given in units of 0.25us/10ms.
       """
       self._send_set_cmd(0x87, channel, target)

   def set_acceleration(self, channel: int, target: int) -> None:
       """Sets the acceleration limit of a servo channel's output.


       The acceleration limit is given in units of 0.25us/10ms/80ms.
       """
       self._send_set_cmd(0x89, channel, target)

   def get_position(self, channel: int) -> int:
       """Returns the current position of a channel.


       If the channel is specified as a servo, the returned value represents the
       current pulse width that the Maestro is transmitting on the channel.


       If the channel is configured as a digital output, a position value less
       than 6000 means the Maestro is driving the line low, while a position value
       of 6000 or greater means the Maestro is driving the line high.
       """
       channel = channel & 0xff
       self._send([0x90, channel])
       return self._recv()

   def get_errors(self) -> int:
       """Returns the errors that the Maestro has detected."""
       self._send([0xa1])
       return self._recv()

   def go_home(self) -> None:
       """Sends all servos and outputs to their home positions."""
       self._send([0xa2])