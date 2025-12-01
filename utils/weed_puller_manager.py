# -*- coding: utf-8 -*-
"""
Created on Thu May 29 05:56:41 2025

@author: mehraj
"""

import RPi.GPIO as GPIO
from time import sleep
import serial
import threading

class StepperMotor:
  def __init__(self, dir_pin=19, step_pin=24, mode=GPIO.BOARD,
               step_angle=1.8, pitch=4, microstepping=0.5, limit_switch_pin=29):
    self.DIR = dir_pin
    self.STEP = step_pin
    self.switch_pin = limit_switch_pin 
    self.CW = 1
    self.CCW = 0

    self.position_px = 0.0  # current linear position
    self.steps_per_rev = int(360 / step_angle)
    self.pitch = pitch
    self.microstepping = microstepping
    self.steps_per_mm = (self.steps_per_rev * microstepping) / pitch

    GPIO.setmode(mode)
    GPIO.setup(self.DIR, GPIO.OUT)
    GPIO.setup(self.STEP, GPIO.OUT)
    GPIO.output(self.DIR, self.CW)
    self.home()

  def rotate(self, steps, direction=1, delay=0.001):
    GPIO.setup(self.switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.output(self.DIR, direction)
    for _ in range(steps):
      if GPIO.input(self.switch_pin) == GPIO.LOW:
          print("Limit switch pressed.....")
          self.position_px = 0
          break  
      GPIO.output(self.STEP, GPIO.HIGH)
      sleep(delay)
      GPIO.output(self.STEP, GPIO.LOW)
      sleep(delay)
    
  def home(self, step_delay=0.001):
    """
    Moves the stepper slowly in reverse until the limit switch is pressed.
    Once pressed, sets position to 0.
    """
    GPIO.setup(self.switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("Starting homing...")

    GPIO.output(self.DIR, self.CCW)  # Move in reverse toward the switch

    while GPIO.input(self.switch_pin) == GPIO.HIGH:
        GPIO.output(self.STEP, GPIO.HIGH)
        sleep(step_delay)
        GPIO.output(self.STEP, GPIO.LOW)
        sleep(step_delay)
    GPIO.output(self.DIR, self.CW)
    for _ in range(20):
        GPIO.output(self.STEP, GPIO.HIGH)
        sleep(step_delay)
        GPIO.output(self.STEP, GPIO.LOW)
        sleep(step_delay)
    # Switch is hit
    print("Limit switch pressed. Homing complete.")
    self.position_px = 0  # Set current position as zero

  def move_to_position(self, target_px, pixel_per_mm, delay=0.001):
    delta_px = target_px - self.position_px
    delta_mm = delta_px / pixel_per_mm
    direction = self.CW if delta_px >= 0 else self.CCW
    steps = abs(delta_mm * self.steps_per_mm)

    print(f"Moving from {self.position_px:.2f}px to {target_px:.2f}px ({steps:.0f} steps)")
    self.rotate(steps=int(steps), direction=direction,delay=delay)
    self.position_px = target_px  # update position
    movement_time = 2 * steps * delay
    print(f"Waiting {movement_time:.2f}s for claw positioning to complete...")
    return movement_time

  def cleanup(self):
    print("Cleaning up GPIO...")
    GPIO.cleanup([self.DIR,self.STEP])

# RhinoMotor Class


class RhinoMotor:
    def __init__(self, port='/dev/ttyS0',baudrate = 9600, timeout =1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(self.port, baudrate=self.baudrate,timeout = self.timeout)
        sleep(2) # Allow connection to stabilize
        print(f"Connected to Rhino Motor on {self.port} at {self.baudrate} baud.")

    def set_speed(self, speed):
        """
        Sends speed command to motor.
        Speed can be from -255 (full reverse) to 255 (full forward).
        """
        if not self.ser.is_open:
            print("Error: Serial connection is not open.")
            return
        if -255 <= speed <= 255:
            command = f"S{speed}\r".encode()
            self.ser.write(command)
            # print(f"Set speed to: {speed}")
        else:
            print("Speed must be between -255 and 255.")
    def soft_start(self, final_speed=100, step =5,delay=0.1):
        """
        Gradually starts the motor and then fully enables it.
        """
        speed = 0
        if final_speed > 0:
          while final_speed > speed:
            self.set_speed(speed)
            speed += step
            sleep(delay)
        elif final_speed < 0:
          while final_speed < speed:
            self.set_speed(speed)
            speed -= step
            sleep(delay)
        self.set_speed(final_speed)
        print("Motor started.")

    def soft_stop(self, initial_speed=100, step =10,delay=0.2):
        """
        Gradually slows down the motor and then fully disables it.
        """
        speed = initial_speed
        if speed > 0:
          while speed > 0:
            self.set_speed(speed)
            speed -= step
            sleep(delay)
        elif speed < 0:
          while speed < 0:
            self.set_speed(speed)
            speed += step
            sleep(delay)
        self.set_speed(0)
        print("Motor stopped.")

    def close(self):
        """
        Closes the serial connection.
        """
        self.ser.close()
        print("Serial connection closed.")

# Linear Actuator Class

class LinearActuator:
  def __init__(self, RPWN_pin=12, LPWN_pin=35, REN_pin=16
               , LEN_pin=18, mode = GPIO.BOARD):
    self.RPWN = RPWN_pin
    self.LPWN = LPWN_pin
    self.REN = REN_pin
    self.LEN = LEN_pin
    GPIO.setmode(mode)
    GPIO.setup(self.RPWN, GPIO.OUT)
    GPIO.setup(self.LPWN, GPIO.OUT)
    GPIO.setup(self.REN, GPIO.OUT)
    GPIO.setup(self.LEN, GPIO.OUT)

    self.rpwm = GPIO.PWM(self.RPWN, 20000)
    self.lpwm = GPIO.PWM(self.LPWN, 20000)
    self.rpwm.start(0)
    self.lpwm.start(0)
    self.Actuate(-80,14)

  def Actuate(self, A_speed=100, A_time = 41):
    '''
    A_speed = actuating speed (>0 Extending; <0 Retracting)
    A_time = actuating time
    '''
    if A_speed > 0:
      self.fspeed = A_speed
      self.bspeed = 0
      print("Extending...")
    elif A_speed < 0:
      self.fspeed = 0
      self.bspeed = -A_speed
      print("Retracting...")
    else:
      self.fspeed = 0
      self.bspeed = 0
      print("Stopped.")
    GPIO.output(self.REN, GPIO.HIGH)
    GPIO.output(self.LEN, GPIO.HIGH)
    self.rpwm.ChangeDutyCycle(self.fspeed)
    self.lpwm.ChangeDutyCycle(self.bspeed)
    sleep(A_time)
    self.Stop()
  def Stop(self):
    self.rpwm.ChangeDutyCycle(0)
    self.lpwm.ChangeDutyCycle(0)
    GPIO.output(self.REN, GPIO.LOW)
    GPIO.output(self.LEN, GPIO.LOW)
  def cleanup(self):
    print("Cleaning up...")
    self.rpwm.stop()
    self.lpwm.stop()
    GPIO.cleanup([self.RPWN,self.LPWN, self.REN, self.LEN])


class Buzzer:
    def __init__(self, buzz_pin=11, mode=GPIO.BOARD):
        self.buzz_pin = buzz_pin
        GPIO.setmode(mode)
        GPIO.setup(self.buzz_pin, GPIO.OUT)
        self._beeping = False
        self._thread = None

    def _beep_loop(self):
        while self._beeping:
            GPIO.output(self.buzz_pin, GPIO.HIGH)
            sleep(0.1)
            GPIO.output(self.buzz_pin, GPIO.LOW)
            sleep(0.5)

    def start_beep(self):
        if not self._beeping:
            self._beeping = True
            self._thread = threading.Thread(target=self._beep_loop)
            self._thread.start()

    def stop_beep(self):
        self._beeping = False
        if self._thread:
            self._thread.join()

    def cleanup(self):
        self.stop_beep()
        GPIO.cleanup([self.buzz_pin])



class WeedPuller:
    def __init__(self, dc_motor_speed=255):
        self.busy = False  # Flag to track if system is running
        self.motor = StepperMotor()
        self.actuator = LinearActuator()
        self.dc_motor = RhinoMotor()
        self.dc_motor_speed = dc_motor_speed
        self.buzzer = Buzzer()
        print("Weed plucking system initialized.")

    def run_sequence(self, weed_location_px, pixel_per_mm):
        if self.busy:
            print("System busy.Inside run sequence")
            return
        self.busy = True
        self.buzzer.start_beep()
        
        try:
            print('inside weed puller')
            # Move to weed location
            movement_time = self.motor.move_to_position(weed_location_px, pixel_per_mm)
            # Extend actuator (plucking action)
            self.dc_motor.soft_start(final_speed=self.dc_motor_speed)
            self.actuator.Actuate(-80, 15)
            self.dc_motor.soft_stop(initial_speed=self.dc_motor_speed)
            # Retract actuator
            print("Retracting actuator...")
            self.actuator.Actuate(80, 15)

        except KeyboardInterrupt:
            print("Process interrupted by user. Stopping DC motor and homing claw...")
            self.dc_motor.set_speed(0)
            #self.actuator.Actuate(-80, 21)
        finally:
            self.busy = False
            self.dc_motor.set_speed(0)
            self.buzzer.stop_beep()

    def cleanup(self):
        # self.actuator.Actuate(-80, 21)
        print("Cleaning up system...")
        self.motor.cleanup()
        self.dc_motor.close()
        self.actuator.Actuate(80, 14)
        self.actuator.cleanup()
        self.buzzer.cleanup()

    def __del__(self):
        self.cleanup()

