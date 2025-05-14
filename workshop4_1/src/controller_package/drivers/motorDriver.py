#!/usr/bin/env python2

from math import fabs, floor
import Jetson.GPIO as GPIO
import hat


class DaguWheelsDriver:
    """Class handling communication with motors."""

    LEFT_MOTOR_MIN_PWM = 60  #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM = 255  #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60  #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255  #: Maximum speed for right motor
    SPEED_TOLERANCE = 1.0e-2  #: Speed tolerance level

    def __init__(self):
        self.DTHAT = hat.HATv3()
        self.leftMotor = self.DTHAT.get_motor(1, "left")
        self.rightMotor = self.DTHAT.get_motor(2, "right")
        # print out some stats
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self._pwm_update()

    def set_wheels_speed(self, left, right):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
        """
        self.leftSpeed = left
        self.rightSpeed = right
        self._pwm_update()

    def _pwm_value(self, v, min_pwm, max_pwm):
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def _pwm_update(self):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.
        """
        vl = self.leftSpeed
        vr = self.rightSpeed

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode = hat.MotorDirection.RELEASE
        rightMotorMode = hat.MotorDirection.RELEASE

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = hat.MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = hat.MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = hat.MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = hat.MotorDirection.BACKWARD

        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)

    def close(self):
        """Releases the motors and cleans up resources."""
        self.leftMotor.set(hat.MotorDirection.RELEASE)
        self.rightMotor.set(hat.MotorDirection.RELEASE)
        GPIO.cleanup()
