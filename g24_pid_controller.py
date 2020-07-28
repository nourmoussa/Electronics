'''
-------------------------------------------------------
Group 24
This script handles the PID Controller of the Segway
-------------------------------------------------------
'''

class PIDC:
    def __init__(self, Kp, Kd, Ki, theta_0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.baseTarget = theta_0 # save initial calibration value
        self.target = theta_0 # initially set the target to the base calibrated value
        self.limit = 2 # prevent the tilt from being more than 2 degrees from balanced
        self.integrator = 0

    def target_inc(self, increment):
        if abs(self.target + increment) < self.limit:
            self.target += increment
            self.integrator = 0

    def target_dec(self, decrement):
        if abs(self.target - decrement) < self.limit:
            self.target -= decrement
            self.integrator = 0

    def target_set(self, value):
        if abs(value) <= self.limit:
            self.target = self.baseTarget + value
            self.integrator = 0

    def target_reset(self):
        self.target = self.baseTarget
        self.integrator = 0

    def get_pwm(self, pitch, pitch_dot):
        pError = pitch - self.target # calculate error from target

        output = self.Kp * pError + self.Kd * pitch_dot + self.Ki * self.integrator

        self.integrator += pError

        # limit output to + - 100 PWM value
        if output > 100:
            output = 100
        elif output < -100:
            output = -100

        return output
