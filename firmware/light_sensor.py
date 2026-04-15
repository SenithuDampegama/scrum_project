from machine import Pin, SoftI2C
from bh1750 import BH1750
from locomotion import QuadrupedLocomotion
import time
import random
import math
robot = QuadrupedLocomotion()
actions = {
    
    1: lambda: robot.walk_backward(1),
    2: lambda: robot.turn_left(1),
    3: lambda: robot.turn_right(1)
}


class LightNavigator:
    def __init__(self, scl_pin=27, sda_pin=26, addr=0x23,
                 alpha=1, beta=0.3, gamma=0.2):

        # Initialize I2C and BH1750 sensor
        self.i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=400000)
        self.sensor = BH1750(bus=self.i2c, addr=addr)

        # Algorithm parameters
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

        # Initial readings
        self.previous_lux = self.sensor.luminance(BH1750.CONT_HIRES_1)
        self.S_previous = 0

        print("Initial lux: {:.2f}".format(self.previous_lux))

    def read_lux(self):
        #"""Read current luminance"""
        return self.sensor.luminance(BH1750.CONT_HIRES_1)

    def compute_leak(self, lux):
        #"""Compute leak value based on light change"""
        change = lux - self.previous_lux
        C = math.log(abs(change) + 1)

        F = self.alpha * C
        S = self.beta * self.S_previous + self.gamma * C

        leak = F - S

        # Update internal state
        self.S_previous = S
        self.previous_lux = lux

        return leak

    def stop(self):
        #"""Replace this with actual motor control"""
        robot.stand()
        print("move forward")

    def goback(self):
        #"""Replace this with actual motor control"""
        robot.walk_backward(steps=1)
        print("move forward")
    def move_forward(self):
        #"""Replace this with actual motor control"""
        robot.walk_forward(steps=1)
        print("move forward")

    def random_turn(self):
        #"""Replace this with actual motor control"""
        choice = random.randint(1, 3)  # pick number
        actions[choice]()
        print("random turn")

    def decide_action(self, leak):
        #""Decide movement based on leak""
        if leak > 0:            
            self.move_forward()
        else:
            self.random_turn()

    def run(self, delay=1):
        #"""Main loop"""
        while True:
            lux = self.read_lux()
            if lux < 320:
                    print("Luminance: {:.2f} lux".format(lux))

                    leak = self.compute_leak(lux)
                    self.decide_action(leak)
            elif lux > 450:
                print("Luminance: {:.2f} lux".format(lux))
                self.goback()
            else:
                print("Stopping due to stable light: {:.2f} lux".format(lux))
                self.stop()
                

            #time.sleep(50)

          


# ===== MAIN PROGRAM =====
if __name__ == "__main__":
    navigator = LightNavigator()
    navigator.run()