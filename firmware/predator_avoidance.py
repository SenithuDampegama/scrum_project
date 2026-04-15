from machine import Pin, time_pulse_us, I2C
from locomotion import QuadrupedLocomotion
import time
import ustruct


class Robot:
    def __init__(self):

        self.locomotion = QuadrupedLocomotion()

        self.powerL = Pin(6, Pin.OUT)
        self.powerR = Pin(7, Pin.OUT)
        self.powerL.high()
        self.powerR.high()

        self.trigL = Pin(3, Pin.OUT)
        self.echoL = Pin(2, Pin.IN)

        self.trigR = Pin(17, Pin.OUT)
        self.echoR = Pin(16, Pin.IN)

        # I2C + PCA9685 SETUP
        self.i2c  = I2C(0, scl=Pin(5), sda=Pin(4))
        self.addr = 0x40

        self.MODE1    = 0x00
        self.PRESCALE = 0xFE
        self.LED0_ON_L = 0x06

        self.write(self.MODE1, 0x00)
        time.sleep_ms(10)

        self.set_pwm_freq(50)

        # Servo starting position
        self.set_angle(25)

        self.LEFT_THRESHOLD       = 50
        self.RIGHT_THRESHOLD      = 50
        self.FRONT_DIFF_THRESHOLD = 20
        self.SPEED_THRESHOLD      = 150
        self.COOLDOWN             = 500

        self.prevDist        = None
        self.prevTime        = time.ticks_us()
        self.lastTriggerTime = 0

    def write(self, reg, value):
        try:
            self.i2c.writeto_mem(self.addr, reg, bytes([value]))
        except Exception as e:
            print("I2C write error:", e)

    def set_pwm(self, channel, on, off):
        reg  = self.LED0_ON_L + 4 * channel
        data = ustruct.pack("<HH", on, off)
        try:
            self.i2c.writeto_mem(self.addr, reg, data)
        except Exception as e:
            print("PWM error:", e)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)

        old_mode = self.i2c.readfrom_mem(self.addr, self.MODE1, 1)[0]
        new_mode = (old_mode & 0x7F) | 0x10

        self.write(self.MODE1, new_mode)
        self.write(self.PRESCALE, prescale)
        self.write(self.MODE1, old_mode)

        time.sleep_ms(5)
        self.write(self.MODE1, old_mode | 0xA1)

    def set_angle(self, angle):
        angle     = max(0, min(180, angle))
        pulse_min = 102
        pulse_max = 512
        pulse     = int(pulse_min + (angle / 180) * (pulse_max - pulse_min))
        self.set_pwm(0, 0, pulse)

    def flap_wings(self, times=5):
        for _ in range(times):
            self.set_angle(25)
            time.sleep(0.2)
            self.set_angle(40)
            time.sleep(0.2)
        self.set_angle(25)

    def check_predator(self):
        """Original method — reads sensors directly. Used in standalone mode."""
        L = self.get_avg_distance(self.trigL, self.echoL)
        time.sleep_ms(30)
        R = self.get_avg_distance(self.trigR, self.echoR)

        avg   = (L + R) / 2
        speed = self.compute_speed(avg)

        left  = L < self.LEFT_THRESHOLD
        right = R < self.RIGHT_THRESHOLD
        diff  = abs(L - R)

        if left and right and diff < self.FRONT_DIFF_THRESHOLD:
            motion = "FRONT"
        elif left:
            motion = "LEFT"
        elif right:
            motion = "RIGHT"
        else:
            return False, None, speed, L, R

        now = time.ticks_ms()
        if speed > self.SPEED_THRESHOLD and \
           time.ticks_diff(now, self.lastTriggerTime) > self.COOLDOWN:
            return True, motion, speed, L, R

        return False, None, speed, L, R

    def check_predator_from(self, L, R):
        """
        Architecture mode — accepts pre-read distances from SharedUltrasonicReader
        instead of reading sensors itself. Avoids sensor conflicts with obstacle layer.
        """
        avg   = (L + R) / 2
        speed = self.compute_speed(avg)

        left  = L < self.LEFT_THRESHOLD
        right = R < self.RIGHT_THRESHOLD
        diff  = abs(L - R)

        if left and right and diff < self.FRONT_DIFF_THRESHOLD:
            motion = "FRONT"
        elif left:
            motion = "LEFT"
        elif right:
            motion = "RIGHT"
        else:
            return False, None, speed

        now = time.ticks_ms()
        if speed > self.SPEED_THRESHOLD and \
           time.ticks_diff(now, self.lastTriggerTime) > self.COOLDOWN:
            return True, motion, speed

        return False, None, speed

    def get_distance(self, trig, echo):
        trig.low()
        time.sleep_us(2)
        trig.high()
        time.sleep_us(10)
        trig.low()

        duration = time_pulse_us(echo, 1, 30000)

        if duration < 0:
            return None

        return (duration * 0.0343) / 2

    def get_avg_distance(self, trig, echo, samples=5):
        vals = []
        for _ in range(samples):
            d = self.get_distance(trig, echo)
            if d is not None:
                vals.append(d)
            time.sleep_ms(5)

        if not vals:
            return 999  # fallback if all readings fail

        return sum(vals) / len(vals)

    def compute_speed(self, avgDist):
        now   = time.ticks_us()
        speed = 0

        if self.prevDist is not None:
            dt = time.ticks_diff(now, self.prevTime) / 1_000_000
            if dt > 0:
                speed = (self.prevDist - avgDist) / dt
                speed = max(min(speed, 300), -300)

        self.prevDist = avgDist
        self.prevTime = now

        return speed

    def act_predator(self, motion, speed):
        print("Predator detected:", motion, "| Speed:", speed)
        self.set_angle(25)
        self.locomotion.stand()
        self.flap_wings(5)
        self.lastTriggerTime = time.ticks_ms()

    def run(self):
        """Standalone loop — used when running predator_avoidance.py directly."""
        while True:
            triggered, motion, speed, L, R = self.check_predator()
            print("Motion:", motion, "| Speed:", speed, " |Left: ", L, " |Right: ", R)
            if triggered:
                self.act_predator(motion, speed)
            time.sleep_ms(50)


if __name__ == "__main__":
    robot = Robot()
    robot.run()

