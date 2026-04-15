from machine import Pin, time_pulse_us
import time
from locomotion import QuadrupedLocomotion

class ObstacleAvoidance:
    def __init__(self, loco=None):
        # Power pins
        self.powerL = Pin(6, Pin.OUT)
        self.powerR = Pin(7, Pin.OUT)
        self.powerL.high()
        self.powerR.high()
        # Ultrasonic pins
        self.trigL = Pin(3, Pin.OUT)
        self.echoL = Pin(2, Pin.IN)

        self.trigR = Pin(17, Pin.OUT)
        self.echoR = Pin(16, Pin.IN)
        
        #time.sleep_ms(200)  # give pins time to settle after reinit

        self.robot = loco if loco is not None else QuadrupedLocomotion()
        self.robot.stand(smooth=True)
        self.threshold = 20
        
    def get_distance(self, trig, echo):
        trig.low()
        time.sleep_us(2)
        trig.high()
        time.sleep_us(10)
        trig.low()

        try:
            duration = time_pulse_us(echo, 1, 5000)
            if duration < 0:
                return 100
            return (duration * 0.0343) / 2
        except:
            return 100
        
    def quick_check(self) -> bool:
        """
        Fast single-sample check for use as interrupt callback
        during locomotion. No averaging — just one read per sensor.
        Returns True if either side is blocked.
        """
        left_d  = self.get_distance(self.trigL, self.echoL)
        right_d = self.get_distance(self.trigR, self.echoR)
        return left_d < self.threshold or right_d < self.threshold

    def run(self):
        left_d = self.get_distance(self.trigL, self.echoL)
        right_d = self.get_distance(self.trigR, self.echoR)

        print("Left: {:.1f} cm | Right: {:.1f} cm".format(left_d, right_d))

        left_blocked = left_d < self.threshold
        right_blocked = right_d < self.threshold

        if left_blocked and right_blocked:
            print(">> ACTION: BACK")
            self.robot.walk_backward(1)

        elif left_blocked:
            print(">> ACTION: TURN RIGHT")
            self.robot.turn_right(1)

        elif right_blocked:
            print(">> ACTION: TURN LEFT")
            self.robot.turn_left(1)

        else:
            print(">> ACTION: FORWARD")
            self.robot.walk_forward(1)

        print()

if __name__ == "__main__":
    obstacle_system = ObstacleAvoidance()

    print("OBSTACLE AVOIDANCE MODE\n")

    while True:
        obstacle_system.run()
        time.sleep(0.2)