from machine import Pin, I2C
import time

from locomotion import QuadrupedLocomotion
import light_sensor
from light_sensor import LightNavigator
from obstacle_avoidance import ObstacleAvoidance
from predator_avoidance import Robot as PredatorRobot

TICK_MS = 50


class LocomotionAdapter:
    def __init__(self, robot: QuadrupedLocomotion, interrupt_checks=None):
        self._robot  = robot
        self._checks = interrupt_checks or []

    def check(self) -> bool:
        return True

    def _should_interrupt(self):
        return any(check() for check in self._checks)

    def execute(self):
        interrupted = self._robot.walk_forward_interruptible(
            steps=1,
            should_interrupt=self._should_interrupt
        )
        if interrupted:
            print("[ARB] Locomotion interrupted by higher priority layer")


class PhototaxisAdapter:
    def __init__(self, navigator: LightNavigator, interrupt_checks=None):
        self._nav    = navigator
        self._leak   = 0.0
        self._checks = interrupt_checks or []

    def _should_interrupt(self):
        return any(check() for check in self._checks)

    def check(self) -> bool:
        lux = self._nav.read_lux()
        self._leak = self._nav.compute_leak(lux)
        return abs(self._leak) > 0.05

    def execute(self):
        self._nav.decide_action(self._leak)


class ObstacleAdapter:
    def __init__(self, obstacle: ObstacleAvoidance):
        self._obstacle      = obstacle
        self._left_blocked  = False
        self._right_blocked = False

    def check(self) -> bool:
        o = self._obstacle
        left_d  = o.get_distance(o.trigL, o.echoL)
        right_d = o.get_distance(o.trigR, o.echoR)
        self._left_blocked  = left_d  < o.threshold
        self._right_blocked = right_d < o.threshold
        return self._left_blocked or self._right_blocked

    def execute(self):
        o = self._obstacle
        if self._left_blocked and self._right_blocked:
            o.robot.walk_backward(1)
        elif self._left_blocked:
            o.robot.turn_right(1)
        elif self._right_blocked:
            o.robot.turn_left(1)


class PredatorAdapter:
    def __init__(self, predator: PredatorRobot):
        self._predator    = predator
        self._triggered   = False
        self._last_motion = None
        self._last_speed  = 0

    def check(self) -> bool:
        triggered, motion, speed, L, R = self._predator.check_predator()
        self._triggered   = triggered
        self._last_motion = motion
        self._last_speed  = speed
        return triggered

    def quick_check(self) -> bool:
        L = self._predator.get_distance(self._predator.trigL, self._predator.echoL)
        R = self._predator.get_distance(self._predator.trigR, self._predator.echoR)
        if L is None or R is None:
            return False
        return L < self._predator.LEFT_THRESHOLD or R < self._predator.RIGHT_THRESHOLD

    def execute(self):
        if self._triggered:
            self._predator.act_predator(self._last_motion, self._last_speed)


def init_hardware():
    loco = QuadrupedLocomotion()
    loco.stand(smooth=True)

    # LightNavigator uses a module-level `robot` variable internally,
    # so we patch it to use the shared loco instance instead of the
    # duplicate one created at import time.
    light_sensor.robot = loco
    light_nav = LightNavigator()

    obstacle = ObstacleAvoidance(loco=loco)

    pred_robot = PredatorRobot()
    pred_robot.locomotion = loco

    layer2 = ObstacleAdapter(obstacle)
    layer3 = PredatorAdapter(pred_robot)

    layer1 = PhototaxisAdapter(light_nav, interrupt_checks=[
        layer2._obstacle.quick_check,
        layer3.quick_check,
    ])

    layer0 = LocomotionAdapter(loco, interrupt_checks=[
        layer2._obstacle.quick_check,
        layer3.quick_check,
    ])

    return layer0, layer1, layer2, layer3


def main():
    print("[ARCHITECTURE] Initialising hardware...")
    layer0, layer1, layer2, layer3 = init_hardware()
    print("[ARCHITECTURE] Ready. Entering control loop at 20 Hz.")

    while True:
        predator_active = False
        obstacle_active = False
        light_seeking   = False

        predator_active = layer3.check()

        if not predator_active:
            obstacle_active = layer2.check()

            if not obstacle_active:
                light_seeking = layer1.check()

        if predator_active:
            print("[ARB] Layer 3 — Predator active")
            layer3.execute()

        elif obstacle_active:
            print("[ARB] Layer 2 — Obstacle active")
            layer2.execute()

        elif light_seeking:
            print("[ARB] Layer 1 — Phototaxis active")
            layer1.execute()

        else:
            print("[ARB] Layer 0 — Locomotion (default)")
            layer0.execute()

        time.sleep_ms(TICK_MS)


if __name__ == "__main__":
    main()