from machine import Pin, I2C
import time

from locomotion import QuadrupedLocomotion
import light_sensor
from light_sensor import LightNavigator
from obstacle_avoidance import ObstacleAvoidance
from predator_avoidance import Robot as PredatorRobot

TICK_MS = 50


# --- Shared ultrasonic reader using predator's get_distance (30000us timeout) ---
class SharedUltrasonicReader:
    def __init__(self, obstacle: ObstacleAvoidance, predator: PredatorRobot):
        self._obstacle = obstacle
        self._predator = predator
        self.left_d  = 999
        self.right_d = 999
        self.classification = "CLEAR"  # "CLEAR", "OBSTACLE", "PREDATOR", "BOTH"

    def update(self):
        p = self._predator
        # Use predator's get_distance — 30000us timeout, more reliable
        self.left_d  = p.get_distance(p.trigL, p.echoL) or 999
        time.sleep_ms(30)
        self.right_d = p.get_distance(p.trigR, p.echoR) or 999

        obs_left   = self.left_d  < self._obstacle.threshold   # 20cm
        obs_right  = self.right_d < self._obstacle.threshold   # 20cm
        pred_left  = self.left_d  < p.LEFT_THRESHOLD           # 50cm
        pred_right = self.right_d < p.RIGHT_THRESHOLD          # 50cm

        is_obstacle = obs_left or obs_right
        is_predator = pred_left or pred_right

        if is_obstacle and is_predator:
            self.classification = "BOTH"
        elif is_obstacle:
            self.classification = "OBSTACLE"
        elif is_predator:
            self.classification = "PREDATOR"
        else:
            self.classification = "CLEAR"

        print("[SENSOR] L:{:.1f} R:{:.1f} -> {}".format(
            self.left_d, self.right_d, self.classification))


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
        self._lux    = 0.0

    def _should_interrupt(self):
        return any(check() for check in self._checks)

    def check(self) -> bool:
        self._lux = self._nav.read_lux()
        print(self._lux)

        if self._lux > 450:
            self._leak = -999  # sentinel for goback
            return True

        if self._lux < 320:
            self._leak = self._nav.compute_leak(self._lux)
            return abs(self._leak) > 0.05

        # Stable light range (320-450): no action
        self._leak = 0.0
        return False

    def execute(self):
        if self._lux > 450:
            self._nav.goback()
        elif self._lux < 320:
            self._nav.decide_action(self._leak)
        else:
            self._nav.stop()


class ObstacleAdapter:
    def __init__(self, obstacle: ObstacleAvoidance, shared: SharedUltrasonicReader):
        self._obstacle      = obstacle
        self._shared        = shared
        self._left_blocked  = False
        self._right_blocked = False

    def check(self) -> bool:
        # Only do real work if classification suggests obstacle range
        if self._shared.classification not in ("OBSTACLE", "BOTH"):
            self._left_blocked  = False
            self._right_blocked = False
            return False

        self._left_blocked  = self._shared.left_d  < self._obstacle.threshold
        self._right_blocked = self._shared.right_d < self._obstacle.threshold
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
    def __init__(self, predator: PredatorRobot, shared: SharedUltrasonicReader):
        self._predator    = predator
        self._shared      = shared
        self._triggered   = False
        self._last_motion = None
        self._last_speed  = 0

    def check(self) -> bool:
        L = self._shared.left_d
        R = self._shared.right_d

        # Always run speed check — fast approach may not be close enough yet
        triggered, motion, speed = self._predator.check_predator_from(L, R)

        # If speed check didn't trigger, use proximity fallback
        # only when classification suggests predator range
        if not triggered and self._shared.classification in ("PREDATOR", "BOTH"):
            left  = L < self._predator.LEFT_THRESHOLD
            right = R < self._predator.RIGHT_THRESHOLD
            if (left or right) and speed > 0:  # only if actually approaching
                diff = abs(L - R)
                if left and right and diff < self._predator.FRONT_DIFF_THRESHOLD:
                    motion = "FRONT"
                elif left:
                    motion = "LEFT"
                else:
                    motion = "RIGHT"
                triggered = True

        self._triggered   = triggered
        self._last_motion = motion
        self._last_speed  = speed
        return triggered

    def quick_check(self) -> bool:
        L = self._shared.left_d
        R = self._shared.right_d
        if L is None or R is None:
            return False
        return L < self._predator.LEFT_THRESHOLD or R < self._predator.RIGHT_THRESHOLD

    def execute(self):
        if self._triggered:
            self._predator.act_predator(self._last_motion, self._last_speed)


def init_hardware():
    loco = QuadrupedLocomotion()
    loco.stand(smooth=True)

    light_sensor.robot = loco
    light_nav = LightNavigator()

    # Predator inits pins FIRST
    pred_robot = PredatorRobot()
    pred_robot.locomotion = loco

    # Obstacle inits AFTER, then we override its pins
    # to use the exact same pin objects as predator
    obstacle = ObstacleAvoidance(loco=loco)
    obstacle.trigL = pred_robot.trigL
    obstacle.echoL = pred_robot.echoL
    obstacle.trigR = pred_robot.trigR
    obstacle.echoR = pred_robot.echoR

    shared = SharedUltrasonicReader(obstacle, pred_robot)

    layer2 = ObstacleAdapter(obstacle, shared)
    layer3 = PredatorAdapter(pred_robot, shared)

    layer1 = PhototaxisAdapter(light_nav, interrupt_checks=[
        layer2._obstacle.quick_check,
        layer3.quick_check,
    ])

    layer0 = LocomotionAdapter(loco, interrupt_checks=[
        layer2._obstacle.quick_check,
        layer3.quick_check,
    ])

    return layer0, layer1, layer2, layer3, shared


def main():
    print("[ARCHITECTURE] Initialising hardware...")
    layer0, layer1, layer2, layer3, shared = init_hardware()
    print("[ARCHITECTURE] Ready. Entering control loop at 20 Hz.")

    while True:
        # ONE sensor read per tick — shared by both obstacle and predator layers
        shared.update()

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

