from machine import I2C, Pin
import time
import ujson


PCA9685_ADDR = 0x40
I2C_ID = 0
I2C_SDA_PIN = 4
I2C_SCL_PIN = 5
SERVO_FREQUENCY = 50
CALIBRATION_FILE = "calibration.json"
LEG_ORDER = ("FL", "FR", "RL", "RR")
CRAWL_SEQUENCE = ("FL", "RR", "FR", "RL")

# Edit channel wiring here if your robot is wired differently.
DEFAULT_CHANNEL_MAP = {
    "FL": {"hip": 1, "knee": 2},
    "FR": {"hip": 3, "knee": 4},
    "RL": {"hip": 5, "knee": 6},
    "RR": {"hip": 7, "knee": 8},
}

DEFAULT_CALIBRATION = {
    "FL": {"hip_home": 90, "knee_home": 15, "hip_sign": 1, "knee_lift_sign": 1},
    "FR": {"hip_home": 90, "knee_home": 15, "hip_sign": 1, "knee_lift_sign": 1},
    "RL": {"hip_home": 90, "knee_home": 15, "hip_sign": 1, "knee_lift_sign": 1},
    "RR": {"hip_home": 90, "knee_home": 15, "hip_sign": 1, "knee_lift_sign": 1},
}


def clamp_angle(angle):
    try:
        angle = int(angle)
    except:
        angle = 90
    return max(0, min(180, angle))


def clamp_sign(value, default_value=1):
    try:
        value = int(value)
    except:
        value = default_value
    return 1 if value >= 0 else -1


def copy_calibration(data):
    result = {}
    for leg in LEG_ORDER:
        leg_data = data.get(leg, {})
        result[leg] = {
            "hip_home": clamp_angle(
                leg_data.get(
                    "hip_home",
                    leg_data.get("hip", DEFAULT_CALIBRATION[leg]["hip_home"]),
                )
            ),
            "knee_home": clamp_angle(
                leg_data.get(
                    "knee_home",
                    leg_data.get("knee", DEFAULT_CALIBRATION[leg]["knee_home"]),
                )
            ),
            "hip_sign": clamp_sign(
                leg_data.get("hip_sign", DEFAULT_CALIBRATION[leg]["hip_sign"]),
                DEFAULT_CALIBRATION[leg]["hip_sign"],
            ),
            "knee_lift_sign": clamp_sign(
                leg_data.get(
                    "knee_lift_sign",
                    DEFAULT_CALIBRATION[leg]["knee_lift_sign"],
                ),
                DEFAULT_CALIBRATION[leg]["knee_lift_sign"],
            ),
        }
    return result


def load_calibration(filename=CALIBRATION_FILE):
    try:
        with open(filename, "r") as calibration_file:
            data = ujson.load(calibration_file)
        return copy_calibration(data)
    except:
        return copy_calibration(DEFAULT_CALIBRATION)


class PCA9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c, address=PCA9685_ADDR, frequency=SERVO_FREQUENCY):
        self.i2c = i2c
        self.address = address
        self._write_reg(self.MODE1, 0x00)
        self.set_pwm_freq(frequency)

    def _write_reg(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_pwm_freq(self, freq_hz):
        prescale = int(25000000.0 / (4096 * freq_hz) - 1 + 0.5)
        old_mode = self._read_reg(self.MODE1)
        self._write_reg(self.MODE1, (old_mode & 0x7F) | 0x10)
        self._write_reg(self.PRESCALE, prescale)
        self._write_reg(self.MODE1, old_mode)
        time.sleep_ms(5)
        self._write_reg(self.MODE1, old_mode | 0xA1)

    def set_pwm(self, channel, on_count, off_count):
        reg = self.LED0_ON_L + 4 * channel
        data = bytes((
            on_count & 0xFF,
            (on_count >> 8) & 0xFF,
            off_count & 0xFF,
            (off_count >> 8) & 0xFF,
        ))
        self.i2c.writeto_mem(self.address, reg, data)


class ServoDriver:
    def __init__(
        self,
        i2c=None,
        address=PCA9685_ADDR,
        frequency=SERVO_FREQUENCY,
        min_pulse_us=500,
        max_pulse_us=2500,
        frame_us=20000,
        smooth_steps=8,
        smooth_delay_ms=20,
    ):
        if i2c is None:
            i2c = I2C(
                I2C_ID,
                sda=Pin(I2C_SDA_PIN),
                scl=Pin(I2C_SCL_PIN),
                freq=400000,
            )
        self.frequency = frequency
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.frame_us = frame_us
        self.smooth_steps = max(1, int(smooth_steps))
        self.smooth_delay_ms = max(0, int(smooth_delay_ms))
        self._pca = PCA9685(i2c, address, frequency=frequency)
        self._angles = {}

    def _angle_to_count(self, angle):
        angle = clamp_angle(angle)
        pulse_us = self.min_pulse_us + (
            (self.max_pulse_us - self.min_pulse_us) * angle / 180
        )
        return int(pulse_us * 4096 / self.frame_us)

    def set_angle(self, channel, angle):
        angle = clamp_angle(angle)
        self._pca.set_pwm(channel, 0, self._angle_to_count(angle))
        self._angles[channel] = angle

    def get_angle(self, channel):
        return self._angles.get(channel)

    def move_smooth(self, targets, steps=None, delay_ms=None):
        if not targets:
            return

        steps = self.smooth_steps if steps is None else max(1, int(steps))
        delay_ms = self.smooth_delay_ms if delay_ms is None else max(0, int(delay_ms))

        has_all_starts = True
        starts = {}
        for channel, target in targets.items():
            start = self.get_angle(channel)
            if start is None:
                has_all_starts = False
                break
            starts[channel] = start

        if not has_all_starts or steps <= 1:
            for channel, target in targets.items():
                self.set_angle(channel, target)
            return

        for step_index in range(1, steps + 1):
            for channel, target in targets.items():
                start = starts[channel]
                intermediate = start + (target - start) * step_index / steps
                self.set_angle(channel, intermediate)
            if delay_ms:
                time.sleep_ms(delay_ms)


class Leg:
    def __init__(
        self,
        name,
        driver,
        hip_channel,
        knee_channel,
        hip_home,
        knee_home,
        hip_sign,
        knee_lift_sign,
    ):
        self.name = name
        self.driver = driver
        self.hip_channel = hip_channel
        self.knee_channel = knee_channel
        self.hip_home = clamp_angle(hip_home)
        self.knee_home = clamp_angle(knee_home)
        self.hip_sign = clamp_sign(hip_sign)
        self.knee_lift_sign = clamp_sign(knee_lift_sign)
        self._hip_angle = self.hip_home
        self._knee_angle = self.knee_home

    def set_pose(self, hip_angle, knee_angle, smooth=False):
        hip_angle = clamp_angle(hip_angle)
        knee_angle = clamp_angle(knee_angle)
        if smooth:
            self.driver.move_smooth({
                self.hip_channel: hip_angle,
                self.knee_channel: knee_angle,
            })
        else:
            self.driver.set_angle(self.hip_channel, hip_angle)
            self.driver.set_angle(self.knee_channel, knee_angle)
        self._hip_angle = hip_angle
        self._knee_angle = knee_angle

    def home(self, smooth=False):
        self.set_pose(self.hip_home, self.knee_home, smooth=smooth)

    def current_pose(self):
        return {
            "hip_angle": self._hip_angle,
            "knee_angle": self._knee_angle,
        }


class QuadrupedLocomotion:
    def __init__(
        self,
        calibration_file=CALIBRATION_FILE,
        channel_map=None,
        swing_amount=30,
        lift_amount=15,
        support_push_amount=30,
        placed_leg_push_end=0,
        recovery_order=None,
        recover_threshold=10,
        settle_delay_ms=120,
        phase_delay_ms=160,
        push_substeps=2,
        turn_swing_amount=55,
        bow_front_extend_amount=10,
        bow_rear_support_amount=5,
        wave_swing_amount=10,
        servo_driver=None,
    ):
        self.channel_map = channel_map or DEFAULT_CHANNEL_MAP
        self.calibration = load_calibration(calibration_file)
        self.driver = servo_driver or ServoDriver()

        self.swing_amount = max(1, int(swing_amount))
        self.lift_amount = max(1, int(lift_amount))
        self.support_push_amount = max(1, int(support_push_amount))
        self.placed_leg_push_end = int(placed_leg_push_end)
        self.recover_threshold = max(0, int(recover_threshold))

        self.settle_delay_ms = max(0, int(settle_delay_ms))
        self.phase_delay_ms = max(0, int(phase_delay_ms))
        self.push_substeps = max(1, int(push_substeps))

        self.turn_swing_amount = max(1, int(turn_swing_amount))
        self.bow_front_extend_amount = max(0, int(bow_front_extend_amount))
        self.bow_rear_support_amount = max(0, int(bow_rear_support_amount))
        self.wave_swing_amount = max(1, int(wave_swing_amount))

        self.legs = self._create_legs()

        self.hip_offsets = {leg_name: 0 for leg_name in LEG_ORDER}
        self.knee_offsets = {leg_name: 0 for leg_name in LEG_ORDER}

        if recovery_order is None:
            self.recovery_order = CRAWL_SEQUENCE
        else:
            self.recovery_order = tuple(recovery_order)

    def _create_legs(self):
        legs = {}
        for leg_name in LEG_ORDER:
            leg_channels = self.channel_map[leg_name]
            leg_calibration = self.calibration[leg_name]
            legs[leg_name] = Leg(
                name=leg_name,
                driver=self.driver,
                hip_channel=leg_channels["hip"],
                knee_channel=leg_channels["knee"],
                hip_home=leg_calibration["hip_home"],
                knee_home=leg_calibration["knee_home"],
                hip_sign=leg_calibration["hip_sign"],
                knee_lift_sign=leg_calibration["knee_lift_sign"],
            )
        return legs

    def _delay(self):
        if self.phase_delay_ms:
            time.sleep_ms(self.phase_delay_ms)

    def _settle(self):
        if self.settle_delay_ms:
            time.sleep_ms(self.settle_delay_ms)

    def get_leg(self, leg_name):
        if leg_name not in self.legs:
            raise ValueError("Unknown leg: %s" % leg_name)
        return self.legs[leg_name]

    def _relative_to_absolute_hip(self, leg_name, hip_offset):
        leg = self.get_leg(leg_name)
        return clamp_angle(leg.hip_home + leg.hip_sign * hip_offset)

    def _relative_to_absolute_knee(self, leg_name, knee_offset):
        leg = self.get_leg(leg_name)
        return clamp_angle(leg.knee_home + leg.knee_lift_sign * knee_offset)

    def _apply_pose_targets(self, targets, smooth=True, steps=None):
        channel_targets = {}
        for leg_name, (hip_offset, knee_offset) in targets.items():
            leg = self.get_leg(leg_name)
            channel_targets[leg.hip_channel] = self._relative_to_absolute_hip(
                leg_name, hip_offset
            )
            channel_targets[leg.knee_channel] = self._relative_to_absolute_knee(
                leg_name, knee_offset
            )

        if smooth:
            self.driver.move_smooth(channel_targets, steps=steps)
        else:
            for channel, angle in channel_targets.items():
                self.driver.set_angle(channel, angle)

        for leg_name, (hip_offset, knee_offset) in targets.items():
            leg = self.get_leg(leg_name)
            self.hip_offsets[leg_name] = int(hip_offset)
            self.knee_offsets[leg_name] = int(knee_offset)
            leg._hip_angle = self._relative_to_absolute_hip(leg_name, hip_offset)
            leg._knee_angle = self._relative_to_absolute_knee(leg_name, knee_offset)

    def _set_leg_relative(self, leg_name, hip_offset=None, knee_offset=None, smooth=True):
        if hip_offset is None:
            hip_offset = self.hip_offsets[leg_name]
        if knee_offset is None:
            knee_offset = self.knee_offsets[leg_name]
        self._apply_pose_targets({
            leg_name: (hip_offset, knee_offset)
        }, smooth=smooth)

    def _move_hips_to_offsets(self, hip_targets, smooth=True):
        targets = {}
        for leg_name in hip_targets:
            targets[leg_name] = (hip_targets[leg_name], self.knee_offsets[leg_name])
        self._apply_pose_targets(targets, smooth=smooth)

    def _interpolate_hip_targets(self, hip_targets, steps):
        steps = max(1, int(steps))
        start_offsets = {}
        for leg_name, target in hip_targets.items():
            start_offsets[leg_name] = self.hip_offsets[leg_name]

        for step_index in range(1, steps + 1):
            partial_targets = {}
            for leg_name, target in hip_targets.items():
                start = start_offsets[leg_name]
                interpolated = start + (target - start) * step_index / steps
                partial_targets[leg_name] = int(round(interpolated))
            self._move_hips_to_offsets(partial_targets, smooth=True)
            self._delay()

    def _push_phase(self, stepping_leg_name, direction):
        """
        direction = +1 for forward walking
        direction = -1 for backward walking

        stepping leg:
            from forward swing position toward placed_leg_push_end
        support legs:
            pushed backward on ground
        """
        hip_targets = {}
        for leg_name in LEG_ORDER:
            if leg_name == stepping_leg_name:
                hip_targets[leg_name] = direction * self.placed_leg_push_end
            else:
                hip_targets[leg_name] = -direction * self.support_push_amount

        self._interpolate_hip_targets(hip_targets, self.push_substeps)

    def _needs_recovery(self, leg_name, direction):
        hip_offset = self.hip_offsets[leg_name]
        return (direction * hip_offset) < -self.recover_threshold

    def _recover_leg_to_home(self, leg_name):
        """
        Recovery rule:
        - lift leg
        - move hip back to home (0) in air
        - place down
        """
        current_hip = self.hip_offsets[leg_name]

        if current_hip == 0 and self.knee_offsets[leg_name] == 0:
            return

        # lift
        self._set_leg_relative(
            leg_name,
            hip_offset=current_hip,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()

        # move hip to home while lifted
        self._set_leg_relative(
            leg_name,
            hip_offset=0,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()

        # place
        self._set_leg_relative(
            leg_name,
            hip_offset=0,
            knee_offset=0,
            smooth=True,
        )
        self._settle()

    def _recovery_phase(self, stepping_leg_name, direction):
        """
        Recover legs one by one, lifted, back to home.
        We skip the stepping leg here because it has just been placed and pushed.
        """
        for leg_name in self.recovery_order:
            if leg_name == stepping_leg_name:
                continue
            if self._needs_recovery(leg_name, direction):
                self._recover_leg_to_home(leg_name)

    def _execute_step(self, stepping_leg_name, direction):
        """
        Hybrid gait:
        1. Lift stepping leg
        2. Swing stepping leg forward in air
        3. Place stepping leg
        4. Push phase
        5. Recover over-back support legs one by one while lifted
        """
        swing_target = direction * self.swing_amount

        # 1. Lift stepping leg
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=self.hip_offsets[stepping_leg_name],
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()

        # 2. Swing stepping leg forward in air
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=swing_target,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()

        # 3. Place stepping leg
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=swing_target,
            knee_offset=0,
            smooth=True,
        )
        self._settle()

        # 4. Push phase
        self._push_phase(stepping_leg_name, direction)

        # 5. Recovery phase (leg by leg, lifted)
        self._recovery_phase(stepping_leg_name, direction)

    def _crawl_cycle(self, direction_provider, steps):
        steps = max(0, int(steps))
        for _ in range(steps):
            for leg_name in CRAWL_SEQUENCE:
                direction = direction_provider(leg_name)
                self._execute_step(leg_name, direction)

    def _execute_step_interruptible(self, stepping_leg_name, direction, should_interrupt):
        """
        Same as _execute_step but checks should_interrupt() between
        each of the 5 phases. Returns True if interrupted.
        Robot is left in a safe standing position if interrupted.
        """
        swing_target = direction * self.swing_amount

        # 1. Lift stepping leg
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=self.hip_offsets[stepping_leg_name],
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()
        if should_interrupt():
            self.stand(smooth=False)
            return True

        # 2. Swing stepping leg forward in air
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=swing_target,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._settle()
        if should_interrupt():
            self.stand(smooth=False)
            return True

        # 3. Place stepping leg
        self._set_leg_relative(
            stepping_leg_name,
            hip_offset=swing_target,
            knee_offset=0,
            smooth=True,
        )
        self._settle()
        if should_interrupt():
            self.stand(smooth=False)
            return True

        # 4. Push phase
        self._push_phase(stepping_leg_name, direction)
        if should_interrupt():
            self.stand(smooth=False)
            return True

        # 5. Recovery phase
        self._recovery_phase(stepping_leg_name, direction)

        return False

    def _crawl_cycle_interruptible(self, direction_provider, steps, should_interrupt):
        """
        Same as _crawl_cycle but checks should_interrupt() between
        each phase of every leg movement. If it returns True, the robot
        stands immediately and returns True to signal interruption.
        """
        steps = max(0, int(steps))
        for _ in range(steps):
            for leg_name in CRAWL_SEQUENCE:
                if should_interrupt():
                    self.stand(smooth=False)
                    return True
                direction = direction_provider(leg_name)
                interrupted = self._execute_step_interruptible(
                    leg_name, direction, should_interrupt
                )
                if interrupted:
                    return True
        return False

    def walk_forward_interruptible(self, steps=1, should_interrupt=None):
        """
        Walk forward but stop immediately between leg movements
        if should_interrupt() returns True.
        Returns True if interrupted, False if completed normally.
        """
        if should_interrupt is None:
            return self.walk_forward(steps)
        self.stand(smooth=True)
        self._delay()
        return self._crawl_cycle_interruptible(
            lambda leg_name: 1, steps, should_interrupt
        )

    def stand(self, smooth=True):
        targets = {}
        for leg_name in LEG_ORDER:
            targets[leg_name] = (0, 0)
        self._apply_pose_targets(targets, smooth=smooth)
        return self

    def walk_forward(self, steps=1, stand_before=True, stand_after=False):
        if stand_before:
            self.stand(smooth=True)
            self._delay()
        self._crawl_cycle(lambda leg_name: 1, steps)
        if stand_after:
            self.stand(smooth=True)
        return self

    def walk_backward(self, steps=1, stand_before=True, stand_after=False):
        if stand_before:
            self.stand(smooth=True)
            self._delay()
        self._crawl_cycle(lambda leg_name: -1, steps)
        if stand_after:
            self.stand(smooth=True)
        return self

    def turn_left(self, steps=1, stand_before=True, stand_after=False):
        if stand_before:
            self.stand(smooth=True)
            self._delay()

        def left_turn_direction(leg_name):
            if leg_name in ("FR", "RR"):
                return 1
            return -1

        saved_swing = self.swing_amount
        saved_push = self.support_push_amount

        self.swing_amount = self.turn_swing_amount
        self.support_push_amount = self.turn_swing_amount

        self._crawl_cycle(left_turn_direction, steps)

        self.swing_amount = saved_swing
        self.support_push_amount = saved_push

        if stand_after:
            self.stand(smooth=True)
        return self

    def turn_right(self, steps=1, stand_before=True, stand_after=False):
        if stand_before:
            self.stand(smooth=True)
            self._delay()

        def right_turn_direction(leg_name):
            if leg_name in ("FL", "RL"):
                return 1
            return -1

        saved_swing = self.swing_amount
        saved_push = self.support_push_amount

        self.swing_amount = self.turn_swing_amount
        self.support_push_amount = self.turn_swing_amount

        self._crawl_cycle(right_turn_direction, steps)

        self.swing_amount = saved_swing
        self.support_push_amount = saved_push

        if stand_after:
            self.stand(smooth=True)
        return self

    def bow(self, smooth=True):
        self.stand(smooth=smooth)

        targets = {}
        for leg_name in ("FL", "FR"):
            targets[leg_name] = (0, -self.bow_front_extend_amount)

        for leg_name in ("RL", "RR"):
            targets[leg_name] = (0, self.bow_rear_support_amount)

        self._apply_pose_targets(targets, smooth=smooth)
        self._delay()
        return self

    def wave_front_left(self, cycles=2):
        self.stand(smooth=True)
        self._delay()

        self._set_leg_relative(
            "FL",
            hip_offset=0,
            knee_offset=self.lift_amount + 4,
            smooth=True,
        )
        self._delay()

        cycles = max(1, int(cycles))
        for _ in range(cycles):
            self._set_leg_relative(
                "FL",
                hip_offset=self.wave_swing_amount,
                knee_offset=self.lift_amount + 4,
                smooth=True,
            )
            self._delay()
            self._set_leg_relative(
                "FL",
                hip_offset=-self.wave_swing_amount,
                knee_offset=self.lift_amount + 4,
                smooth=True,
            )
            self._delay()

        self._set_leg_relative("FL", hip_offset=0, knee_offset=0, smooth=True)
        self._delay()
        self.stand(smooth=True)
        return self

    def test_leg(self, leg_name):
        self.stand(smooth=True)
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=0,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=self.swing_amount,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=self.swing_amount,
            knee_offset=0,
            smooth=True,
        )
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=-self.support_push_amount,
            knee_offset=0,
            smooth=True,
        )
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=0,
            knee_offset=self.lift_amount,
            smooth=True,
        )
        self._delay()

        self._set_leg_relative(
            leg_name,
            hip_offset=0,
            knee_offset=0,
            smooth=True,
        )
        self._delay()
        return self

    def pose_snapshot(self):
        snapshot = {}
        for leg_name in LEG_ORDER:
            snapshot[leg_name] = {
                "hip_offset": self.hip_offsets[leg_name],
                "knee_offset": self.knee_offsets[leg_name],
                "absolute_hip": self.get_leg(leg_name)._hip_angle,
                "absolute_knee": self.get_leg(leg_name)._knee_angle,
            }
        return snapshot


if __name__ == "__main__":
    robot = QuadrupedLocomotion()
    robot.stand(smooth=True)
    print("Robot initialized and standing.")
    print("Try: robot.walk_forward(steps=1)")