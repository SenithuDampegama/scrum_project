from machine import I2C, Pin
import time
import ujson


PCA9685_ADDR = 0x40
I2C_SDA_PIN = 4
I2C_SCL_PIN = 5
CALIBRATION_FILE = "calibration.json"
LEGS = ["FL", "FR", "RL", "RR"]

# Change channel numbers here if your wiring differs.
SERVO_CHANNELS = {
    "FL": {"hip": 1, "knee": 2},
    "FR": {"hip": 3, "knee": 4},
    "RL": {"hip": 5, "knee": 6},
    "RR": {"hip": 7, "knee": 8},
}

DEFAULT_CALIBRATION = {
    "FL": {"hip_home": 90, "knee_home": 90, "hip_sign": 1, "knee_lift_sign": 1},
    "FR": {"hip_home": 90, "knee_home": 90, "hip_sign": 1, "knee_lift_sign": 1},
    "RL": {"hip_home": 90, "knee_home": 90, "hip_sign": 1, "knee_lift_sign": 1},
    "RR": {"hip_home": 90, "knee_home": 90, "hip_sign": 1, "knee_lift_sign": 1},
}


def clamp_angle(angle):
    try:
        angle = int(angle)
    except:
        angle = 0
    return max(0, min(180, angle))


def clamp_sign(value, default_value):
    try:
        value = int(value)
    except:
        value = default_value
    return 1 if value >= 0 else -1


def copy_calibration(data):
    result = {}
    for leg in LEGS:
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


def default_calibration():
    return copy_calibration(DEFAULT_CALIBRATION)


class PCA9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c, address=PCA9685_ADDR):
        self.i2c = i2c
        self.address = address
        self.write_reg(self.MODE1, 0x00)
        self.set_pwm_freq(50)

    def write_reg(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1 + 0.5)
        old_mode = self.read_reg(self.MODE1)
        self.write_reg(self.MODE1, (old_mode & 0x7F) | 0x10)
        self.write_reg(self.PRESCALE, prescale)
        self.write_reg(self.MODE1, old_mode)
        time.sleep_ms(5)
        self.write_reg(self.MODE1, old_mode | 0xA1)

    def set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        data = bytes((
            on & 0xFF,
            (on >> 8) & 0xFF,
            off & 0xFF,
            (off >> 8) & 0xFF,
        ))
        self.i2c.writeto_mem(self.address, reg, data)


def angle_to_pulse(angle):
    angle = clamp_angle(angle)
    min_us = 500
    max_us = 2500
    pulse_us = min_us + (max_us - min_us) * angle / 180
    return int(pulse_us * 4096 / 20000)


def set_servo_angle(pca, channel, angle):
    pulse = angle_to_pulse(angle)
    pca.set_pwm(channel, 0, pulse)


def load_calibration():
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = ujson.load(f)
        return copy_calibration(data)
    except:
        return default_calibration()


def save_calibration(calibration):
    clean = copy_calibration(calibration)
    with open(CALIBRATION_FILE, "w") as f:
        ujson.dump(clean, f)
    print("Calibration saved to calibration.json")


def move_leg(leg_name):
    if not initialized_legs[leg_name]:
        return False

    hip_angle = calibration[leg_name]["hip_home"]
    knee_angle = calibration[leg_name]["knee_home"]
    set_servo_angle(pca, SERVO_CHANNELS[leg_name]["hip"], hip_angle)
    set_servo_angle(pca, SERVO_CHANNELS[leg_name]["knee"], knee_angle)
    return True


def move_all_home():
    for leg in LEGS:
        if initialized_legs[leg]:
            move_leg(leg)
        else:
            print("Skipping %s: not initialized." % leg)


def print_leg_config(leg_name):
    leg = calibration[leg_name]
    print("Selected leg: %s" % leg_name)
    print("hip_home: %d" % leg["hip_home"])
    print("knee_home: %d" % leg["knee_home"])
    print(
        "Direction settings: hip_sign=%d knee_lift_sign=%d"
        % (leg["hip_sign"], leg["knee_lift_sign"])
    )
    print("initialized: %s" % initialized_legs[leg_name])


def print_selected_leg():
    print_leg_config(LEGS[selected_leg_index])


def print_all_calibration():
    for leg in LEGS:
        print_leg_config(leg)


def reload_calibration():
    global calibration
    global initialized_legs
    calibration = load_calibration()
    initialized_legs = {
        "FL": False,
        "FR": False,
        "RL": False,
        "RR": False,
    }
    print("Reloaded from", CALIBRATION_FILE)


def adjust_selected(joint, delta):
    leg = LEGS[selected_leg_index]
    if not initialized_legs[leg]:
        print("Leg not initialized. Run INIT first.")
        return

    calibration[leg][joint] = clamp_angle(calibration[leg][joint] + delta)
    move_leg(leg)
    print_selected_leg()


def read_angle_prompt(prompt):
    while True:
        try:
            return clamp_angle(input(prompt).strip())
        except:
            print("Please enter a number.")


def init_selected_leg():
    leg = LEGS[selected_leg_index]
    hip_angle = read_angle_prompt("Initial hip angle for %s: " % leg)
    knee_angle = read_angle_prompt("Initial knee angle for %s: " % leg)
    calibration[leg]["hip_home"] = hip_angle
    calibration[leg]["knee_home"] = knee_angle
    initialized_legs[leg] = True
    move_leg(leg)
    print_selected_leg()


def configure_hip_direction():
    leg = LEGS[selected_leg_index]
    if not initialized_legs[leg]:
        print("Leg not initialized. Run INIT first.")
        return

    while True:
        choice = input("Which key moves this leg forward? A or D: ").strip().upper()
        if choice == "D":
            calibration[leg]["hip_sign"] = 1
            break
        if choice == "A":
            calibration[leg]["hip_sign"] = -1
            break
        print("Please enter A or D.")
    print_selected_leg()


def configure_knee_direction():
    leg = LEGS[selected_leg_index]
    if not initialized_legs[leg]:
        print("Leg not initialized. Run INIT first.")
        return

    while True:
        choice = input("Which key makes this leg lift off the ground (extend)? W or S: ").strip().upper()
        if choice == "W":
            calibration[leg]["knee_lift_sign"] = 1
            break
        if choice == "S":
            calibration[leg]["knee_lift_sign"] = -1
            break
        print("Please enter W or S.")
    print_selected_leg()


def main():
    global selected_leg_index
    global step

    print("Simple Pico servo calibration tool")
    print("Commands: Q E W A S D | STEP 1 | STEP 5 | INIT | HIPDIR | KNEEDIR | SHOW | HOME | SAVE | RELOAD | EXIT")
    print_selected_leg()

    while True:
        try:
            cmd = input("> ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            print("Exiting.")
            break

        if not cmd:
            continue

        if cmd == "Q":
            selected_leg_index = (selected_leg_index - 1) % len(LEGS)
            print_selected_leg()
        elif cmd == "E":
            selected_leg_index = (selected_leg_index + 1) % len(LEGS)
            print_selected_leg()
        elif cmd == "A":
            adjust_selected("hip_home", -step)
        elif cmd == "D":
            adjust_selected("hip_home", step)
        elif cmd == "W":
            adjust_selected("knee_home", step)
        elif cmd == "S":
            adjust_selected("knee_home", -step)
        elif cmd == "STEP 1":
            step = 1
            print("Step =", step)
        elif cmd == "STEP 5":
            step = 5
            print("Step =", step)
        elif cmd == "INIT":
            init_selected_leg()
        elif cmd == "HIPDIR":
            configure_hip_direction()
        elif cmd == "KNEEDIR":
            configure_knee_direction()
        elif cmd == "SHOW":
            print_all_calibration()
        elif cmd == "HOME":
            move_all_home()
        elif cmd == "SAVE":
            save_calibration(calibration)
        elif cmd == "RELOAD":
            reload_calibration()
            print_selected_leg()
        elif cmd == "EXIT":
            print("Exiting.")
            break
        else:
            print("Unknown command:", cmd)


i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=400000)
pca = PCA9685(i2c, PCA9685_ADDR)
calibration = load_calibration()
initialized_legs = {
    "FL": False,
    "FR": False,
    "RL": False,
    "RR": False,
}
selected_leg_index = 0
step = 1


if __name__ == "__main__":
    main()
