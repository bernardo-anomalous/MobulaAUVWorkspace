import time
import threading
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo
import board

# === Presets ===
PRESETS = {
    "glide": [90.0, 135.0, 90.0, 135.0, 90.0, 90.0],# left main, left pitch, right main, right pitch, tail 1, tail 2
    "up":  [0.0,  180.0,  180.0,  90.0,  90.0,  90.0],#swing up full pitch
    "down":  [180.0, 90.0, 0.0, 180.0, 90.0, 90.0],#swing down full pitch
}

ANGLE_MIN = 0.0
ANGLE_MAX = 270.0


class ServoTestNode:
    def __init__(self):
        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 100
        
        self.servos = {
            0: Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500),
            1: Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),
            2: Servo(self.pca.channels[2], min_pulse=500, max_pulse=2500),
            3: Servo(self.pca.channels[3], min_pulse=500, max_pulse=2500),
            4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),
            5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),
        }

        self.current_angles = [90.0, 135.0, 90.0, 135.0]  # Default hold
        self.sweep_thread = None
        self.sweep_active = False
        self.manual_mode = False
        self.selected_servo = 0
        self.step_size = 5.0
        
    def print_instructions(self):
        print("\n=== SERVO TEST & CALIBRATION TOOL ===")
        print("---------------------------------------")
        print(" Available Commands:")
        print("  set <servo> <angle>        → Set servo # to angle (clamped 0°–270°)")
        print("  sweep <servo> <min> <max>  → Sweep servo between min and max angles")
        print("  stop                      → Stop sweeping")
        print("  preset <name>             → Move all servos to preset positions")
        print("  mode manual               → Enter manual fine-tuning mode (key controls)")
        print("  exit                      → Exit the program")
        print("")
        print(" Presets Available:")
        for preset in PRESETS:
            print(f"  - {preset}: {PRESETS[preset]}")
        print("---------------------------------------\n")


    def clamp_angle(self, angle):
        return max(ANGLE_MIN, min(ANGLE_MAX, angle))

    def apply_angles(self, angles):
        for i, angle in enumerate(angles):
            if i in self.servos:
                self.servos[i].angle = self.clamp_angle(angle)
        self.current_angles = [self.clamp_angle(a) for a in angles]

    def print_positions(self):
        print("\n---------------------------------------")
        print(" SERVO POSITIONS (degrees):")
        for idx, angle in enumerate(self.current_angles):
            selected = "→" if idx == self.selected_servo and self.manual_mode else " "
            print(f" {selected} [ SERVO {idx} ] → {angle:.1f}")
        if self.manual_mode:
            print(f" Step size: {self.step_size:.1f}°")
            print("---------------------------------------")
            print(" e: increase | d: decrease | n: next | p: previous | +: step up | -: step down | g: glide | q: quit manual mode")
        print("---------------------------------------")

    def move_to_preset(self, name):
        if name in PRESETS:
            self.apply_angles(PRESETS[name])
            print(f"Moved to preset '{name}'.")
        else:
            print(f"Preset '{name}' not found.")

    def sweep_servo(self, servo, min_angle, max_angle):
        def sweep_loop():
            angle = min_angle
            direction = 1
            while self.sweep_active:
                self.servos[servo].angle = self.clamp_angle(angle)
                self.current_angles[servo] = self.clamp_angle(angle)
                self.print_positions()
                angle += direction * 5.0
                if angle >= max_angle or angle <= min_angle:
                    direction *= -1
                time.sleep(0.1)

        self.sweep_active = True
        self.sweep_thread = threading.Thread(target=sweep_loop)
        self.sweep_thread.start()

    def stop_sweep(self):
        self.sweep_active = False
        if self.sweep_thread:
            self.sweep_thread.join()
            self.sweep_thread = None
        print("Sweep stopped.")

    def manual_control(self):
        self.manual_mode = True
        self.print_positions()
        while self.manual_mode:
            key = input("Key: ").strip().lower()
            if key == 'e':
                self.current_angles[self.selected_servo] = self.clamp_angle(self.current_angles[self.selected_servo] + self.step_size)
                self.servos[self.selected_servo].angle = self.current_angles[self.selected_servo]
            elif key == 'd':
                self.current_angles[self.selected_servo] = self.clamp_angle(self.current_angles[self.selected_servo] - self.step_size)
                self.servos[self.selected_servo].angle = self.current_angles[self.selected_servo]
            elif key == 'n':
                self.selected_servo = (self.selected_servo + 1) % 6
            elif key == 'p':
                self.selected_servo = (self.selected_servo - 1) % 6
            elif key == '+':
                self.step_size += 1.0
            elif key == '-':
                self.step_size = max(1.0, self.step_size - 1.0)
            elif key == 'k':
                self.move_to_preset("glide")
            elif key == 'j':
                self.move_to_preset("up")
            elif key == 'l':
                self.move_to_preset("down")
            elif key == 'q':
                self.manual_mode = False
                break
            self.print_positions()

    def run(self):
        print("=== Servo Test Node Started ===")
        self.move_to_preset("glide")
        self.print_instructions()
        try:
            while True:
                self.print_positions()
                cmd = input("COMMAND: ").strip().lower()
                if cmd.startswith("set"):
                    _, idx, angle = cmd.split()
                    idx, angle = int(idx), float(angle)
                    self.current_angles[idx] = self.clamp_angle(angle)
                    self.servos[idx].angle = self.current_angles[idx]
                elif cmd.startswith("sweep"):
                    _, idx, min_angle, max_angle = cmd.split()
                    self.sweep_servo(int(idx), float(min_angle), float(max_angle))
                elif cmd == "stop":
                    self.stop_sweep()
                elif cmd.startswith("preset"):
                    _, name = cmd.split()
                    self.move_to_preset(name)
                elif cmd == "mode manual":
                    if not self.sweep_active:
                        self.manual_control()
                    else:
                        print("Stop the sweep before entering manual mode.")
                elif cmd == "exit":
                    break
                elif cmd == "help":
                    self.print_instructions()

                else:
                    print("Unknown command.")
        except KeyboardInterrupt:
            print("\n[EMERGENCY STOP] Ctrl+C detected. Returning to glide position.")
            self.stop_sweep()
            self.move_to_preset("glide")
        finally:
            self.pca.deinit()
            print("PCA9685 deinitialized. Exiting.")

if __name__ == "__main__":
    node = ServoTestNode()
    node.run()

