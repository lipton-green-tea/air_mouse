from reader import Reader
import pyautogui
import keyboard
import time

class Controller:
    def __init__(self, mouse_speed=1):
        self.reader = Reader(baud=38400, port='COM7')

        self.x_velocity = 0
        self.y_velocity = 0

        self.x_offset = 0
        self.y_offset = 0

        # seconds till velocity returns to 0
        self.x_brake = 0.7
        self.y_brake = 0.7

        self.mouse_speed = mouse_speed

    def set_offsets(self, repeats=40):
        x_total = 0
        y_total = 0
        for x in range(0,repeats):
            time.sleep(0.01)
            accelerations = self.reader.read()
            if len(accelerations) == 3:
                x_total += accelerations[0]
                y_total += accelerations[1]

        self.x_offset = -int(x_total/repeats)
        self.y_offset = -int(y_total/repeats)

    def start_tracking(self):
        old_time = time.time()
        while(True):
            time.sleep(0.0025)
            if keyboard.is_pressed('q'):
                print('quitting')
                self.reader.close()
                break
            elif keyboard.is_pressed('r'):
                print('resetting speed and offsets')
                self.x_velocity = 0
                self.y_velocity = 0
                self.set_offsets()
                old_time = time.time()
            else:
                accelerations = self.reader.read()
                if len(accelerations) == 3:
                    x_acceleration = accelerations[0] + self.x_offset
                    y_acceleration = accelerations[1] + self.y_offset
                    new_time = time.time()
                    time_diff = new_time - old_time
                    if time_diff > self.y_brake or time_diff > self.x_brake:
                        x_retention = 0
                        y_retention = 0
                    else:
                        x_retention = 1 - (time_diff / self.x_brake)
                        y_retention = 1 - (time_diff / self.y_brake)
                    self.x_velocity = self.x_velocity * x_retention + x_acceleration * time_diff
                    self.y_velocity = self.y_velocity * y_retention + y_acceleration * time_diff
                    x_displacement = self.x_velocity * time_diff * self.mouse_speed * -1
                    y_displacement = self.y_velocity * time_diff * self.mouse_speed
                    pyautogui.moveRel(xOffset=x_displacement, yOffset=y_displacement, duration=0.001)
                    old_time = new_time
                    print(x_acceleration, y_acceleration, self.x_velocity, self.y_velocity, time_diff)
                else:
                    old_time = time.time()