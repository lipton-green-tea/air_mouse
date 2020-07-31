import RPi.GPIO as GPIO
from time import sleep
import smbus
import pyautogui

class IMU:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(10, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

        self.registers = {'gyro_x':0x43,'gyro_y':0x45,'gyro_z':0x47,'acc_x':0x3b,'acc_y':0x3d,'acc_z':0x3f}

        self.x_gyro_offset = 0
        self.y_gyro_offset = 0
        self.z_gyro_offset = 0

        self.x_acc_offset = 0
        self.y_acc_offset = 0
        self.z_acc_offset = 0

        self.bus = smbus.SMBus(1)
        self.address = 0x68
        
        power_mgmt_1 = 0x6b        
        self.bus.write_byte_data(self.address, power_mgmt_1, 0)

    def read_gyro(self):
        x_read = self.read_word_2c(self.registers['gyro_x'])
        y_read = self.read_word_2c(self.registers['gyro_y'])
        z_read = self.read_word_2c(self.registers['gyro_z'])
        x = x_read + self.x_gyro_offset
        y = y_read + self.y_gyro_offset
        z = z_read + self.z_gyro_offset
        return x,y,z

    def read_acc(self):
        x_read = self.read_word_2c(self.registers['acc_x'])
        y_read = self.read_word_2c(self.registers['acc_y'])
        z_read = self.read_word_2c(self.registers['acc_z'])
        x = x_read + self.x_acc_offset
        y = y_read + self.y_acc_offset
        z = z_read + self.z_acc_offset
        return x,y,z

    def set_offsets(self):
        repeats = 10
        x_gyro_mean = 0
        y_gyro_mean = 0
        z_gyro_mean = 0
        x_acc_mean = 0
        y_acc_mean = 0
        z_acc_mean = 0
        for repeat in range(0,repeats):
            sleep(0.03)
            x,y,z = self.read_gyro()
            x_gyro_mean += x
            y_gyro_mean += y
            z_gyro_mean += z
            x,y,z = self.read_acc()
            x_acc_mean += x
            y_acc_mean += y
            z_acc_mean += z
        x_gyro_mean = round(x_gyro_mean/repeats)
        y_gyro_mean = round(y_gyro_mean/repeats)
        z_gyro_mean = round(z_gyro_mean/repeats)
        x_acc_mean = round(x_acc_mean/repeats)
        y_acc_mean = round(y_acc_mean/repeats)
        z_acc_mean = round(z_acc_mean/repeats)
        self.x_gyro_offset = self.x_gyro_offset - x_gyro_mean
        self.y_gyro_offset = self.y_gyro_offset - y_gyro_mean
        self.z_gyro_offset = self.z_gyro_offset - z_gyro_mean
        self.x_acc_offset = self.x_acc_offset - x_acc_mean
        self.y_acc_offset = self.y_acc_offset - y_acc_mean
        self.z_acc_offset = self.z_acc_offset - z_acc_mean

    def calc_acc_tilt(self):
        x,y,z = self.read_acc()
        rho = math.atan(x/math.sqrt((y*y) + (z*z)))
        phi = math.atan(y/math.sqrt((x*x) + (z*z)))
        theta = math.atan(math.sqrt((x*x) + (y*y))/z)

    def read_word(self, reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg + 1)
        value = (h << 8) + l
        return value

    def read_word_2c(self, reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val 

class tracker:
    def __init__(self):
        self.x_velocity = 0
        self.y_velocity = 0
        self.z_velocity = 0

        self.x_acceleration = 0
        self.y_acceleration = 0
        self.z_acceleration = 0

        self.x_brake = 0.995
        self.y_brake = 0.995
        self.z_brake = 0.995

        self.mouse_speed = 1

        self.imu = IMU()

    def update_mouse_pos(self, time):
        x_displacement = self.x_velocity * time + 0.5 * self.x_acceleration * time * time
        y_displacement = self.y_velocity * time + 0.5 * self.y_acceleration * time * time

        pyautogui.dragRel(self.mouse_speed * x_displacement, self.mouse_speed * y_displacement, duration=0.0001)

        self.x_velocity = self.x_velocity * self.x_brake + self.x_acceleration * time
        self.y_velocity = self.y_velocity * self.y_brake + self.y_acceleration * time

    def begin_tracking(self):
        def track(self):
        old_time = time.time()
        while(True):
            if GPIO.input(10) == GPIO.HIGH:
                print("resetting offsets")
                self.imu.set_offsets()
                self.x_velocity = 0
                self.y_velocity = 0
                self.z_velocity = 0
            if GPIO.input(12) == GPIO.HIGH:
                break
            
            self.x_acceleration,self.y_acceleration,self.z_acceleration = self.imu.read_acc()
            #print('%8s' % str(x) + '%8s' % str(y) + '%8s' % str(z))
            new_time = time.time()
            time_diff = new_time - old_time
            old_time = new_time
            self.update_mouse_pos(self, time_diff)
            print(self.pkf.predicted_state[1],self.pkf.predicted_state[2])


    

    