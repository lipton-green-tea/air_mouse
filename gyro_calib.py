import RPi.GPIO as GPIO
from time import sleep
import smbus

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(10, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

registers = {'gyro_x':0x43,'gyro_y':0x45,'gyro_z':0x47,'acc_x':0x3b,'acc_y':0x3d,'acc_z':0x3f}
power_mgmt_1 = 0x6b

x_offset = 0
y_offset = 0
z_offset = 0

def read_gyro():
    x_read = read_word_2c(registers['gyro_x'])
    y_read = read_word_2c(registers['gyro_y'])
    z_read = read_word_2c(registers['gyro_z'])
    x = x_read + x_offset
    y = y_read + y_offset
    z = z_read + z_offset
    return x,y,z

def read_acc():
    x_read = read_word_2c(registers['acc_x'])
    y_read = read_word_2c(registers['acc_y'])
    z_read = read_word_2c(registers['acc_z'])
    x = x_read + x_offset
    y = y_read + y_offset
    z = z_read + z_offset
    return x,y,z

def set_offsets():
    repeats = 10
    x_mean = 0
    y_mean = 0
    z_mean = 0
    sleep(1.5)
    for repeat in range(0,repeats):
        sleep(0.03)
        x,y,z = read_gyro()
        x_mean += x
        y_mean += y
        z_mean += z
    x_mean = round(x_mean/repeats)
    y_mean = round(y_mean/repeats)
    z_mean = round(z_mean/repeats)
    new_x_offset = x_offset - x_mean
    new_y_offset = y_offset - y_mean
    new_z_offset = z_offset - z_mean
    return new_x_offset, new_y_offset, new_z_offset

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg + 1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, power_mgmt_1, 0)
    

while(True):
    if GPIO.input(10) == GPIO.HIGH:
        print("resetting offsets")
        x_offset,y_offset,z_offset = set_offsets()
        sleep(0.1)
    if GPIO.input(12) == GPIO.HIGH:
        x,y,z = read_acc()
        print('%8s' % str(x) + '%8s' % str(y) + '%8s' % str(z))
        sleep(0.1)
        
