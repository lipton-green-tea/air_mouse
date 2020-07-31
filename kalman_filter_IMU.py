import numpy as np
import time

class PositionKalmanFilter:
    def __init__(self, initial_state, initial_estimate_uncertainty, v_ret=0.995):
        self.predicted_state = initial_state
        self.estimate_uncertainty = initial_estimate_uncertainty
        self.kalman_gain = None

        self.observation_matrix = np.array([[0,0,0,0,0,0,1,0,0],
                                       [0,0,0,0,0,0,0,1,0],
                                       [0,0,0,0,0,0,0,0,1]])

        time_diff = 0
        self.transition_matrix = np.array([[1,0,0,time_diff,0,0,0.5*time_diff**2,0,0],
                                           [0,1,0,0,time_diff,0,0,0.5*time_diff**2,0],
                                           [0,0,1,0,0,time_diff,0,0,0.5*time_diff**2],
                                           [0,0,0,v_ret,0,0,time_diff,0,0],
                                           [0,0,0,0,v_ret,0,0,time_diff,0],
                                           [0,0,0,0,0,v_ret,0,0,time_diff],
                                           [0,0,0,0,0,0,1,0,0],
                                           [0,0,0,0,0,0,0,1,0],
                                           [0,0,0,0,0,0,0,0,1]])

        self.process_noise_uncertainty = np.array([[0.5,0,0,0,0,0,0,0,0],
                                                   [0,0.5,0,0,0,0,0,0,0],
                                                   [0,0,0.5,0,0,0,0,0,0],
                                                   [0,0,0,0.5,0,0,0,0,0],
                                                   [0,0,0,0,0.5,0,0,0,0],
                                                   [0,0,0,0,0,0.5,0,0,0],
                                                   [0,0,0,0,0,0,0.5,0,0],
                                                   [0,0,0,0,0,0,0,0.5,0],
                                                   [0,0,0,0,0,0,0,0,0.5],])
        self.measurement_uncertainty = np.array([[0.5,0,0],
                                                 [0,0.5,0],
                                                 [0,0,0.5]])

    def predict_and_correct(self, time_diff, measurement):
        self.state_extrapolation(time_diff)
        self.estimate_uncertainty_extrapolation()
        self.kalman_gain_calculation()
        self.estimate_update(measurement)
        self.estimate_uncertainty_update()

    def create_transition_matrix(self, time_diff):
        self.transition_matrix = np.array([[1,0,0,time_diff,0,0,0.5*time_diff**2,0,0],
                                           [0,1,0,0,time_diff,0,0,0.5*time_diff**2,0],
                                           [0,0,1,0,0,time_diff,0,0,0.5*time_diff**2],
                                           [0,0,0,0.998,0,0,time_diff,0,0],
                                           [0,0,0,0,0.998,0,0,time_diff,0],
                                           [0,0,0,0,0,0.998,0,0,time_diff],
                                           [0,0,0,0,0,0,1,0,0],
                                           [0,0,0,0,0,0,0,1,0],
                                           [0,0,0,0,0,0,0,0,1]])
    
    def state_extrapolation(self, time_diff):
        self.create_transition_matrix(time_diff)
        estimated_future_state = np.matmul(self.transition_matrix, self.predicted_state)
        self.predicted_state = estimated_future_state

    def estimate_uncertainty_extrapolation(self):
        covariance = np.matmul(np.matmul(self.transition_matrix,self.estimate_uncertainty),np.transpose(self.transition_matrix)) + self.process_noise_uncertainty
        return covariance

    def kalman_gain_calculation(self):
        self.kalman_gain = np.matmul(self.estimate_uncertainty,np.matmul(np.transpose(self.observation_matrix),np.linalg.inv(np.matmul(np.matmul(self.observation_matrix,self.estimate_uncertainty),np.transpose(self.observation_matrix)) + self.measurement_uncertainty)))
        return self.kalman_gain

    # takes a (numpy) vector of the x,y,z acceleration
    def estimate_update(self,measurement):
        self.predicted_state = self.predicted_state + np.matmul(self.kalman_gain,(measurement - np.matmul(self.observation_matrix,self.predicted_state)))
        return self.predicted_state

    def estimate_uncertainty_update(self):
        calc_part_1 = np.matmul((np.identity(9) - np.matmul(self.kalman_gain,self.observation_matrix)),self.estimate_uncertainty)
        calc_part_2 = np.transpose(np.identity(9) - np.matmul(self.kalman_gain,self.observation_matrix))
        calc_part_3 = np.matmul(np.matmul(self.kalman_gain,self.measurement_uncertainty),np.transpose(self.kalman_gain))
        self.estimate_uncertainty = np.matmul(calc_part_1,calc_part_2) + calc_part_3
        return self.estimate_uncertainty

initial_state = np.array([0,0,0,0,0,0,0,0,0])
initial_estimate_uncertainty = np.array([[0.5,0,0,0,0,0,0,0,0],
                                         [0,0.5,0,0,0,0,0,0,0],
                                         [0,0,0.5,0,0,0,0,0,0],
                                         [0,0,0,0.5,0,0,0,0,0],
                                         [0,0,0,0,0.5,0,0,0,0],
                                         [0,0,0,0,0,0.5,0,0,0],
                                         [0,0,0,0,0,0,0.5,0,0],
                                         [0,0,0,0,0,0,0,0.5,0],
                                         [0,0,0,0,0,0,0,0,0.5]])





class OrientationKalmanFilter:
    def __init__(self, initial_state, initial_estimate_uncertainty):
        self.predicted_state = initial_state
        self.estimate_uncertainty = initial_estimate_uncertainty
    
    # estimates the next state based on the current state, inputs to
    # the system and a transition model
    def state_extrapolation(self, input_variables):
        transition_matrix = np.array([])
        control_matrix = np.array([])
        estimated_future_state = np.matmul(transition_matrix, self.predicted_state) + np.matmul(control_matrix, input_variables)
        return estimated_future_state

    # calculates the uncertainty in our state extrapolation prediction
    def covariance_extrapolation(self):
        covariance = 1
        return covariance

    # returns an estimated current state based on the measurement
    # input and the estimated state
    def state_update(self):
        return 1

    # calculates the Kalman gain
    def kalman_gain(self):
        k_gain = 1
        return k_gain

    # calculates the uncertainty in our state estimate
    def coveriance_update_equation(self):
        return 1

import math
import RPi.GPIO as GPIO
from time import sleep
import smbus

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

        
class controller:
    def __init__(self):
        self.imu = IMU()
        self.imu.set_offsets()
        initial_state = np.array([0,0,0,0,0,0,0,0,0])
        initial_estimate_uncertainty = np.array([[0.5,0,0,0,0,0,0,0,0],
                                                 [0,0.5,0,0,0,0,0,0,0],
                                                 [0,0,0.5,0,0,0,0,0,0],
                                                 [0,0,0,0.5,0,0,0,0,0],
                                                 [0,0,0,0,0.5,0,0,0,0],
                                                 [0,0,0,0,0,0.5,0,0,0],
                                                 [0,0,0,0,0,0,0.5,0,0],
                                                 [0,0,0,0,0,0,0,0.5,0],
                                                 [0,0,0,0,0,0,0,0,0.5]])


        self.pkf = PositionKalmanFilter(initial_state, initial_estimate_uncertainty,v_ret=0.93)
        
    def track(self):
        old_time = time.time()
        while(True):
            if GPIO.input(10) == GPIO.HIGH:
                print("resetting offsets")
                self.imu.set_offsets()
                self.pkf.predicted_state[0] = 0
                self.pkf.predicted_state[1] = 0
                self.pkf.predicted_state[2] = 0
            if GPIO.input(12) == GPIO.HIGH:
                break
            
            x,y,z = self.imu.read_acc()
            #print('%8s' % str(x) + '%8s' % str(y) + '%8s' % str(z))
            measurement = np.array([x,y,z])
            new_time = time.time()
            time_diff = new_time - old_time
            old_time = new_time
            self.pkf.predict_and_correct(time_diff, measurement)
            print(self.pkf.predicted_state[1],self.pkf.predicted_state[2])
        
ctrl = controller()
ctrl.track()

