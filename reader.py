import serial
import time
import re
from matplotlib import pyplot as plt

class Reader:
    def __init__(self, baud=9600, port='COM7'):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.regex = re.compile(r'Ac[XYZ] = (-?\d+)')

    def read(self):
        self.ser.write(b' ')
        time.sleep(0.0025)
        readings = self.ser.readline()
        readings_decoded = readings.decode("utf-8")
        accelerations = self.regex.findall(readings_decoded)
        for x in range(0, len(accelerations)):
            accelerations[x] = int(accelerations[x])
        return accelerations

    def close(self):
        self.ser.close()

    def calc_variance(self, repeats=100, rest_time=0.005, plot=False):
        x_total = 0
        y_total = 0
        z_total = 0
        xsq_total = 0
        ysq_total = 0
        zsq_total = 0
        readings = 0
        for x in range(0, repeats):
            accelerations = self.read()

            if len(accelerations) == 3:
                readings += 1

                x_total += accelerations[0]
                y_total += accelerations[1]
                z_total += accelerations[2]

                xsq_total += accelerations[0] ** 2
                ysq_total += accelerations[1] ** 2
                zsq_total += accelerations[2] ** 2

            time.sleep(rest_time)

        x_var = xsq_total/readings - (x_total/readings) ** 2
        y_var = ysq_total/readings - (y_total/readings) ** 2
        z_var = zsq_total/readings - (z_total/readings) ** 2


        return x_var, y_var, z_var, readings
