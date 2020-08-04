from controller import Controller
from reader import Reader
import time




control = Controller(mouse_speed=0.4)
control.set_offsets()
control.start_tracking()


# reader = Reader(baud=38400)
# time.sleep(0.05)
# print(reader.calc_variance(repeats=100))
# reader.close()
