from tofDriver import VL53L0X
import time

sensor = VL53L0X()

try:
    while True:
        distance = sensor.read_distance()
        if distance is not None:
            print("Distance: {} mm".format(distance))
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Measurement stopped by User")
finally:
    sensor.close()
