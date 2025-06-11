import smbus
import time


class VL53L0X:
    def __init__(self, bus=1, address=0x29, sampleTime=0.005):
        self.address = address
        self.sampleTime = sampleTime
        self.bus = smbus.SMBus(bus)
        self.init_sensor()

    def init_sensor(self):
        # Initialization sequence based on VL53L0X datasheet and API
        try:
            # Device reset sequence
            self.write_byte(0x88, 0x00)
            self.write_byte(0x80, 0x01)
            self.write_byte(0xFF, 0x01)
            self.write_byte(0x00, 0x00)
            self.stop_variable = self.read_byte(0x91)
            self.write_byte(0x00, 0x01)
            self.write_byte(0xFF, 0x00)
            self.write_byte(0x80, 0x00)

            # Recommended settings from datasheet
            self.write_byte(0x60, 0x00)
            self.write_byte(0x01, 0xFF)
            self.write_byte(0x02, 0x00)
            self.write_byte(0x16, 0x00)
            self.write_byte(0x17, 0x00)
            self.write_byte(0x31, 0x04)
            self.write_byte(0x40, 0x83)
            self.write_byte(0xFF, 0x01)
            self.write_byte(0x00, 0x00)
            self.write_byte(0x91, self.stop_variable)
            self.write_byte(0x00, 0x01)
            self.write_byte(0xFF, 0x00)
            self.write_byte(0x80, 0x00)

            # Start continuous measurements
            self.write_byte(0x00, 0x04)
            time.sleep(self.sampleTime)
        except Exception as e:
            print("Initialization error: {}".format(e))

    def read_distance(self):
        try:
            # Read measurement data
            self.write_byte(0x00, 0x01)
            time.sleep(0.01)
            data = self.read_bytes(0x14, 12)
            distance = (data[10] << 8) | data[11]
            return distance
        except Exception as e:
            print("Error reading distance: {}".format(e))
            return None

    def write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def read_bytes(self, reg, length):
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def close(self):
        self.bus.close()
