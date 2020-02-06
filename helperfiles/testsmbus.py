from smbus import SMBus
bus = SMBus(1)
data = [30, 170, 170, 22]
bus.write_i2c_block_data(8, 0, data)
bus.close()


import time
import picamera

with picamera.PiCamera() as camera:
    camera.resolution = (1024, 768)
    camera.start_preview()
    # Camera warm-up time
    time.sleep(2)
    camera.capture('foo.jpg')
