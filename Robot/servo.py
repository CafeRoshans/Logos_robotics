
from builtins import hex, eval, int, object
from xr_i2c import I2c
import os

i2c = I2c()
import xr_config as cfg

from xr_configparser import HandleConfig
path_data = os.path.dirname(os.path.realpath(__file__)) + '/data.ini'
cfgparser = HandleConfig(path_data)


class Servo(object):
	def __init__(self):
		pass

	def angle_limit(self, angle):
		if angle > cfg.ANGLE_MAX:
			angle = cfg.ANGLE_MAX
		elif angle < cfg.ANGLE_MIN:
			angle = cfg.ANGLE_MIN
		return angle

	def set(self, servonum, servoangle):
		angle = self.angle_limit(servoangle)
		buf = [0xff, 0x01, servonum, angle, 0xff]
		try:
			i2c.writedata(i2c.mcu_address, buf)
		except Exception as e:
			print('servo write error:', e)

	def store(self):
		cfgparser.save_data("servo", "angle", cfg.ANGLE)

	def restore(self):
		cfg.ANGLE = cfgparser.get_data("servo", "angle")
		for i in range(0, 8):
			cfg.SERVO_NUM = i + 1
			cfg.SERVO_ANGLE = cfg.ANGLE[i]
			self.set(i + 1, cfg.ANGLE[i])
