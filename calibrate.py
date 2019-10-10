import i2c
import time


class Calibrate(object):
	def __init__(self, i2c, pwm):
		self.i2c = i2c
		self.pwm = pwm

	def setup(self):
		v_max = self.bottom()
		v_min = self.top()

		return v_min, v_max

	def bottom(self):
		self.pwm.DC(20)
		v_min = self.average()
		self.pwm.DC(0)

		return v_min

	def top(self):
		self.pwm.DC(100)
		time.sleep(2)
		v_min = self.average()
		self.pwm.DC(0)

		return v_min

	def average(self):
		time.sleep(2)
		v_1 = self.i2c.getVoltage()
		time.sleep(0.1)
		v_2 = self.i2c.getVoltage()
		time.sleep(0.1)
		v_3 = self.i2c.getVoltage()
		v_ave = ( v_1 + v_2 + v_3 ) / 3

		max_difference = 0.02

		if ( abs(v_ave - v_1) > max_difference or abs(v_ave - v_2) > max_difference or abs(v_ave - v_2) > max_difference):
			raise ValueError('Signal to noise ratio too big, try blocking out lightbulbs')

		return v_ave
			



if __name__ == '__main__':
	import i2c
	import controller
	import calibrate
	
	cal = Calibrate(i2c, pwm)
