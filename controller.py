import RPi.GPIO as GPIO


class PWM(object):
	def __init__(self):
		GPIO.setmode(GPIO.BOARD)
		self.pwm_pin = 12
		self.pwm_freq = 10000
		GPIO.setup(self.pwm_pin, GPIO.OUT)
		self.p = GPIO.PWM(self.pwm_pin, self.pwm_freq)
		self.p.start(0)

	def DC(self, dutyCycle):
		self.p.ChangeDutyCycle(dutyCycle)

	def cleanup(self):
		self.p.ChangeDutyCycle(0)
		self.p.stop()
		GPIO.cleanup()
