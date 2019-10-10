import time
import RPi.GPIO as GPIO
import curses
import i2c
import controller
import calibrate
import PID


def range_check(value):
	if value > 100:
		value = 100
	elif value < 0:
		value = 0
	return value


screen = curses.initscr()
screen.refresh()
curses.cbreak()
screen.keypad(True)
value = 1

i2c = i2c.I2C()
pwm = controller.PWM()
v_min = v_max = 0

try:
	screen.addstr('press any key to start')
	key = 0

	while (key != 27): #Escape key
		key = screen.getch()
		voltage = i2c.getVoltage()

		if key == curses.KEY_UP:
			value = value + 1
		elif key == curses.KEY_RIGHT:
			value = value + 10
		elif key == curses.KEY_DOWN:
			value = value - 1
		elif key == curses.KEY_LEFT:
			value = value - 10
		elif key == curses.KEY_ENTER or key == 10 or key == 13:
			value = 0 
		elif key == 115: #letter s
			cal = calibrate.Calibrate(i2c, pwm)
			v_min, v_max = cal.setup()
			screen.addstr(str(v_max))
			time.sleep(1)
		elif key == 32: #SPACEBAR
			location = (v_min + v_max) / 2
			pid = PID.PID(pwm, i2c, v_max, v_min)
			pid.position(location)

		value = range_check(value)
		pwm.DC(value)

		screen.clear()
		screen.addstr('percentage power = ')
		screen.addstr(str(value))
		screen.addstr('       ')
		screen.addstr('sensor voltage = ')
		screen.addstr(str(voltage))
		screen.addstr('      ')

		#screen.addstr(str(key))

except Exception as e:
	curses.endwin()
	pwm.cleanup()
	raise

curses.endwin()
pwm.cleanup()

# import click

# while (1):
# 	a = click.getchar()
# 	print(a)
# 	if a == ' ':
# 		break
