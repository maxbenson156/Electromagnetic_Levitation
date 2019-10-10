import datetime
import csv
import numpy as np


class PID(object):

	def __init__(self, pwm, i2c, v_max, v_min):
		self.pwm = pwm
		self.i2c = i2c
		self.v_max = v_max
		self.v_min = v_min

	def force_normaliser(self, location, G):
		'''As the attractive force depends on the distance from the magnet, the force should be normalised to compensate. 
		The force of the electromagnet is dependent on the square of the distance the maximum current should be proportional to the max distance
		'''

	#checking the reading for errors
		if location < self.v_min - 0.5:
			raise ValueError(location, self.v_min, 'position read as above the maximum value, probably problem with light source.')
		operation_range = self.v_max - self.v_min
	#top location is v_min, bottom location is v_max (because a higher position means a larger shadow on the PV cell.)
		normalised_distance_from_top = (location - self.v_min) / operation_range
		if normalised_distance_from_top > 1.5:
			raise ValueError(normalised_distance_from_top, 'NDFT is greater than 1')
		elif normalised_distance_from_top > 1:
			normalised_distance_from_top = 1
		force = G * normalised_distance_from_top ** 2
		self.pwm.DC(force)

		return force

	def logger(self, timestep, location, force):
		location = np.transpose(location)
		myData = [timestep, location, force]  
		date = datetime.datetime.now().strftime("%H-%M-%S-%B-%d-%Y")	
		filename = './logs/PID-Response-' + date +'.csv'
		myFile = open(filename, 'w')  
		with myFile:  
			writer = csv.writer(myFile)
			writer.writerows(myData)


	def position(self, position):
		target_position = position
		i = 0
		error_past = 0
		Integral = 0
		location = np.zeros(1000)
		G = 0
		peak_limit = 100
		force = np.zeros(1000)
		while i < 1000:
			location[i] = self.i2c.getVoltage()
			error = (location[i] - target_position)/ target_position

			KP = 600
			KI = 0.001
			KD = 10

			V = error - error_past
			D = KD * V

			P = error * KP
			if G != peak_limit:
				Integral += error
				I = Integral * KI

			G = P + I + D

			if G > peak_limit:
				G = peak_limit
			elif G < 0:
				G = 0

			force[i] = self.force_normaliser(location[i], G)

			i +=1
			error  = error_past
		timestep = np.transpose(np.linspace(0,999,1000))
		self.logger(timestep, location, force)



