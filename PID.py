import datetime
import csv
import numpy as np


class PID(object):

    def __init__(self, pwm, i2c, v_max, v_min, position, KP=600, KI=0.001, KD=10):
        self.pwm = pwm
        self.i2c = i2c
        self.v_max = v_max
        self.v_min = v_min
        self.KP = KP    # made these initialized variables so GA can work with PID
        self.KI = KI
        self.KD = KD
        self.position_target = position

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

        #normalised_distance_from_top = 1 # used to disable the force normaliser for testing
        force = G * normalised_distance_from_top ** 2
        self.pwm.DC(force)

    def position(self):
        target_position = self.position_target
        i = 0
        error_past = 0
        Integral = 0
        steps = 1000
        location = np.zeros(steps) # for csv data -> will have 1000 datapoints per chromosome
        gain = np.zeros(steps)
        errors = np.zeros(steps)
        G = 0
        peak_limit = 100
        
        current_position = self.i2c.getVoltage()
        location[i] = current_position
        
        while i < steps: # i > 3 ensures code doesn't stop at the start
            
            current_position = self.i2c.getVoltage()
            location[i] = current_position
            error = (current_position - target_position)/ target_position

            V = error - error_past
            D = self.KD * V

            P = error * self.KP
            if G != peak_limit:
                Integral += error
            I = Integral * self.KI

            G = P + I + D

            if G > peak_limit:
                G = peak_limit
            elif G < 0:
                G = 0

            self.force_normaliser(location[i], G)
            gain[i] = G
            errors[i] = error

            i +=1
            error = error_past

        print('peak gain in chromosome is {}'.format(max(gain)))

        return errors

#       KP = 600    # Emil's PID parameter values... should probably keep these just in case!
#       KI = 0.001
#       KD = 10

        


        # length = np.transpose(np.linspace(0,999,1000))
        # location = np.transpose(location)
        # myData = [length, location]  
        # date = datetime.datetime.now().strftime("%H-%M-%S-%B-%d-%Y")  
     #        filename = './logs/PID-Response-' + date +'.csv'
  #               myFile = open(filename, 'w')  
        # with myFile:  
  #             writer = csv.writer(myFile)
  #             writer.writerows(myData)