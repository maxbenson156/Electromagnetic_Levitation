import smbus
import time


class I2C(object):

	def __init__(self):
		self.address = 0x04
		self.bus = smbus.SMBus(1)
		self.value = 1
		
	def getVoltage(self):
		flag = 0
		while flag == 0:
			try:
				self.writeNumber(self.value)
				number = self.readNumber()
				temp = ''.join(str(x) for x in number)
				voltage = float(temp)/ 1000
				flag = 1
			except:
				pass




		return voltage

	def writeNumber(self, value):
		self.bus.write_byte(self.address, value)
	# bus.write_byte_data(address, 0, value)
		return -1

	def readNumber(self):
	    #number = bus.read_byte(address)
	    number = self.bus.read_i2c_block_data(self.address, 0, 4)
	    #number = bus.read_byte_data(address, 1)
	    return number
