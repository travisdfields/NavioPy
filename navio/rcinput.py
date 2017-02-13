class RCInput():
	CHANNEL_COUNT = 6
	channels = []
	

	def __init__(self):
		for i in range(0, self.CHANNEL_COUNT):
			try:
				f = open("/sys/kernel/rcio/rcin/ch%d" % i, "r")
				self.channels.append(f)
			except: 
				print ("Can't open file /sys/kernel/rcio/rcin/ch%d" % i)
    
	def read(self, ch):
		value = self.channels[ch].read()
		position = self.channels[ch].seek(0, 0)
		return value[:-1]
        
	def read_all(self):
		rc_in = [0,0,0,0,0,0]
		for i in range(0,self.CHANNEL_COUNT):
			rc_in[i] = self.channels[i].read()
			position = self.channels[i].seek(0,0)
		return rc_in
