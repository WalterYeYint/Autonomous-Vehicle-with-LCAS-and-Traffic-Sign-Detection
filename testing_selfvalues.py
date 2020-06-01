class draft(object):
	def __init__(self):
		self.value = 90

	def update(self):
		self.value = self.value + 100
		print("in update, value = ", format(self.value))

	def read(self):
		return self.value

i = 0
a = draft()

while i < 3:
	a.update()
	i = i + 1

print("After everything, value = ", format(a.read()))