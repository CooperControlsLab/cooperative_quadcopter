import numpy as n
import matplotlib.pyplot as p
import scipy as sci


t = n.linspace(0,2,2001)
x = n.cos(n.pi*t)
p.plot(t,x)
p.plot(t,t)
p.xlabel('Pi Radians'),p.ylabel('Displacement (m)'),p.title('Hello')
p.grid()
p.show()
a = n.array([[1,2,3],[4,5,6]])
b = a.reshape(3,2)

print(a)
print(b)

