import matplotlib.pyplot as plt
import numpy as np
  
x = np.linspace(0, 10*np.pi, 100)
y = np.sin(x)
  
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(10,10 , 'b-')
teste = 0
for phase in np.linspace(0, 10*np.pi, 100):
    line1.set_ydata(0.1 + teste)
    teste = 0.1 + teste
    fig.canvas.draw()
    fig.canvas.flush_events()
