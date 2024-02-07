import numpy as np;
import matplotlib.pyplot as plt;
import matplotlib.cm as cm


x = y = np.arange(0, 10.0, 0.25)
X, Y = np.meshgrid(x, y)

cmax=9;
gamma=1
p1=[1,5]
p2=[7,3]

p = [[2,3],[2,7],[5,3],[5,7],[8,3],[8,7]]

Z = 1+cmax*np.exp((-(X-p[0][0])**2-(Y-p[0][1])**2)/gamma)
for i in range(1,len(p)):
	Z = Z + 1+cmax*np.exp((-(X-p[i][0])**2-(Y-p[i][1])**2)/gamma)


fig, ax = plt.subplots()
ax.imshow(Z,cmap='gist_gray',extent=[0,10,0,10],origin='lower')

plt.xticks(np.arange(0, 10, 1.0))
plt.yticks(np.arange(0,10, 1.0))

data=np.loadtxt("Data/states.txt", 	delimiter=',')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'.b',markersize=2)

for i in range(len(data)):

	ax.plot([data[i][0],data[i][2]],[data[i][1],data[i][3]],'g',lw=0.6)

plt.plot(data[0][0],data[0][1],lw=.9,color='g')

data=np.loadtxt("Data/solution.txt", 	delimiter=',')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'m',label="Path", linewidth=2.)

ax.plot(obx[0],oby[0],'.b',markersize=20,label='Start')
ax.plot(obx[-1],oby[-1],'.r',markersize=20,label='Goal')

ax.axis('tight')
ax.legend(numpoints=1,loc='upper right')
plt.savefig("realvectorspace.png",bbox_inches='tight')

plt.show()
