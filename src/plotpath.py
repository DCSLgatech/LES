import numpy as np;
import matplotlib.pyplot as plt;

from Grid import Grid

grid=Grid(100,100,100,100)

obstacles=[]
#
# ob1=grid.quadObs2([0,2/2.],[8.5/2.,1/2.])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([-4/2.,8/2.],[1/2.,2.5/2.])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([-4/2.,4.5/2.],[4/2.,2/2.	])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([7/2.,3.5/2.],[2.5/2.,11/2.])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([3/2.,5/2.],[2.5/2.,2/2.])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([-6/2.,-4/2.],[3/2.,5/2.])
# obstacles.append(ob1)
#
# ob1=grid.quadObs2([3/2.,-5/2.],[2.5/2.,5/2.])
# obstacles.append(ob1)

# xc=[[-3,-3],[-3,-1],[-3,1],[-3,3],
#     [-2,-3],[-2,-1],[-2,1],[-2,3],
#     [-1,-3],[-1,-1],[-1,1],[-1,3],
#     [0,-3],[0,-1],[0,1],[0,3],
#     [1,-3], [1,-1],[1,1],[1,3],
#     [2,-3],[2,-1],[2,1],[2,3],
#     [3,-3],[3,-1],[3,1],[3,3]]

xc=[[-3,-3],[-3,0],[-3,3],
    [-2,-3],[-2,0],[-2,3],
    [-1,-3],[-1,0],[-1,3],
    [0,-3],[0,0],[0,3],
    [1,-3], [1,0],[1,3],
    [2,-3],[2,0],[2,3],
    [3,-3],[3,0],[3,3]]
size=0.8

for i in range(len(xc)):
    obs=grid.quadObs2(xc[i],[size,size])
    obstacles.append(obs)

fig=plt.figure()
plt.axis((-5,5,-5,5))
plt.xticks(np.arange(-5, 5, 1.0))
plt.yticks(np.arange(-5, 5, 1.0))
# plt.autoscale(enable=True, tight=True)

ax = fig.add_subplot(111)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Many Homotopy Classes World')

for i in range(len(obstacles)):
    obx=[x[0] for x in obstacles[i]]
    oby=[x[1] for x in obstacles[i]]
    if i==0:
        ax.plot(obx,oby,'-r',label="Obstacles")
    else:
        ax.plot(obx,oby,'-r')


# data=np.loadtxt("Data/states.txt", 	delimiter=',')
#
# obx=[x[0] for x in data]
# oby=[x[1] for x in data]
# ax.plot(obx,oby,'.b',markersize=2)

# for i in range(len(data)):
# 	ax.plot([data[i][0],data[i][2]],[data[i][1],data[i][3]],'g',lw=0.5)
#
# plt.plot(data[0][0],data[0][1],linewidth=.7,color='g')

data=np.loadtxt("Data/solution_les.txt", 	delimiter=',')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'m',label="LES Path", linewidth=2.)

data=np.loadtxt("Data/solution_drrt.txt", 	delimiter=',')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'k',label="DRRT Path", linewidth=2.)

ax.plot(obx[0],oby[0],'.b',markersize=20,label='Start')
ax.plot(obx[-1],oby[-1],'.r',markersize=20,label='Goal')

ax.axis('tight')
ax.legend(numpoints=1,loc='upper right')
plt.savefig("many_homotopy_world.png",bbox_inches='tight')

plt.show()
