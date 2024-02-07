import math
import numpy as np
import random

class Grid:
    def __init__(self,length,breadth,Nx,Ny):
        self.length=length
        self.breadth=breadth
        self.Nx=Nx
        self.Ny=Ny
        self.clearance=.05 # in metres
        self.gridmat=[[0 for col in range(self.Nx)] for row in range(self.Ny)]
        self.obstacles=[]

        self.move=  [[-1, 0 ], #
                     [-1,-1 ], #
                     [ 0,-1 ], #
                     [ 1,-1 ], #
                     [ 1, 0 ], #
                     [ 1, 1 ], #
                     [ 0, 1 ], #
                     [-1, 1 ]] #

    def coord2grid(self,x,y):
        x_min=self.length/float(self.Nx)
        y_min=self.length/float(self.Ny)

        m=math.floor(y/y_min)
        n=math.floor(x/x_min)

        if x==self.length:
            n=self.Nx-1
        if y==self.breadth:
            m=self.Ny-1

        return [int(m),int(n)]

    def grid2coord(self,m,n):
        x_min=self.length/float(self.Nx);
        y_min=self.breadth/float(self.Ny);

        y=m*y_min;
        x=n*x_min;
        return [x,y]

    def setStart(self,startp):
        self.init=startp
        grid_coord=self.coord2grid(self.init[0],self.init[1])
        self.gridinit=grid_coord

    def setgridStart(self,startp):
        self.gridinit=startp

    def setGoal(self,goalp):
        self.goal=goalp
        grid_coord=self.coord2grid(self.goal[0],self.goal[1])
        self.gridgoal=grid_coord

    def setgridGoal(self,goalp):
        self.gridgoal=goalp

    def setClearance(self,clr):
        self.clearance=clr

    def distance(self,x1,y1,x2,y2):
        return ((x1-x2)**2+(y1-y2)**2)**0.5

    def heuristic(self):
        heuristic=[[0 for col in range(self.Nx)] for row in range(self.Ny)]
        gridgoal=self.coord2grid(self.goal[0],self.goal[1])

        for i in range(self.Ny):
            for j in range(self.Nx):
                heuristic[i][j]=self.distance(i,j,gridgoal[0],gridgoal[1])
        return heuristic

    def setObstacles(self,obstaclelist):
        x_min=self.length/float(self.Nx)

        self.obstacles.append(obstaclelist)
        for i in range(len(obstaclelist)):
            gridobs=self.coord2grid(obstaclelist[i][0],obstaclelist[i][1])
            if x_min<=self.clearance:
                #print '\nclearance required\n'
                grid_clearance=math.ceil(self.clearance/float(x_min))
                for k in np.arange(1,grid_clearance+1):
                    for j in range(len(self.move)):
                        row=gridobs[0]+int(k)*self.move[j][0]
                        col=gridobs[1]+int(k)*self.move[j][1]
                        if row >= 0 and row < len(self.gridmat) and col >=0 and col < len(self.gridmat[0]):
                            self.gridmat[row][col]=1
            self.gridmat[gridobs[0]][gridobs[1]]=1

    def lineObs(self,x1,y1,x2,y2):

        x_min=self.length/float(self.Nx)
        y_min=self.breadth/float(self.Ny)
        obstaclelist=[]
        if x1==x2:
            for i in np.arange(0,abs(y2-y1),y_min/10):
                y_new=y1+i*(y2-y1)/abs(y2-y1)
                obstaclelist.append([x1,y_new])
            obstaclelist.append([x1,y2])
        else:
            slope=(y2-y1)/float((x2-x1))
##            for i in range(int(abs(x2-x1)/x_min)*50):
            for i in np.arange(0,abs(x2-x1),x_min/10):
                x_new=x1+i*(x2-x1)/abs(x2-x1)
                y_new=y1+slope*(x_new-x1)
                obstaclelist.append([x_new,y_new])
            obstaclelist.append([x2,y2])
        return obstaclelist

    def circleObs(self,xc,yc,r):
        x_min=self.length/float(self.Nx)
        delta_theta=x_min/r
        obstaclelist=[]
        for ri in np.arange(0,r,4*x_min):
            for theta in np.arange(0,2*np.pi,delta_theta):
                x=xc+ri*np.cos(theta)
                y=yc+ri*np.sin(theta)
                obstaclelist.append([x,y])
        return obstaclelist

    def quadObs(self,x1,y1,x2,y2,x3,y3,x4,y4):
        obstaclelist=self.lineObs(x1,y1,x2,y2)+self.lineObs(x2,y2,x3,y3)+self.lineObs(x3,y3,x4,y4)+self.lineObs(x4,y4,x1,y1)
        return obstaclelist

    def quadObs2(self,center,size):

        x1=center[0]-size[0]/2.
        y1=center[1]-size[1]/2.

        x2=center[0]+size[0]/2.
        y2=y1

        x3=x2
        y3=center[1]+size[1]/2.

        x4=x1
        y4=y3

        print (x1,y1,x2,y2,x3,y3,x4,y4)

        return self.quadObs(x1,y1,x2,y2,x3,y3,x4,y4)


    def randomObs(self,no):
        obstaclelist=[]
        for i in range(no):
            x=self.length*random.random()
            y=self.breadth*random.random()
            obstaclelist.append([x,y])
        return obstaclelist


    def getMatrix(self):
        x_min=self.length/float(self.Nx)
        for obstacle in range(len(self.obstacles)):
            for i in range(len(self.obstacles[obstacle])):
                gridobs=self.coord2grid(self.obstacles[obstacle][i][0],self.obstacles[obstacle][i][1])
                if x_min<=self.clearance:
                    #print '\nclearance required\n'
                    grid_clearance=math.ceil(self.clearance/float(x_min))
                    for k in np.arange(1,grid_clearance+1):
                        for j in range(len(self.move)):
                            row=gridobs[0]+int(k)*self.move[j][0]
                            col=gridobs[1]+int(k)*self.move[j][1]
                            if row >= 0 and row < len(self.gridmat) and col >=0 and col < len(self.gridmat[0]):
                                self.gridmat[row][col]=1
                self.gridmat[gridobs[0]][gridobs[1]]=1

        return self.gridmat



    def getStart(self):
        return self.init

    def getGoal(self):
        return self.goal

    def getObstacles(self):
        return self.obstacles

    def getClearance(self):
        return self.clearance

    def getParams(self):
        return [self.length,self.breadth,self.Nx,self.Ny]

    def gridParams(self):

        print ('**************************************************')
        # print '\nGrid Parameters are as follows\n'
        # print 'Length in metres:',self.length
        # print '\nbreadth in metres:',self.breadth
        # print '\nNumber of nodes:', self.Nx*self.Ny
        #
        # try:
        #     heuristic=self.heuristic()
        #     print '\nheuristic function\n'
        #     for i in range(len(heuristic)):
        #         print heuristic[i]
        # except AttributeError:
        #         print '\nGoal not set'
        #
        # try:
        #     gridmat=self.getMatrix()
        #     print '\nGrid matrix:\n'
        #     for i in range(len(gridmat)):
        #         print gridmat[i]
        # except AttributeError:
        #         print '\nObstacles not set'
        #
        # try:
        #     print '\nstart point:', self.init
        #     print '\ngoal point:',self.goal
        # except AttributeError:
        #         print '\nstart point not set'
        #
        # print '\n**************************************************'
