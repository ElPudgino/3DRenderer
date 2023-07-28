import os
import time, traceback
#import meshio
import msvcrt
import math
import pygame
import numpy as np
Map = []
DepthBuffer = []
MapWidth = 1000
MapHeight = 1000
CamOffsetX = MapWidth/2
CamOffsetY = MapHeight/2
CamOffsetZ = MapWidth * 0.35#0.156
ViewOffsetX = MapWidth/2
ViewOffsetY = MapHeight/2
ViewOffsetZ = 60
Input = ''
from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
    K_w,
    K_s,
    K_d,
    K_a,
    K_b,
    K_f,
    K_u
)
pygame.init()


screen = pygame.display.set_mode((MapWidth,MapHeight))
running = True
def UpdateTimer(delay, task):
    next_time = time.time() + delay
    while True:
        time.sleep(max(0, next_time - time.time()))
        try:
            task()
        except Exception:
            traceback.print_exc()
        next_time += (time.time() - next_time) // delay * delay + delay

class Point():
    def __init__(self,X,Y):
        self.X = X
        self.Y = Y
        
class Quaternion():
    def __init__(self,r,i,j,k):
        self.r = r
        self.i = i
        self.j = j
        self.k = k 

    def MultiplyBy(self,q2):
        rt = self.r
        it = self.i
        jt = self.j
        kt = self.k
        #print(self.r,'aaa')
        #print(rt,it,jt,kt,'rijk')
        #print(q2.r,q2.i,q2.j,q2.k,'rijk')
        self.r = rt * q2.r - it * q2.i - jt * q2.j - kt * q2.k
        #print(self.r,'aaas')
        self.i = rt * q2.i + it * q2.r + jt * q2.k - kt * q2.j
        self.j = rt * q2.j - it * q2.k + jt * q2.r + kt * q2.i
        self.k = rt * q2.k + it * q2.j - jt * q2.i + kt * q2.r

        return self
class mesh():
    def __init__(self,Vertices,Edges,Faces):
        self.Vertices = Vertices
        self.Edges = Edges
        self.Faces = Faces

def Update():
    global meshtest
    global DepthBuffer
    #os.system('cls')
    #print("\033[1;1H", end='')
    #if msvcrt.kbhit():
    #    Input = msvcrt.getch().decode("utf-8")
        
   # p1 = Point(27,30)
   # p2 = Point(48,30)
   # p3 = Point(23,11)
   # p4 = Point(7,34)
    
    #ClearMap()
    #DrawLine(p1,p2)
    #DrawLine(p3,p4)
    DepthBuffer = []
    for d in range(MapWidth):
        tb = []
        for dy in range(MapHeight):
            tb.append('a')
        DepthBuffer.append(tb)
    Draw3D(meshtest,[0,0,0],(0,0,255))
    Draw3D(mesh,[0,0,20],(0,0,0))
    #PrintMap()

def ClearMap():
    global Map
    global MapWidth
    global MapHeight
    Map.clear()
    for i in range(MapWidth * MapHeight):
        Map.append(" ")

def PrintMap():
    global Map
    for h in range(MapHeight):
        for w in range(MapWidth):
            print(Map[w + h * MapWidth],end='')
        print("\n",end='')



def InterpolateYs(samplevalues,point1,point2):
    resvalues = []
    for v in samplevalues:
        if point2.X != point1.X:
            resvalues.append(point1.Y + ((point2.Y - point1.Y)*(v - point1.X)/(point2.X - point1.X))) 
    return resvalues

def InterpolateXs(samplevalues,point1,point2):
    resvalues = []
    for v in samplevalues:
        if point2.Y != point1.Y:
            resvalues.append(point1.X + ((point2.X - point1.X)*(v - point1.Y)/(point2.Y - point1.Y)))
       
    return resvalues

def DrawLine(origin,end,color,z1,z2):
    global Map
    global MapWidth
    global MapHeight
    inpointsY = []
    inpointsX = []
    inpointsZ = []
    prpoints = []

    
    if abs(end.X - origin.X) > abs(end.Y - origin.Y):
        length = abs(end.X - origin.X)
        for lY in range(MapWidth):
            if lY in range(origin.X,end.X) or lY in range(end.X,origin.X):
                prpoints.append(lY)
        inpointsY = list(InterpolateYs(prpoints,origin,end))
        inpointsX = prpoints
    else:
        length = abs(end.Y - origin.Y)
        for lY in range(MapHeight):
            if lY in range(origin.Y,end.Y) or lY in range(end.Y,origin.Y):
                prpoints.append(lY)
        inpointsX = list(InterpolateXs(prpoints,origin,end))
        inpointsY = prpoints

    for po in range(len(inpointsX)):
        if end.X - origin.X != 0:
            w1 = (end.X - inpointsX[po]) / (end.X - origin.X)
        else:
            w1 = (end.X - inpointsX[po]) / 0.0001
        w2 = 1 - w1
        zv = z1 * w1 + z2 * w2
        inpointsZ.append(zv)
    
    for p in range(len(inpointsX)):
        if int(inpointsX[p]) < MapWidth and int(inpointsY[p]) < MapHeight:
            #Map[int(inpointsX[p] + int(inpointsY[p]) * MapWidth)] = "X"
            #pygame.draw.circle(screen, (0, 0, 255), (inpointsX[p], inpointsY[p]),1)
            if DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] == 'a' or DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] >= inpointsZ[p]:
                screen.set_at((int(inpointsX[p]),  int(inpointsY[p])), color)
                DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] = inpointsZ[p]  
            
    #if origin.X < MapWidth and origin.Y < MapHeight:
    #    Map[origin.X + origin.Y * MapWidth] = "O"
    #if end.X < MapWidth and end.Y < MapHeight: 
    #    Map[end.X + end.Y * MapWidth] = "O"
    #print(inpointsX,"inpointsX")
    #print(inpointsY,"inpointsY")
    #print(prpoints,"prpoints")

def DistanceFromCamera(Cords):
    global CamOffsetX
    global CamOffsetY
    global CamOffsetZ
    distance = np.square(CamOffsetX - Cords[0] + CamOffsetY - Cords[1] + CamOffsetZ - Cords[2])
    return distance

def DrawTriangle(Vertices,color): #lists of 3 verts and points
    global DepthBuffer
    global CamOffsetX
    global CamOffsetY
    global CamOffsetZ
    global ViewOffsetX
    global ViewOffsetY
    global ViewOffsetZ
    global MapWidth
    global MapHeight
    inpointsY1 = []
    inpointsX1 = []
    inpointsY2 = []
    inpointsX2 = []
    inpointsY3 = []
    inpointsX3 = []
    allpoints1 = []
    allpoints2 = []
    allpoints3 = []
    allpoints = []
    prpoints = []
    Points = []
    for vert in Vertices:
        pX = CamOffsetZ * (vert[0]+ViewOffsetX - CamOffsetX) / (vert[2] + ViewOffsetZ) + CamOffsetX
        pY = CamOffsetZ * (vert[1]+ViewOffsetY - CamOffsetY) / (vert[2] + ViewOffsetZ) + CamOffsetY
        Points.append([pX,pY,vert[2]])

    X1 = Points[0][0]
    X2 = Points[1][0]
    X3 = Points[2][0]
    Y1 = Points[0][1]
    Y2 = Points[1][1]
    Y3 = Points[2][1]
    Z1 = Points[0][2]
    Z2 = Points[1][2]
    Z3 = Points[2][2]
    Dist1 = DistanceFromCamera([X1,Y1,Z1])
    Dist2 = DistanceFromCamera([X2,Y2,Z2])
    Dist3 = DistanceFromCamera([X3,Y3,Z3])
    
    for p in Points:
        p[0] = int(p[0])
        p[1] = int(p[1])
   # print(Points[0][1],Points[1][1],Points[2][1])
    for v in range(3):
        prpoints = []
        if v == 0:
            origin = Point(Points[0][0],Points[0][1])
            end = Point(Points[1][0],Points[1][1])
            if abs(end.X - origin.X) > abs(end.Y - origin.Y):
                length = abs(end.X - origin.X)
                for lY in range(MapWidth):
                    if lY in range(origin.X,end.X) or lY in range(end.X,origin.X):
                        prpoints.append(lY)
                inpointsY1 = list(InterpolateYs(prpoints,origin,end))
                inpointsX1 = prpoints
            else:
                length = abs(end.Y - origin.Y)
                for lY in range(MapHeight):
                    if lY in range(origin.Y,end.Y) or lY in range(end.Y,origin.Y):
                        prpoints.append(lY)
                inpointsX1 = list(InterpolateXs(prpoints,origin,end))
                inpointsY1 = prpoints

            for o in range(len(inpointsX1)):
                allpoints1.append([inpointsX1[o],inpointsY1[o]])
            
        if v == 1:
            origin = Point(Points[1][0],Points[1][1])
            end = Point(Points[2][0],Points[2][1])
            if abs(end.X - origin.X) > abs(end.Y - origin.Y):
                length = abs(end.X - origin.X)
                for lY in range(MapWidth):
                    if lY in range(origin.X,end.X) or lY in range(end.X,origin.X):
                        prpoints.append(lY)
                inpointsY2 = list(InterpolateYs(prpoints,origin,end))
                inpointsX2 = prpoints
            else:
                length = abs(end.Y - origin.Y)
                for lY in range(MapHeight):
                    if lY in range(origin.Y,end.Y) or lY in range(end.Y,origin.Y):
                        prpoints.append(lY)
                inpointsX2 = list(InterpolateXs(prpoints,origin,end))
                inpointsY2 = prpoints
            for o in range(len(inpointsX2)):
                allpoints2.append([inpointsX2[o],inpointsY2[o]])
        
        if v == 2:
            origin = Point(Points[0][0],Points[0][1])
            end = Point(Points[2][0],Points[2][1])
            if abs(end.X - origin.X) > abs(end.Y - origin.Y):
                length = abs(end.X - origin.X)
                for lY in range(MapWidth):
                    if lY in range(origin.X,end.X) or lY in range(end.X,origin.X):
                        prpoints.append(lY)
                inpointsY3 = list(InterpolateYs(prpoints,origin,end))
                inpointsX3 = prpoints
            else:
                length = abs(end.Y - origin.Y)
                for lY in range(MapHeight):
                    if lY in range(origin.Y,end.Y) or lY in range(end.Y,origin.Y):
                        prpoints.append(lY)
                inpointsX3 = list(InterpolateXs(prpoints,origin,end))
                inpointsY3 = prpoints
        
            for o in range(len(inpointsX3)):
                allpoints3.append([inpointsX3[o],inpointsY3[o]]) 

        #DrawLine(origin,end)
    allpoints = allpoints1 + allpoints2 + allpoints3
    maxY = -100000
    minY = 100000
    #print(len(Points),len(allpoints))
    for p in Points:
        if p[1] > maxY:
            maxY = p[1]
            
    
        if p[1] < minY:
            minY = p[1]
   
    #exp1 = Point(0,minY)
    #exp2 = Point(800,minY)
    #DrawLine(exp1,exp2)
    maxY = int(maxY)
    minY = int(minY)

    for l in range(maxY-minY):
      
        syp = []
        maxXpoint = [-10000,0]
        minXpoint = [10000,0]
        for p1 in allpoints:
	    
            if int(p1[1]) == maxY:
        	    syp.append(p1)
			

            
    
        for sp in syp:
            if sp[0] > maxXpoint[0]:
                maxXpoint = sp
   
            if sp[0] < minXpoint[0]:
                minXpoint = sp

        mx = [int(maxXpoint[0]),int(maxXpoint[1]),0]
      
        mi = [int(minXpoint[0]),int(minXpoint[1]),0]

        for p in range(mx[0]-mi[0]):
            W1 = (((Y2 - Y3)*(mx[0] - X3) + (X3 - X2)*(mx[1] - Y3)) / ((Y2 - Y3)*(X1 - X3) + (X3 - X2)*(Y1 - Y3)))
            W2 = (((Y3 - Y1)*(mx[0] - X3) + (X1 - X3)*(mx[1] - Y3)) / ((Y2 - Y3)*(X1 - X3) + (X3 - X2)*(Y1 - Y3)))
            W3 = 1 - W1 - W2
            mx[2] = Z1 * W1 + Z2 * W2 + Z3 * W3
            if DepthBuffer[mx[0] ][mx[1] ] == 'a' or DepthBuffer[mx[0] ][mx[1] ] >= mx[2]:
                screen.set_at((mx[0],  mx[1]), color)
                DepthBuffer[mx[0] ][mx[1] ] = mx[2]  
            mx[0] = mx[0] - 1
        #DrawLine(mi,mx)
        #pygame.draw.line(screen, (50,50,50), (mi.X, mi.Y), (mx.X, mx.Y), 2)
        if maxY - 1 > minY:
            maxY = maxY - 1
def Draw3D(mesh,position,color):
    global Map
    global MapWidth
    global MapHeight
    global Input
    global CamOffsetX
    global CamOffsetY
    global CamOffsetZ
    global ViewOffsetX
    global ViewOffsetY
    global ViewOffsetZ
    Listofpoints = []
    x = 0
    y = 1
    z = 2
    
    

    #for vert in mesh.Vertices:
        
        #vert[x] = vert[x] + position[x]
        #vert[y] = vert[y] + position[y]
        #vert[z] = vert[z] + position[z]

    

    for vert in mesh.Vertices: # get projection of vertices on screen
        if vert[z] + ViewOffsetZ == 0:# no divisions by zero
            vert[z] = vert[z] + 0.0001
        pX = CamOffsetZ * (vert[x]+ViewOffsetX - CamOffsetX) / (vert[z] + ViewOffsetZ) + CamOffsetX
        pY = CamOffsetZ * (vert[y]+ViewOffsetY - CamOffsetY) / (vert[z] + ViewOffsetZ) + CamOffsetY
        #print(vert[x],vert[y])
        Listofpoints.append([pX,pY,vert[z]])
    #print(Listofpoints)
    for edge in mesh.Edges: # draw edges
        pd1 = Point(int(Listofpoints[edge[0]][x]),int(Listofpoints[edge[0]][y]))
        pd2 = Point(int(Listofpoints[edge[1]][x]),int(Listofpoints[edge[1]][y]))
        #print(pd1.X,pd1.Y,pd2.X,pd2.Y)
        DrawLine(pd1,pd2,color,Listofpoints[edge[0]][z],Listofpoints[edge[1]][z])
    for face in mesh.Faces:
        #tri = [Listofpoints[face[0]],Listofpoints[face[1]],Listofpoints[face[2]]]
        #print(Listofpoints)
        tri = [mesh.Vertices[face[0]],mesh.Vertices[face[1]],mesh.Vertices[face[2]]]
        DrawTriangle(tri,color)
    
def RotateMesh(mesh,angle,axis): #angle is float , axis is quaternion
    
    x = 0
    y = 1
    z = 2
    sinus = math.sin(math.radians(angle/2))
    q = Quaternion(math.cos(math.radians(angle/2)),sinus * axis.i,sinus * axis.j,sinus * axis.k)
    q1 = Quaternion(q.r,-q.i,-q.j,-q.k)
    qin = Quaternion(0,0,0,0)
    qv = Quaternion(0,0,0,0)
    for vert in mesh.Vertices:
        q = Quaternion(math.cos(math.radians(angle/2)),sinus * axis.i,sinus * axis.j,sinus * axis.k)
        q1 = Quaternion(q.r,-q.i,-q.j,-q.k)
        qv.r = 0
        qv.i = 0
        qv.j = 0
        qv.k = 0
        
        qv.i = vert[x]
        qv.j = vert[y]
        qv.k = vert[z]
        #print(q.r,qin.r)
        qin = q
        qin.MultiplyBy(qv)
        #print(qv.r,qv.i,qv.j,qv.k)
        #print(qin.r,qin.i,qin.j,qin.k)
        qin.MultiplyBy(q1)
        #print(qv.r,qv.i,qv.j,qv.k)
        vert[x] = qin.i
        #print(vert[x],qv.i)
        vert[y] = qin.j
        vert[z] = qin.k

    return(mesh)
meshtest = mesh([[25,15,15] , [25,15,-15] , [25,-15,15] , [25,-15,-15] ,
                     [-25,15,15] , [-25,15,-15], [-25,-15,15] , [-25,-15,-15]],
                    [[0,1] , [0,4] , [0,2] , [1,3] , [1,5] , [2,3] ,
                    [2,6] , [3,7] , [4,6] , [4,5] , [5,7] , [6,7]],[[0,1,5]])

mesh = mesh([[25,15,50] , [25,15,20] , [25,-15,50] , [25,-15,20] ,
                     [-25,15,50] , [-25,15,20], [-25,-15,50] , [-25,-15,20]],
                    [[0,1] , [0,4] , [0,2] , [1,3] , [1,5] , [2,3] ,
                    [2,6] , [3,7] , [4,6] , [4,5] , [5,7] , [6,7]],[[0,1,5]])
TestAxis = Quaternion(0,0.6666,-0.6666,-0.3333)
XAxis = Quaternion(0,1,0,0)
YAxis = Quaternion(0,0,1,0)

mode = 'mode ' + str(MapWidth) + ',' + str(MapHeight)    
cmd = mode
os.system(cmd)
#UpdateTimer(0.05, Update)
while running:

 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    i = pygame.key.get_pressed()    
    
    if i[K_w]:
        ViewOffsetY = ViewOffsetY - 1
    if i[K_s]:
        ViewOffsetY = ViewOffsetY + 1
    if i[K_a]:
        ViewOffsetX = ViewOffsetX - 1
    if i[K_d]:
        ViewOffsetX = ViewOffsetX + 1
    if i[K_ESCAPE]:
        meshtest = RotateMesh(meshtest,2,TestAxis)
    if i[K_f]:
        ViewOffsetZ = ViewOffsetZ - 1
    if i[K_b]:
        ViewOffsetZ = ViewOffsetZ + 1
    if i[K_UP]:
        meshtest = RotateMesh(meshtest,-2,XAxis)
    if i[K_DOWN]:
        meshtest = RotateMesh(meshtest,2,XAxis)
    if i[K_LEFT]:
        meshtest = RotateMesh(meshtest,2,YAxis)
    if i[K_RIGHT]:
        meshtest = RotateMesh(meshtest,-2,YAxis)
    screen.fill((255, 255, 255))

    
    Update()
    
    pygame.display.flip()

pygame.quit()
