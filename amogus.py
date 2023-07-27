import os
import time, traceback
#import meshio
import msvcrt
import math
Map = []
MapWidth = 200
MapHeight = 120
CamOffsetX = MapWidth/2
CamOffsetY = MapHeight/2
CamOffsetZ = MapWidth * 0.156
ViewOffsetX = 100
ViewOffsetY = 60
ViewOffsetZ = 42
Input = ''

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
    os.system('cls')
    print("\033[1;1H", end='')
    if msvcrt.kbhit():
        Input = msvcrt.getch().decode("utf-8")
        
    p1 = Point(27,30)
    p2 = Point(48,30)
    p3 = Point(23,11)
    p4 = Point(7,34)
    
    ClearMap()
    #DrawLine(p1,p2)
    #DrawLine(p3,p4)
    Draw3D(meshtest,[0,0,0])
    PrintMap()

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

def DrawLine(origin,end):
    global Map
    global MapWidth
    global MapHeight
    inpointsY = []
    inpointsX = []
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
    
    
    for p in range(len(inpointsX)):
        if int(inpointsX[p]) < MapWidth and int(inpointsY[p]) < MapHeight and Map[int(inpointsX[p] + int(inpointsY[p]) * MapWidth)] != "O":
            Map[int(inpointsX[p] + int(inpointsY[p]) * MapWidth)] = "X"
    if origin.X < MapWidth and origin.Y < MapHeight:
        Map[origin.X + origin.Y * MapWidth] = "O"
    if end.X < MapWidth and end.Y < MapHeight: 
        Map[end.X + end.Y * MapWidth] = "O"
    #print(inpointsX,"inpointsX")
    #print(inpointsY,"inpointsY")
    #print(prpoints,"prpoints")

def Draw3D(mesh,position):
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

    

    for vert in mesh.Vertices:
        vert[x] = vert[x] + position[x]
        vert[y] = vert[y] + position[y]
        vert[z] = vert[z] + position[z]

    

    for vert in mesh.Vertices: # get projection of vertices on screen
        #if vert[z] == 0:
            #vert[z] = 0.0001
        pX = CamOffsetZ * (vert[x]+ViewOffsetX - CamOffsetX) / (vert[z] + ViewOffsetZ) + CamOffsetX
        pY = CamOffsetZ * (vert[y]+ViewOffsetY - CamOffsetY) / (vert[z] + ViewOffsetZ) + CamOffsetY
        #print(vert[x],vert[y])
        Listofpoints.append([pX,pY])
    #print(Listofpoints)
    for edge in mesh.Edges: # draw edges
        pd1 = Point(int(Listofpoints[edge[0]][x]),int(Listofpoints[edge[0]][y]))
        pd2 = Point(int(Listofpoints[edge[1]][x]),int(Listofpoints[edge[1]][y]))
        #print(pd1.X,pd1.Y,pd2.X,pd2.Y)
        DrawLine(pd1,pd2)
   
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
                    [2,6] , [3,7] , [4,6] , [4,5] , [5,7] , [6,7]],[])
TestAxis = Quaternion(0,0.6666,-0.6666,-0.3333)
        
mode = 'mode ' + str(MapWidth) + ',' + str(MapHeight)    
cmd = mode
os.system(cmd)
#UpdateTimer(0.05, Update)
while True:
    Update()
    i = input()
    if i == 'w':
        ViewOffsetY = ViewOffsetY - 5
    if i == 's':
        ViewOffsetY = ViewOffsetY + 5
    if i == 'a':
        ViewOffsetX = ViewOffsetX - 5
    if i == 'd':
        ViewOffsetX = ViewOffsetX + 5
    if i == 'f':
        ViewOffsetZ = ViewOffsetZ - 4
    if i == 'b':
        ViewOffsetZ = ViewOffsetZ + 4
    if i == 'x':
        meshtest = RotateMesh(meshtest,15,TestAxis)
        
