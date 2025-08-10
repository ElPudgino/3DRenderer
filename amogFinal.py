import os
import time, traceback
#import meshio
import math
import pygame
import numpy as np
zbufferactive = True
facesactive = False
Map = []
DepthBuffer = []
MapWidth = 1200
MapHeight = 1200
CamOffsetX = MapWidth/2
CamOffsetY = MapHeight/2
CamOffsetZ = MapWidth * 0.35 # Расстояние камеры от плоскости проекции
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
    K_u,
    K_p,
    K_c,
    K_1,
    K_2,
    K_3,
    K_4
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
    def __init__(self,r,i,j,k): # Кватернион состоит из действительной и векторной части
        self.r = r
        self.i = i # i*i = -1
        self.j = j # j*j = -1
        self.k = k # k*k = -1
                   # i*j*k = -1
                   # из этого следует, что
                   # j*k = i, i*k = j, i*j = k                   
                
                   # Умножение кватернионов не коммутативно, то есть
                   # при перестановке мест множителей произведение меняется

                   # если j*k = i , то k*j = -i и так для всех остальных
                   
    def MultiplyBy(self,q2):
        rt = self.r
        it = self.i
        jt = self.j
        kt = self.k
      
        self.r = rt * q2.r - it * q2.i - jt * q2.j - kt * q2.k 
        self.i = rt * q2.i + it * q2.r + jt * q2.k - kt * q2.j 
        self.j = rt * q2.j - it * q2.k + jt * q2.r + kt * q2.i 
        self.k = rt * q2.k + it * q2.j - jt * q2.i + kt * q2.r

        return self
    
class mesh():
    def __init__(self,Vertices,Edges,Faces,Rotation):
        self.Vertices = Vertices
        self.Edges = Edges
        self.Faces = Faces
        self.Rotation = Rotation

def Update():
    global meshtest
    global DepthBuffer
    global mesh2
    global res
    global time
    DepthBuffer = []

    for y in range(res):
        for x in range(res): # Анимация волны через тригонометрию
            val = math.sin(y * 4 + time) + math.cos(x * 2 + time/2) / 4
            mesh2.Vertices[y * res + x][1] = val * 8 
              
    for d in range(MapWidth):
        tb = []
        for dy in range(MapHeight):
            tb.append('a')
        DepthBuffer.append(tb) 
        
    Draw3D(activemesh,[0,0,0],[180,180,180])
    

def InterpolateYs(samplevalues,point1,point2):
    resvalues = []
    for v in samplevalues:
        if point2.X != point1.X:
            resvalues.append(point1.Y + ((point2.Y - point1.Y)*(v - point1.X)/(point2.X - point1.X))) # Линейная интерполяция между двумя точками для отрисовки рёбер
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
            if DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] == 'a' or DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] >= inpointsZ[p]:
                screen.set_at((int(inpointsX[p]),  int(inpointsY[p])), color)
                DepthBuffer[int(inpointsX[p])][int(inpointsY[p])] = inpointsZ[p]  
            
    
def DistanceFromCamera(Cords):
    global CamOffsetX
    global CamOffsetY
    global CamOffsetZ
    distance = np.square(CamOffsetX - Cords[0] + CamOffsetY - Cords[1] + CamOffsetZ - Cords[2]) # Расстояние между камерой и точкой для буфера глубины
    return distance



def DrawTriangle(Vertices,color): 
    global DepthBuffer
    global CamOffsetX
    global CamOffsetY
    global CamOffsetZ
    global ViewOffsetX
    global ViewOffsetY
    global ViewOffsetZ
    global MapWidth
    global zbufferactive
    global LightSourcePos
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
    Center = [0,0,0]
    for vert in Vertices:
        pX = CamOffsetZ * (vert[0]+ViewOffsetX - CamOffsetX) / (vert[2] + ViewOffsetZ) + CamOffsetX # Проецирование на плоскость с перспективой
        pY = CamOffsetZ * (vert[1]+ViewOffsetY - CamOffsetY) / (vert[2] + ViewOffsetZ) + CamOffsetY
        Points.append([pX,pY,vert[2]])
    colortemp = [0,0,0]
    

    l = -2 # Вектор освещения
    m = 3
    n = 3

   
    
    vect1 = Quaternion(0,Vertices[1][0] - Vertices[0][0],Vertices[1][1] - Vertices[0][1],Vertices[1][2] - Vertices[0][2]) 
    vect2 = Quaternion(0,Vertices[2][0] - Vertices[0][0],Vertices[2][1] - Vertices[0][1],Vertices[2][2] - Vertices[0][2])
   
    vect1.MultiplyBy(vect2) # Умножение кватернионов без действительной части то же самое, что и векторное произведение векторов
   

    plane = [vect1.i,vect1.j,vect1.k,0] 
    
    plane[3] = 0 - (vect1.i + vect1.j + vect1.k) # Уравнение плоскости 

    
    

    AngleSine = abs(plane[0] * l + plane[1] * m + plane[2] * n) / (np.sqrt(plane[0]**2 + plane[1]**2 + plane[2]**2) * np.sqrt(l**2 + n**2 + m**2))
    # синус угла между прямой и плоскостью из их уравнений
   

    colortemp[0] = color[0] * AngleSine # Цвет темнее или светлее в зависимости от значения синуса
    colortemp[1] = color[1] * AngleSine
    colortemp[2] = color[2] * AngleSine
    
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

       
    allpoints = allpoints1 + allpoints2 + allpoints3
    maxY = -100000
    minY = 100000
    
    for p in Points:
        if p[1] > maxY:
            maxY = p[1]
            
    
        if p[1] < minY:
            minY = p[1]
   
   
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
            if zbufferactive:
                W1 = (((Y2 - Y3)*(mx[0] - X3) + (X3 - X2)*(mx[1] - Y3)) / ((Y2 - Y3)*(X1 - X3) + (X3 - X2)*(Y1 - Y3))) # Интерполяция треугольника 
                W2 = (((Y3 - Y1)*(mx[0] - X3) + (X1 - X3)*(mx[1] - Y3)) / ((Y2 - Y3)*(X1 - X3) + (X3 - X2)*(Y1 - Y3))) # на основе барицентрических координат 
                W3 = 1 - W1 - W2                                                                                       # mx[0] - координата x точки, mx[1] - координата y точки
                mx[2] = Z1 * W1 + Z2 * W2 + Z3 * W3       
                
                try:
                    if DepthBuffer[mx[0] ][mx[1] ] == 'a' or DepthBuffer[mx[0] ][mx[1] ] > mx[2]:
                        screen.set_at((mx[0],  mx[1]), colortemp)
                        DepthBuffer[mx[0] ][mx[1] ] = mx[2]  
                    mx[0] = mx[0] - 1
                except:
                    mx[0] = mx[0] - 1
            else:
                screen.set_at((mx[0],  mx[1]), colortemp)
                mx[0] = mx[0] - 1
            
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
    global facesactive
    
    Listofpoints = []
    x = 0
    y = 1
    z = 2
    
    
    
   
    newverts = []
    

    for vert in mesh.Vertices: 

        q = mesh.Rotation 
        
        q1 = Quaternion(q.r,-q.i,-q.j,-q.k) # Формула для вращения q * p * q^-1
                                            # q = cos(x/2) + sin(x/2)(g)
                                            # g - ось вращения 
                                            # Модуль векторной части кватерниона оси вращения должени быть равен единице,
                                            # а действительная часть должна быть равна нулю
                                            
        
        qv = Quaternion(0,vert[x],vert[y],vert[z]) # Вектор или координаты точки могут быть записан как кватернион с нулевой действительной частью
        
        qcopy = Quaternion(q.r,q.i,q.j,q.k)
        
        qcopy.MultiplyBy(qv) 
                            
        
        qcopy.MultiplyBy(q1)

        newvert = [qcopy.i,qcopy.j,qcopy.k] # Новые координаты точки
        
        if newvert[z] + ViewOffsetZ == 0:# т.к. в проецировании есть деление нужно сделать знаменатель не нулевым
            newvert[z] = newvert[z] + 0.0001
        pX = CamOffsetZ * (newvert[x]+ViewOffsetX - CamOffsetX) / (newvert[z] + ViewOffsetZ) + CamOffsetX # Проекция вершин на экран
        pY = CamOffsetZ * (newvert[y]+ViewOffsetY - CamOffsetY) / (newvert[z] + ViewOffsetZ) + CamOffsetY

        newverts.append(newvert)
        Listofpoints.append([pX,pY,newvert[z]])
   
    for edge in mesh.Edges: # Грани
        try:
            pd1 = Point(int(Listofpoints[edge[0]][x]),int(Listofpoints[edge[0]][y]))
            pd2 = Point(int(Listofpoints[edge[1]][x]),int(Listofpoints[edge[1]][y]))
       
            DrawLine(pd1,pd2,[0,0,0],Listofpoints[edge[0]][z],Listofpoints[edge[1]][z])
        except:
            print(edge)
    if facesactive:
        for face in mesh.Faces:
            
            tri = [newverts[face[0]],newverts[face[1]],newverts[face[2]]]
            DrawTriangle(tri,color)
    
def RotateMesh(mesh,angle,axis):  
    
    x = 0
    y = 1
    z = 2
    sinus = math.sin(math.radians(angle/2))

    q = Quaternion(math.cos(math.radians(angle/2)),sinus * axis.i,sinus * axis.j,sinus * axis.k)
     

    mesh.Rotation.MultiplyBy(q)
        

meshtest = mesh([[25,15,15] , [25,15,-15] , [25,-15,15] , [25,-15,-15] ,
                     [-25,15,15] , [-25,15,-15], [-25,-15,15] , [-25,-15,-15]],
                    [[0,1] , [0,4] , [0,2] , [1,3] , [1,5] , [2,3] ,
                    [2,6] , [3,7] , [4,6] , [4,5] , [5,7] , [6,7]],[[0,5,1],[0,4,5],[0,2,1],[2,1,3],[4,5,6],[2,3,6],[3,5,7],[5,6,7],[3,6,7],[4,6,0],[5,1,3],[6,0,2]],Quaternion(1,0,0,0))

mesh2 = mesh([],[],[],Quaternion(1,0,0,0))

activemesh = meshtest



res = 10

for y in range(res):
    for x in range(res):
        mesh2.Vertices.append([-(res / 2 - 1) * 7.5 + x * 15,0,-(res / 2 - 1) * 7.5 + y * 15])

for i in range(res * res):
    if ((i + 1) % res != 0):
        mesh2.Edges.append([i,i + 1])
    if (i < res * res - res):
        mesh2.Edges.append([i,i + res])
    if (i < res * res - res and (i + 1) % res != 0):
        mesh2.Faces.append([i,i + res,i + res + 1])
        mesh2.Faces.append([i,i + 1,i + res + 1])
    
time = 0

TestAxis = Quaternion(0,0.6666,-0.6666,-0.3333) 
XAxis = Quaternion(0,1,0,0)
YAxis = Quaternion(0,0,1,0)
LightSourcePos = [170,170,-170]
mode = 'mode ' + str(MapWidth) + ',' + str(MapHeight)    
cmd = mode
os.system(cmd)
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
        RotateMesh(activemesh,1,TestAxis)
    if i[K_f]:
        ViewOffsetZ = ViewOffsetZ - 1
    if i[K_c]:
        activemesh = meshtest
    if i[K_p]:
        activemesh = mesh2
    if i[K_1]:
        zbufferactive = True
    if i[K_2]:
        zbufferactive = False
    if i[K_3]:
        facesactive = True
    if i[K_4]:
        facesactive = False
    if i[K_b]:
        ViewOffsetZ = ViewOffsetZ + 1
    if i[K_UP]:
        RotateMesh(activemesh,-1,XAxis)
    if i[K_DOWN]:
        RotateMesh(activemesh,1,XAxis)
    if i[K_LEFT]:
        RotateMesh(activemesh,1,YAxis)
    if i[K_RIGHT]:
        RotateMesh(activemesh,-1,YAxis)
    screen.fill((255, 255, 255))

    
    Update()
    time = time + 0.04
    pygame.display.flip()

pygame.quit()
