from utils.rtt import RRT
import numpy as np
import cv2

# Map
x = np.array([3, 3, 3, 4, 4, 4, 5, 5, 5, 3, 3, 3, 4, 4, 4, 5, 5, 5, 3, 3, 3, 4, 4, 4, 5, 5, 5, 7, 8, 9, 10, 11, 12, 12, 13, 13])
y = np.array([4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 6, 6, 6, 7, 7, 7])

point_cloud2d = np.c_[x.reshape(-1), y.reshape(-1)]*50

#print (point_cloud2d.shape)

start= (np.random.randint(310),np.random.randint(310))

pic = np.zeros((480,640,3))
clrrt = RRT(GOAL_STOP_RADIUS=40,LINE_SIZE=[6,6],COLL_DIST=40, MAP=point_cloud2d)

while True:

    stop = (np.random.randint(610),np.random.randint(310))

    pic = np.zeros((480,640,3))

    clrrt.rtt(start,stop,2000)
    
    points = clrrt.get_points()
    path = clrrt.get_path()

    # draw obstacles
    for i in point_cloud2d:
        cv2.circle(pic,(i[0],i[1]),15,[0,0,255],-1) 

    # draw lines between points generated
    for p in points:
        p1 = p['prev']
        p2 = p['actual']

        if p1 is None:
            continue
        
        cv2.line(pic,(int(p1[0]),int(p1[1])),(int(p2[0]),int(p2[1])),(0,255,0),1)        

    # start point / goal
    cv2.circle(pic,start,7,[0,255,255],-1)
    cv2.circle(pic,stop,7,[0,255,255],-1)

    cnt = 0
    while len(path) > cnt:
        p = path[cnt].ravel()

        x = int(p[0])
        y = int(p[1])
        cv2.circle(pic,(int(p[0]),int(p[1])),5,[255,0,100],-1)    
        cnt +=1

        cv2.imshow("rrt",pic)

    if cv2.waitKey(1000) == 27:
        break
