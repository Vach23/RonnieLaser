import cv2
print(cv2.__version__)
import numpy as np

import sys

def rotz(a):
    return np.matrix([[np.cos(a), -np.sin(a), 0],
                        [np.sin(a), np.cos(a), 0],
                        [0,0,1]])


def points_in_rect(points, rect, center, angle):
    n = len(points)
    M = rotz(np.deg2rad(-angle))

    rect = np.vstack([np.matrix(rect).transpose(),np.zeros([1,4])])
    points = np.vstack([np.matrix(points).transpose(),np.zeros([1,n])])
    # print(points[0:2,:])
    # print(rect[0:2,:])
    # print(angle)

    rect_r = M.dot(np.matrix(rect))[0:2,:]
    points_r = M.dot(np.matrix(points))[0:2,:]

    # print(rect_r)
    # print(points_r)

    x_min, x_max = np.min(rect_r[0,:]), np.max(rect_r[0,:])  
    y_min, y_max = np.min(rect_r[1,:]), np.max(rect_r[1,:])

    new_points = np.zeros([2, n])

    counter = 0

    for i in range(n):
        xx = points_r[0,i]
        yy = points_r[1,i]
        #print(xx,yy)
        if (xx > x_min) and (xx < x_max):
            if (yy > y_min) and (yy < y_max):

                new_points[:,counter] = np.array([points[0,i],points[1,i]])
                #print(new_points[counter,:])
                counter+=1
    if counter != 0:
        coords = [np.mean([np.max(new_points[0,0:counter]),np.min(new_points[0,0:counter])]), np.mean([np.max(new_points[1,0:counter]),np.min(new_points[1,0:counter])])]
    else:
        coords = None

    return new_points[:,0:counter].transpose(), counter, coords




    

class SKP:
    def __init__(self, mea_e, est_e, q):
        self.mea_e = mea_e
        self.est_e = est_e
        self.q = q

        self.kalman_gain = 0
        self.last_estimate = 0
        self.curr_estimate = 0
    
    def updateEstimate(self, mea):
        self.kalman_gain = self.est_e/(self.est_e + self.mea_e)
        self.curr_estimate = self.last_estimate + self.kalman_gain*(mea-self.last_estimate)
        self.est_e = (1-self.kalman_gain)*self.est_e+abs(self.last_estimate-self.curr_estimate)*self.q
        self.last_estimate = self.curr_estimate

        return self.curr_estimate

class SKP_2D:
    def __init__(self, mea_e, est_e, q):
        self.x = SKP(mea_e=mea_e,est_e=est_e,q=q)
        self.y = SKP(mea_e=mea_e,est_e=est_e,q=q)
    def updateEstimate(self, mea):
        mea[0] = self.x.updateEstimate(mea[0])
        mea[1] = self.y.updateEstimate(mea[1])
        return mea



skp_left = SKP(1, 1, 0.05)
skp_right = SKP(1, 1, 0.05)

skp_2d_p1 = SKP_2D(1, 1, 0.15)
skp_2d_p2 = SKP_2D(1, 1, 0.15)

prev_p1 = [0,0]
prev_p2 = [0,0]



def nothing(x):
    pass
 
cv2.namedWindow('Trackbars')
cv2.moveWindow('Trackbars',1320,0)
 
cv2.createTrackbar('hueLower', 'Trackbars',0,179,nothing)
cv2.createTrackbar('hueUpper', 'Trackbars',50,179,nothing)
 
cv2.createTrackbar('hue2Lower', 'Trackbars',63,179,nothing)
cv2.createTrackbar('hue2Upper', 'Trackbars',179,179,nothing)
 
cv2.createTrackbar('satLow', 'Trackbars', 0, 255,nothing)
cv2.createTrackbar('satHigh', 'Trackbars', 121, 255,nothing)
cv2.createTrackbar('valLow','Trackbars', 165, 255,nothing)
cv2.createTrackbar('valHigh','Trackbars', 255,255,nothing)
cv2.createTrackbar('grey_low2','Trackbars', 179, 255,nothing)
cv2.createTrackbar("kernel_size", "Trackbars",18,30,nothing)


cv2.createTrackbar('grey_low','Trackbars', 47, 255,nothing)
cv2.createTrackbar('grey_High','Trackbars', 188, 255,nothing)
 
cv2.createTrackbar('to_zero','Trackbars', 30, 255,nothing)

dispW=640
dispH=480
flip=2
#Uncomment These next Two Line for Pi Camera
#camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam= cv2.VideoCapture("video_cap3.avi")

background_mask_greyscale = cv2.imread("background.jpg")
background_mask_greyscale = cv2.cvtColor(background_mask_greyscale, cv2.COLOR_BGR2GRAY)

background_mask_hsv = cv2.imread("test_background.png")
background_mask_hsv= cv2.cvtColor(background_mask_hsv, cv2.COLOR_BGR2GRAY)

"""
for i in range(56,61):
    temp = cv2.imread("background" + str(i)+ ".png")
    background_mask_hsv = cv2.add(background_mask_hsv, temp)

cv2.imwrite("test_background.png",background_mask_hsv)
"""
counter = 0
cv2.imshow('FGmaskComp', background_mask_hsv)

while True:
    counter+=1
    try:
        ret, frame = cam.read()

    #frame=cv2.imread('smarties.png')
        
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    except:
        cam= cv2.VideoCapture('video_cap3.avi')
        ret, frame = cam.read()
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        counter = 1

    hueLow=cv2.getTrackbarPos('hueLower', 'Trackbars')
    hueUp=cv2.getTrackbarPos('hueUpper', 'Trackbars')
 
    hue2Low=cv2.getTrackbarPos('hue2Lower', 'Trackbars')
    hue2Up=cv2.getTrackbarPos('hue2Upper', 'Trackbars')
 
    Ls=cv2.getTrackbarPos('satLow', 'Trackbars')
    Us=cv2.getTrackbarPos('satHigh', 'Trackbars')
 
    Lv=cv2.getTrackbarPos('valLow', 'Trackbars')
    Uv=cv2.getTrackbarPos('valHigh', 'Trackbars')

    g_low2 = cv2.getTrackbarPos('grey_low2', 'Trackbars')

    g_low = cv2.getTrackbarPos('grey_low', 'Trackbars')
    g_high = cv2.getTrackbarPos('grey_High', 'Trackbars')
    to_zero = cv2.getTrackbarPos('to_zero', 'Trackbars')

    kernel_size = cv2.getTrackbarPos("kernel_size", "Trackbars")
    if kernel_size <1:
        kernel_size = 1
 
    l_b=np.array([hueLow,Ls,Lv])
    u_b=np.array([hueUp,Us,Uv])
 
    l_b2=np.array([hue2Low,Ls,Lv])
    u_b2=np.array([hue2Up,Us,Uv])


 
    FGmask=cv2.inRange(hsv,l_b,u_b)
    FGmask2=cv2.inRange(hsv,l_b2,u_b2)
    FGmaskComp=cv2.add(FGmask,FGmask2)
    FGmaskComp = cv2.subtract(FGmaskComp, background_mask_hsv)
    FGmaskComp = cv2.blur(FGmaskComp, (kernel_size,kernel_size))
    cv2.imshow('FGmaskComp_BEFORE', FGmaskComp)
    cv2.moveWindow('FGmaskComp_BEFORE', 700, 0)
    ret, FGmaskComp = cv2.threshold(FGmaskComp, g_low2, 255, cv2.THRESH_BINARY)
    cv2.imshow('FGmaskComp', FGmaskComp)
    cv2.moveWindow('FGmaskComp', 0, 530)

    

    frame2 = (cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) - background_mask_greyscale)
    prev = frame2
    
    ret, frame2 = cv2.threshold(frame2, g_low, 255, cv2.THRESH_TOZERO)
    ret, frame2 = cv2.threshold(frame2, g_high, 255, cv2.THRESH_TOZERO_INV)
    ret, frame2 = cv2.threshold(frame2, to_zero, 255, cv2.THRESH_BINARY)
    #frame2 = prev - frame2
    
 
    contours, _ = cv2.findContours(FGmaskComp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours2, _ = cv2.findContours(frame2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print(len(contours))
    #print(len(contours2))

    
    
    #contours=sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)
    
    for cnt in contours:
        area=cv2.contourArea(cnt)
        (x,y,w,h)=cv2.boundingRect(cnt)
        if area>=500 and area <= 4000:
            #print("area", area)
            cv2.drawContours(frame,[cnt],0,(0,0,255),1)

            rows,cols = frame.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.05,0.05)
            lefty = ((-x*vy/vx) + y)
            righty = (((cols-x)*vy/vx)+y)

            # cv2.line(frame,(cols-50,int(righty)),(50,int(lefty)),(0,0,255),1)
            #lefty = int(skp_left.updateEstimate(lefty))
            #righty = int(skp_right.updateEstimate(righty))
            #print(vx,vy,x,y,lefty, righty)
            k = 100
            p1 = [int(-vx*k+x), int(-vy*k+y)]
            p2 = [int(vx*k+x), int(vy*k+y)]

            diff = np.sqrt(pow(p1[0]-prev_p1[0],2) + pow(p1[1]-prev_p1[1],2))
            #print(prev_p1, p1,diff)
            if 2*k-20 < diff:
                p1, p2 = p2,p1
            
            prev_p1 = p1
            prev_p2 = p2
            

            p2[0] = int(skp_left.updateEstimate(p2[0]))
            p2[1] = int(skp_right.updateEstimate(p2[1]))
            
            #cv2.line(frame,(cols-1,righty),(0,lefty),(0,0,255),2)
            cv2.circle(frame,tuple(p1),5,(0,0,255),2)
            cv2.circle(frame,tuple(p2),5,(0,255,0),2)
            # cv2.line(frame,p1,p2,(0,0,255),2)
            
            #cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            (center, shape, angle) = cv2.minAreaRect(cnt)
            if shape[0] > shape[1]:
                shape2 = (shape[1], shape[0])
                angle2 = angle-90
            else:
                angle2 = angle
                shape2 = shape
            s_x = np.cos(np.deg2rad(angle2+90)) * (shape2[1]/2*0.9)
            s_y = np.sin(np.deg2rad(angle2+90)) * (shape2[1]/2*0.9)


            c1 = [s_x+center[0], s_y + center[1]]
            c3 = [-s_x+center[0], -s_y + center[1]]
            shape3 = (shape2[0]*1.1, shape2[1]*0.3)
            shape2 = (shape2[0]*1.1, shape2[1]*0.15)

            box0 = cv2.boxPoints((center, shape, angle))
            box1 = cv2.boxPoints((c1, shape2, angle2))
            box2 = cv2.boxPoints((center, shape3, angle2))
            box3 = cv2.boxPoints((c3, shape2, angle2))


            points1, counter1, coord1 = points_in_rect(cnt, box1, c1, angle2)
            points2, counter2, coord2 = points_in_rect(cnt, box2, center, angle2)
            points3, counter3, coord3 = points_in_rect(cnt, box3, c3, angle2)



            box0 = np.int0(box0)
            box1 = np.int0(box1)
            box2 = np.int0(box2)
            box3 = np.int0(box3)

            cv2.drawContours(frame,[box0],0,(255,0,0),1)
            # cv2.drawContours(frame,[box1],0,(0,0,255),2)
            # cv2.drawContours(frame,[box2],0,(255,0,0),2)
            # cv2.drawContours(frame,[box3],0,(0,255,0),2)
            # print(box3)
            # print(type(box3))

            # print()
            # print(type(points))
            # sys.exit()
            # print()
            # print()
            # print()
            # print("NOASDKASJDLJAKDLWJSKLASJDKLAJSKLAJDAKLJDKLJKDWADSHJKAJSDFHKASDFKASDGFHASGDFGASDGFHJGASDJFGASJDGGAF")

            # if counter1 > 0:
            #     cv2.drawContours(frame,[np.int0(points1)],0,(0,0,255),2)
            #     # coord1 = np.mean(points1, axis=0)
            #     cv2.circle(frame,tuple(np.int0(coord1)),5,(0,0,255),2)
            
            # if counter2 > 0:
            #     cv2.drawContours(frame,[np.int0(points2)],0,(255,0,0),2)
            #     # coord2 = np.mean(points2, axis=0)
            #     cv2.circle(frame,tuple(np.int0(coord2)),5,(255,0,0),2)

            # if counter3 > 0:
            #     cv2.drawContours(frame,[np.int0(points3)],0,(0,255,0),2)
            #     # coord3 = np.mean(points3, axis=0)
            #     cv2.circle(frame,tuple(np.int0(coord3)),5,(0,255,0),2)

            if counter1 > 0 and counter2 > 0 and counter3 > 0:
                coord1[0] = coord1[0] + (-coord2[0] + coord1[0])*1.2
                coord1[1] = coord1[1] + (-coord2[1] + coord1[1])*1.2

                coord3[0] = coord3[0] + (-coord2[0] + coord3[0])*1.2
                coord3[1] = coord3[1] + (-coord2[1] + coord3[1])*1.2

                # if 2*k-20 < diff:
                #     coord3, coord1 = coord1, coord3 

                cv2.line(frame, np.int0(coord2), np.int0(coord1), (0,0,255), 2)
                cv2.line(frame, np.int0(coord2), np.int0(coord3), (0,255,0), 2)
                coord1 = skp_2d_p1.updateEstimate(coord1)
                coord3 = skp_2d_p2.updateEstimate(coord3)
                cv2.circle(frame, tuple(np.int0(coord1)), 5, (0,0,255), 2)
                cv2.circle(frame, tuple(np.int0(coord3)), 5, (0,255,0), 2)
            # if counter == 200:
            #     input()
            break
    
    """
    for cnt in contours2:
        area=cv2.contourArea(cnt)
        (x,y,w,h)=cv2.boundingRect(cnt)
        if area>=500 and area <= 4000:
            #input()
            #print("area", area)
            #print(cnt[:,0,1])
            rect = cv2.minAreaRect(cnt)
            #print(rect)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame,[box],0,(50,0,255),2)

            ellipse = cv2.fitEllipse(cnt)
            #cv2.ellipse(frame,ellipse,(0,255,0),2)

            rows,cols = frame.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)


            #cv2.line(frame,(cols-1,righty),(0,lefty),(255,0,0),2)
            cv2.drawContours(frame,[cnt],0,(255,0,0),2,cv2.LINE_AA)

            break
    """
    
    if counter in [55,56,57,58,59,60]:
        cv2.imwrite("background"+str(counter)+".png", FGmaskComp)
 
    cv2.imshow('nanoCam',frame)
    cv2.moveWindow('nanoCam',0,0)
    cv2.imshow('bitwise_not',frame2)
    cv2.moveWindow('bitwise_not',700,530)

 
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()