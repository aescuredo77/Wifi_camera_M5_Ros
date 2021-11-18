import cv2
import math



class aruco_detect:
    """
        fiducial for tracking are 72 mm, 20 mm for the arm
    """
    def __init__(self, dictio=cv2.aruco.DICT_4X4_100, weight=72, distance = 297):
        self.weight = weight # weight of the fiducial
        self.distance = distance
        self.aruco_dict= cv2.aruco.Dictionary_get(dictio)
        self.parameters =  cv2.aruco.DetectorParameters_create()
        self.corners = []
        self.ids = []
            
    def start(self, frame = None):
        self.frame = frame
        if self.frame is not None:
            self.corners, self.ids, self.rejectedImgPoints = cv2.aruco.detectMarkers(self.frame, self.aruco_dict, parameters=self.parameters)

    def detect_orientation(self, frame=None):
        self.start(frame)
        angle = 0
        if len(self.corners) >0:
            ids = self.ids.flatten()   # local variable
            for (markerCorner, markerId) in zip(self.corners,ids):
                corners = markerCorner.reshape((4,2))  # local variable
                topLeft,topRight,bottonRight,bottonLeft = self.convert(corners)
                cX = int((topLeft[0] + bottonRight[0])/2)
                cY = int((topLeft[1] + bottonRight[1])/2)
                tx = int((topLeft[0] + topRight[0])/2)
                ty = int((topLeft[1] + topRight[1])/2)
                angle = math.degrees(math.atan2(cY-ty,cX-tx))
                if angle < 0:
                    angle = 360 + angle
                cv2.putText(frame, str(markerId)+':'+str(angle), (topLeft[0]-60,topLeft[1]-16),cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0,255,0),2)
                cv2.line(self.frame, (cX,cY),(tx,ty), (255,255,255),2)
                cv2.circle(self.frame, (cX,cY), 4, (0,0,255),-1)
        return self.frame
    
    def detect_corde(self, frame=None):
        self.start(frame)
        cordenadas = (0.0,0.0,0.0)
        if len(self.corners) >0:
            ids = self.ids.flatten()
            for (markerCorner, markerId) in zip(self.corners,ids):
                corners =markerCorner.reshape((4,2))
                topLeft,topRight,bottonRight,bottonLeft = self.convert(corners)
                cX = int((topLeft[0] + bottonRight[0])/2)
                cY = int((topLeft[1] + bottonRight[1])/2)
                cv2.circle(self.frame, (cX,cY), 4, (0,0,255),-1)
                cv2.line(self.frame, topLeft, topRight, (0,255,0),2) 
                cv2.line(self.frame, topRight, bottonRight, (0,255,0),2)
                cv2.line(self.frame, bottonRight, bottonLeft, (0,255,0),2)
                cv2.line(self.frame, bottonLeft, topLeft, (0,255,0),2)
                pixel_wtop = self.npixel(topLeft, topRight)
                pixel_wbotton = self.npixel(bottonLeft, bottonRight)
                pixel_hleft = self.npixel(topLeft, bottonLeft)
                pixel_hright = self.npixel(topRight, bottonRight)
                # distance = 297 # mm
                #rectangle = [pixel_wtop, pixel_wbotton, pixel_hleft, pixel_hright]
                #focal_lenght = (max(rectangle) * distance)/self.weight
                focal_lenght = 237.6 #243.4520575410848
                dis_cam_wtop = (focal_lenght * self.weight)/ pixel_wtop
                dis_cam_wbot = (focal_lenght * self.weight)/ pixel_wbotton
                dis_cam_hleft = (focal_lenght * self.weight)/ pixel_hleft
                dis_cam_hright = (focal_lenght * self.weight)/ pixel_hright
                distance_camera = (dis_cam_wtop + dis_cam_wbot + dis_cam_hright + dis_cam_hleft)/4
                z_floor_pixel = 120 - cY
                z_floor = (z_floor_pixel * distance_camera)/ focal_lenght
                x_center_pixel = 160 - cX
                x_center = (x_center_pixel * distance_camera)/focal_lenght
                cordenadas=(round(x_center,2), round(distance_camera, 2), round(z_floor,2))  
                cv2.putText(frame, str(markerId)+':'+str(cordenadas), (topLeft[0]-60,topLeft[1]-16),cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0,255,0),2)
        return self.frame, cordenadas
    
    def cal_focal_lenght(self, pixel):
        focal_lenght = (pixel * self.distance)/self.weight
        return focal_lenght
            
    def convert(self, corners):
        topLeft,topRight,bottonLeft,bottonRight = corners
        topRight = (int(topRight[0]),int(topRight[1]))
        topLeft = (int(topLeft[0]),int(topLeft[1]))
        bottonRight = (int(bottonRight[0]),int(bottonRight[1]))
        bottonLeft = (int(bottonLeft[0]),int(bottonLeft[1]))
        return topRight,topLeft,bottonRight,bottonLeft
    
    def npixel(self, startpoint, endpoint):
        pixelWeight = math.sqrt((startpoint[0] - endpoint[0])**2 + (startpoint[1] -endpoint[1])**2)
        return pixelWeight



    def calculoDistance(topleft,topright,bottonleft,bottonright):
        cX = int((topleft[0] + bottonright[0])/2)
        cY = int((topleft[1] + bottonright[1])/2)
        pixel_wtop = npixel(topleft, topright)
        pixel_wbotton = npixel(bottonleft, bottonright)
        pixel_hleft = npixel(topleft, bottonleft)
        pixel_hright = npixel(topright, bottonright)
        rectangle = [pixel_wtop, pixel_wbotton, pixel_hleft, pixel_hright]
        #focal_lenght = (max(rectangle) * 297)/20
        #print(focal_lenght)
        focal_lenght = 237.6 #243.4520575410848
        #print(bottonleft)
        weight = 20 #72
        dis_cam_wtop = (focal_lenght * weight)/ pixel_wtop
        dis_cam_wbot = (focal_lenght * weight)/ pixel_wbotton
        dis_cam_hleft = (focal_lenght * weight)/ pixel_hleft
        dis_cam_hright = (focal_lenght * weight)/ pixel_hright
        #distance_camera = (focal_lenght * weight) / max(rectangle)
        distance_camera = (dis_cam_wtop + dis_cam_wbot + dis_cam_hright + dis_cam_hleft)/4
        z_floor_pixel = 120 - cY
        z_floor = (z_floor_pixel * distance_camera)/ focal_lenght
        x_center_pixel = 160 - cX
        x_center = (x_center_pixel * distance_camera)/focal_lenght # 
        return (round(x_center,2), round(distance_camera, 2), round(z_floor,2))
    
    
