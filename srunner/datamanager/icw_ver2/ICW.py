#!C:\Program Files\pythonxy\python\python.exe
# -*- coding:utf-8 -*-

from __future__ import division
import math
from scipy.optimize import fsolve
from srunner.datamanager.icw_ver2.log_format import *


class Point(object):
    def __init__(self, longitude, latitude, heading, speed, acceleration):
        self.longitude = longitude
        self.latitude = latitude
        self.heading = heading
        self.speed = speed  
        self.acceleration = acceleration

class ICW():
    def __init__(self):
        #
        pass

    def __del__(self):
        # 
        pass

    def affineTransformation(self,hv,rv):
        # angle = (rv.heading - hv.heading + 360)/180*math.pi
        # if angle > 2*math.pi:
        #     angle = angle - 2*math.pi

        angle = hv.heading/180*math.pi
        delta_x = rv.longitude - hv.longitude
        delta_y = rv.latitude - hv.latitude
        # print(delta_x,delta_y)
        x_ = delta_x * math.cos(angle) + delta_y * math.sin(angle)
        y_ = -delta_x * math.sin(angle) + delta_y * math.cos(angle)
        # print(x_,y_,angle/math.pi*180)
        return (x_,y_,angle)

    def headingFormat(self,rv_heading,hv_heading):
        angle = rv_heading - hv_heading + 360
        if angle >= 360:
            angle = angle - 360

        # angle = rv_heading

        vectorStandard = {0:['y',True], 90:['x',True], 180:['y',False], 270:['x',False], 360:['y',True]}
        headingStandard,headingInfo = min(vectorStandard.items(), key = lambda x: abs(angle - x[0]))
        print ("headingInfo:",headingStandard,headingInfo)
        return headingInfo

    def locationFormat(self,x_,y_):
        if x_ >=0 :
            x_vector = True
        else:
            x_vector = False

        if y_ >= 0:
            y_vector = True
        else:
            y_vector = False
        return x_vector, y_vector

    #revised20210713
    def boundaryDetermine(self,pnt):
        x = pnt.longitude
        y = pnt.latitude
        heading = pnt.heading

        if (heading >= 0 and heading < 90):
            boundaryX = [x, float('inf')]
            boundaryY = [y, float('inf')]
        elif (heading >= 90 and heading < 180):
            boundaryX = [x, float('inf')]
            boundaryY = [float('-inf'), y]
        elif (heading >= 180 and heading < 270):
            boundaryX = [float('-inf'), x]
            boundaryY = [float('-inf'), y]
        elif (heading >= 270 and heading <= 360):
            boundaryX = [float('-inf'), x]
            boundaryY = [y, float('inf')]

        return boundaryX, boundaryY

    def collisionPointCal(self,hv,rv):
        x1,y1,angleH1 = hv.longitude,hv.latitude,hv.heading
        x2,y2,angleH2 = rv.longitude,rv.latitude,rv.heading

        h1_axisX = 360 - angleH1 + 90
        h2_axisX = 360 - angleH2 + 90
        if h1_axisX >= 360:
            h1_axisX = h1_axisX - 360
        if h2_axisX >= 360:
            h2_axisX = h2_axisX - 360

        # print(h1_axisX,h2_axisX)
        arcH1 = h1_axisX/180*math.pi
        arcH2 = h2_axisX/180*math.pi

        if h1_axisX == 90 or h1_axisX == 270:
            xc = x1
        elif h2_axisX == 90 or h2_axisX == 270:
            xc = x2
        else:
            xc = ((y2-y1) - (x2*math.tan(arcH2) - x1*math.tan(arcH1)))/(math.tan(arcH1) - math.tan(arcH2))

        if h1_axisX == 0 or h1_axisX == 180:
            yc = y1
        elif h2_axisX == 0 or h2_axisX == 180:
            yc = y2
        else: 
            yc = ((x2-x1) - (y2/math.tan(arcH2) - y1/math.tan(arcH1)))/(1/math.tan(arcH1) - 1/math.tan(arcH2))

        # add 20210623
        boundaryX_hv, boundaryY_hv = self.boundaryDetermine(hv)
        boundaryX_rv, boundaryY_rv = self.boundaryDetermine(rv)
        # print(boundaryX_hv, boundaryY_hv)
        # print(boundaryX_rv, boundaryY_rv)
        #
        # print("pre",xc, yc)
        # print(boundaryX_hv, boundaryY_hv)
        # print(boundaryX_rv, boundaryY_rv)

        if not ((xc >= boundaryX_hv[0] and xc <= boundaryX_hv[1]) and (xc >= boundaryX_rv[0] and xc <= boundaryX_rv[1]) and \
                (yc >= boundaryY_hv[0] and yc <= boundaryY_hv[1]) and (yc >= boundaryY_rv[0] and yc <= boundaryY_rv[1])):
            xc, yc = None, None
        # print("hou", xc, yc)

        return (xc,yc)


    def laneDecision(self,x_vector,y_vector,headingInfo):
        axis_vector, heading_vector = headingInfo
        if not y_vector:
            lane_id = 1
        else:
            if axis_vector == 'x':
                if x_vector == heading_vector:
                    lane_id = 48
                else:
                    lane_id = 37
            if axis_vector == 'y':
                if y_vector == heading_vector:
                    lane_id = 26
                else:
                    lane_id = 5
        return lane_id

    def deg2Rad(self,deg):
       return deg * math.pi / 180.0

    def distCal(self,p1,p2):
        earth_radius = 6378.137
        lon1,lat1 = p1
        lon2,lat2 = p2

        radLat1 = self.deg2Rad(lat1)
        radLat2 = self.deg2Rad(lat2)
        a = radLat1 - radLat2
        b = self.deg2Rad(lon1) - self.deg2Rad(lon2)
 
        s = 2 * math.asin(math.sqrt(math.pow(math.sin(a/2),2) + math.cos(radLat1)*math.cos(radLat2)*math.pow(math.sin(b/2),2)))
 
        s = s * earth_radius * 1000
        # s = round(s * 10000) / 10000
        return s


    def vehicle2collisionPnt(self,hv,rv,collisionPnt):
        xc,yc = collisionPnt
        dist_hv = self.distCal((hv.longitude,hv.latitude),(xc,yc))
        dist_rv = self.distCal((rv.longitude,rv.latitude),(xc,yc))
        # print("dist:",dist_hv,dist_rv)
        return dist_hv,dist_rv


    # add 20210608
    def quadraticSolve(self,a,b,c):
        # double x1,x2; //两个
        # int type     //0表示一个根，1表示两个实根，2表示两个复数的根
        if a < 1E-5:
            x = -c/b
        else:
            d = b*b-4*a*c
            if(abs(d) < 1E-5):
                type = 0
                x1 = -b/(2*a)
                x2 = x1
            elif(d > 0):
                type = 1
                x1 = (-b+math.sqrt(d))/(2*a)
                x2 = (-b-math.sqrt(d))/(2*a)
            else:
                type = 2
                x1 =  -b/(2*a)
                x2 = math.sqrt(-d)/(2*a)

            if (type == 0 or type == 1):
                if (x1 >0 and x2>0):
                    x = min(x1, x2)
                elif(x1 <0 and x2<0):
                    x = None
                else:
                    x = max(x1,x2)
            else:
                x = None
        return x
        
    def TTC(self,hv,rv,dist_hv,dist_rv):
        # hv,rv,x_,y_
        # delta_angle = rv.heading - hv.heading
        # speed_x = rv.speed*abs(math.cos(delta_angle))
        # speed_y = rv.speed*abs(math.sin(delta_angle))
        # acceleration_x = rv.acceleration*abs(math.cos(delta_angle))
        # acceleration_y = rv.acceleration*abs(math.sin(delta_angle))

        # t2 = scipy.optimize.fsolve(lambda x: rv.speed_x*t2 + 0.5*rv.acceleration_x*t2*t2 - x_, 0)
        # t1 = scipy.optimize.fsolve(lambda x: hv.speed*t1 + 0.5*hv.acceleration*t1*t1 - y_, 0)

        # revised 20210608
        # t1 = fsolve(lambda t1: hv.speed*t1 + 0.5*hv.acceleration*t1*t1 - dist_hv, 0)
        # t2 = fsolve(lambda t2: rv.speed*t2 + 0.5*rv.acceleration*t2*t2 - dist_rv, 0)
        ttr_hv = self.quadraticSolve(0.5*hv.acceleration,hv.speed,-dist_hv)
        ttr_rv = self.quadraticSolve(0.5*rv.acceleration,rv.speed,-dist_rv)
        # print(t1,t1_)
        # print(t2,t2_)
        ttc = None
        if ttr_hv is None:
            logger.info("The speed of hv is zero on the way to reach the collision point.")
        elif ttr_rv is None:
            logger.info("The speed of rv is zero on the way to reach the collision point.")
        else:
        # print("time:",t1,t2)
            logger.info("The time to reach collision point is %f and %f."%(ttr_hv,ttr_rv))
            ttc = abs(ttr_hv - ttr_rv)
        # revised 20210707
        return ttr_hv,ttr_rv,ttc

    def filterByLocation(self,hv,rv):
        x_,y_,angle = self.affineTransformation(hv,rv)
        x_vector,y_vector = self.locationFormat(x_,y_)
        headingInfo = self.headingFormat(rv.heading,hv.heading)
        lane_id = self.laneDecision(x_vector,y_vector,headingInfo)
        # print(x_vector,y_vector,headingInfo)
        logger.info("The vector of rv relative hv is %s,%s,%s,%s"%(x_vector,y_vector,headingInfo[0],headingInfo[1]))
        # print("lane_id",lane_id)
        if lane_id in [37,5]:
            return True
        else:
            return False


    def filterByAngle(self,hv,rv):
        # 相对航向角 [0,2*pi]
        delt_heading = (rv.heading - hv.heading + 360)/180*math.pi
        if delt_heading >= 2*math.pi:
            delt_heading = delt_heading - 2*math.pi
        
        # 相对方位角 [0,2*pi]
        global delt_orientation
        x_ = rv.longitude - hv.longitude
        y_ = rv.latitude - hv.latitude
        # print(x_,y_)
        if (x_ < 0 and y_ < 0) or (x_ < 0 and  y_ >0):
            delt_orientation = 270/180*math.pi - math.atan(y_/x_)
        elif (x_ > 0 and  y_ <0) or (x_ >0 and y_ >0):
            delt_orientation = -math.atan(y_/x_) + 0.5*math.pi
        else:
            delt_orientation = 0

        delt_orientation = delt_orientation/math.pi*180
        delt_heading = delt_heading/math.pi*180

        logger.info("The delta orientation between hv and rv is %f"%(delt_orientation))        
        logger.info("The delta heading between hv and rv is %f"%(delt_heading))        

        if abs(hv.heading - rv.heading) <= 5:
            flag = False
        else:
            if (delt_orientation >= 0 and delt_orientation <= 90) and (delt_heading > 180 and delt_heading <= 360):
                flag = True
            elif (delt_orientation >= 90 and delt_orientation <= 180) and (delt_heading >= 270 and delt_heading < 360):
                flag = True
            elif (delt_orientation >= 180 and delt_orientation <= 270) and (delt_heading > 0 and delt_heading <= 90):
                flag = True
            elif (delt_orientation >= 270 and delt_orientation <= 360) and (delt_heading >= 0 and delt_heading < 180):
                flag = True
            else:
                flag = False

        print("aa:", delt_orientation, delt_heading, flag)
        return flag 

    # add 20210707
    def t4Cal(self,hv,am,t1,t2,t3,dist_hv):
        s1 = hv.speed * (t1+t2)
        s2 = hv.speed * t3 + 1/6.0 * am * t3 * t3
        v4 = hv.speed + 0.5 * am * t3
        # dist_hv = v4*t4 + 0.5*am*t4*t4
        # t4 = fsolve(lambda t4: v4*t4 + 0.5*am*t4*t4 - (dist_hv - s1 - s2), 0)
        t4 = self.quadraticSolve(0.5*am,v4,- (dist_hv - s1 - s2))
        if t4 is None:
            t4 = hv.speed/am
        return t4 + t3 + t2 + t1

    # add 20210707
    def TTA(self,hv,dist_hv):
        t1 = 1.2
        t2 = 0.1
        t3 = 0.4
        aCom = 3.0
        aReact = 6.0
        tCom = self.t4Cal(hv,aCom,t1,t2,t3,dist_hv)
        tReact = self.t4Cal(hv,aReact,t1,t2,t3,dist_hv)
        logger.info("The Tcom and Treact is %f and %f."%(tCom,tReact))
        return tCom,tReact


    def simplyrun(self,hv,rv):
        filterFlagByLocation = self.filterByLocation(hv,rv)
        filterFlagByAngle = self.filterByAngle(hv,rv)
        
        # print("filter:",filterFlagByLocation,filterFlagByAngle)
        logger.info("The filter result is %s and %s by location and angle."%(filterFlagByLocation,filterFlagByAngle))
        res = "False"
        if filterFlagByLocation:
            collisionPnt = self.collisionPointCal(hv,rv)
            # print("collisionPnt:",collisionPnt)
            # add 20210608(if decision)
            if (collisionPnt[0] is not None) and (collisionPnt[1] is not None):
                logger.info("The collision point is %f,%f."%(collisionPnt[0],collisionPnt[1]))
                dist_hv,dist_rv = self.vehicle2collisionPnt(hv,rv,collisionPnt)
                delt_tm = self.TTC(hv,rv,dist_hv,dist_rv)
                # print("delt_tm",delt_tm)
                logger.info("The delta time is %f."%(delt_tm))

                # revised 20210608
                if delt_tm is not None and delt_tm < 5:
                    tCom,tReact = self.TTA(hv,dist_hv)
                    res = "True"
        else:
            logger.info("There is no possible between HV and RV.")

        logger.info("The finally result is %s."%(res))
        return res


    def run(self,hv,rv):
        filterFlagByLocation = self.filterByLocation(hv,rv)
        filterFlagByAngle = self.filterByAngle(hv,rv)
        
        print("filter:",filterFlagByLocation,filterFlagByAngle)
        logger.info("The filter result is %s and %s by location and angle."%(filterFlagByLocation,filterFlagByAngle))
        if filterFlagByAngle:
            collisionPnt = self.collisionPointCal(hv,rv)
            # print("collisionPnt:",collisionPnt)
            # add 20210608(if decision)
            if (collisionPnt[0] is not None) and (collisionPnt[1] is not None):
                logger.info("The collision point is %f,%f."%(collisionPnt[0],collisionPnt[1]))
                dist_hv,dist_rv = self.vehicle2collisionPnt(hv,rv,collisionPnt)
                ttr_hv,ttr_rv,delt_tm = self.TTC(hv,rv,dist_hv,dist_rv)
                print("dist:",dist_hv,dist_rv)
                # print("delt_tm",delt_tm)
                logger.info("The delta time is %f."%(delt_tm))

                # revised 20210608
                if delt_tm is not None:
                    if delt_tm < 5:
                        # add 20210530
                        res = "True"
                        tCom,tReact = self.TTA(hv, dist_hv)
                        if (ttr_hv > tCom):
                            reason = "The ttr of hv is more than tCom."
                        elif (ttr_hv <= tCom) and (ttr_hv > tReact):
                            reason = "The ttr of hv is less than tCom and more than tReact."
                        elif (ttr_hv <= tReact):
                            reason = "The ttr of hv is less than tReact."
                    else:
                        reason = "The TTC is less than five seconds."
                        res = "False"
                else:
                    reason = "The speed of rv or hv is zero on the way to reach the collision point."
                    res = "False"
            else:
                reason = "There is no collision point between hv and rv."
                res = "False"
        else:
            res = "False"
            reason = "There is no possible between HV and RV by filter."


        if (delt_orientation >= 0 and delt_orientation <= 90):
            orientation = "Right Front."
        elif (delt_orientation >= 90 and delt_orientation <= 180):
            orientation = "Right Back."
        elif (delt_orientation >= 180 and delt_orientation <= 270):
            orientation = "Left Back."
        elif (delt_orientation >= 270 and delt_orientation <= 360):
            orientation = "Left Front."

        # print(delt_orientation,orientation)
        logger.info("collision reason: %s"%(reason))
        logger.info("collision result: %s"%(res))
        logger.info("orientation: %s" % (orientation))
        print("res:",res)
        print("reason:",reason)

        return res



# {'ID':1}——"POINT(116.40269 39.93182)"
# {'ID':6}——"POINT(116.40269 39.93211)"
# {'ID':5}——"POINT(116.40258 39.93211)"
# {'ID':2}——"POINT(116.40259 39.93181)"
# {'ID':3}——"POINT(116.40252 39.931901)"
# {'ID':4}——"POINT(116.40252 39.932011)"
# {'ID':7}——"POINT(116.40278 39.932011)"
# {'ID':8}——"POINT(116.40278 39.931901)"


#inital two points located in intersection
#para——lat,lon,heading,speed,acceleration
if __name__ == '__main__':
    # for i in range(1, len(sys.argv)):  
    #     strs = sys.argv[i]
    #     print(i,strs)

    # hvPoint = sys.argv[1]
    # rvPoint = sys.argv[2]
    # hvList = hvPoint.split(",")
    # rvList = rvPoint.split(",")

    # hv = Point(float(hvList[0]),float(hvList[1]),float(hvList[2]),float(hvList[3]),float(hvList[4]))
    # rv = Point(float(rvList[0]),float(rvList[1]),float(rvList[2]),float(rvList[3]),float(rvList[4]))

    hv = Point(116.4027068, 39.9312193, 0, 8, 0.5)
    rv = Point(116.4017601, 39.9319245, 90, 8, 0.5)
    
    icw = ICW()
    res = icw.run(hv, rv)
    print(res)


