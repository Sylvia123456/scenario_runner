#!C:\Program Files\pythonxy\python\python.exe
# -*- coding:utf-8 -*-

from __future__ import division
from srunner.datamanager.ICW_VER1.log_format import *


class Point(object):
    def __init__(self, longitude, latitude, heading, speed, acceleration):
        self.longitude = longitude
        self.latitude = latitude
        self.heading = heading
        self.speed = speed  # meter/s
        self.acceleration = acceleration


class ICW():
    def __init__(self):
        #
        pass

    def __del__(self):
        #
        pass

    def run(self, hv, rv):
        collision_result = "False."
        collision_reason = "There is no collision point between hv and rv."
        orientation = "Left Front."

        logger.info("collision reason: %s" % (collision_reason))
        logger.info("collision result: %s" % (collision_result))
        logger.info("orientation: %s" % (orientation))

        return collision_result


# inital two points located in intersection
# para——lat,lon,heading,speed,acceleration
if __name__ == '__main__':
    hv = Point(116.4027068, 39.9312193, 0, 8, 0.5)
    rv = Point(116.4017601, 39.9319245, 90, 20, 2)

    icw = ICW()
    res = icw.run(hv, rv)
    print(res)


