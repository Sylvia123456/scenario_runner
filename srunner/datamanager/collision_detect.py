from srunner.datamanager.icw_ver3.ICW import *
from srunner.utilities.calculation import *
import math

class CollisionICWDetector(object):

    def __init__(self):
        self._icw = ICW()

    def detect(self, ego, target, hud, map, showinfo):
        ego_velocity_vec = ego.get_velocity()
        ego_velocity = math.sqrt(ego_velocity_vec.x ** 2 + ego_velocity_vec.y ** 2)
        ego_acc_vec = ego.get_acceleration()
        ego_acc = math.sqrt(ego_acc_vec.x ** 2 + ego_acc_vec.y ** 2)
        ego_geo_vec = map.transform_to_geolocation(ego.get_location())
        ego_forward_vec = ego.get_transform().get_forward_vector()
        ego_angle = get_clock_angle(np.array([ego_forward_vec.x, ego_forward_vec.y])) + 90
        if ego_angle < 0:
            ego_angle = ego_angle + 360
        ego_point = Point(ego_geo_vec.longitude, ego_geo_vec.latitude, ego_angle, ego_velocity, ego_acc)

        target_velocity_vec = target.get_velocity()
        target_velocity = math.sqrt(target_velocity_vec.x ** 2 + target_velocity_vec.y ** 2)
        target_acc_vec = target.get_acceleration()
        target_acc = math.sqrt(target_acc_vec.x ** 2 + target_acc_vec.y ** 2)
        target_geo_vec = map.transform_to_geolocation(target.get_location())
        target_forward_vec = target.get_transform().get_forward_vector()
        target_angle = get_clock_angle(np.array([target_forward_vec.x, target_forward_vec.y])) + 90
        if target_angle < 0:
            target_angle = target_angle + 360
        target_point = Point(target_geo_vec.longitude, target_geo_vec.latitude, target_angle, target_velocity,
                             target_acc)

        result = self._icw.run(ego_point, target_point)

        if showinfo:
            info_text = [
                '[ego]',
                'longitude = % .5f' % ego_geo_vec.longitude,
                'latitude = %.5f' % ego_geo_vec.latitude,
                'forward_vec = {}, {}'.format(ego_forward_vec.x, ego_forward_vec.y),
                'angle = %.2f' % ego_angle,
                'velocity = %.2f ' % ego_velocity,
                'acc = %.2f' % ego_acc,
                '[target]',
                'longitude = %.5f' % target_geo_vec.longitude,
                'latitude = %.5f' % target_geo_vec.latitude,
                'angle = %.2f' % target_angle,
                'velocity = %.2f ' % target_velocity,
                'acc = %.2f' % target_acc,
                '[result]',
                result
            ]
            hud.set_info_text(info_text)
            logger.info("ego car : [longitude] = %f, [latitude] = %f, [angle] = %f" % (
                ego_geo_vec.longitude, ego_geo_vec.latitude, ego_angle))
            logger.info("target car : [longitude] = %f, [latitude] = %f, [angle] = %f" % (
                target_geo_vec.longitude, target_geo_vec.latitude, target_angle))

        return result
