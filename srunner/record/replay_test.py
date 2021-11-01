import carla
import argparse
import time
from carla import ColorConverter as cc
import numpy as np
import pygame
import weakref


def _clear_all_vehicles(world):
    all_actors = world.get_actors()
    for item in all_actors:
        item.destroy()

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, world):
        self.camera = None
        self.surface = None
        self.world = world

    def prepare_camera(self):
        ego_player = None
        while ego_player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "hero":
                    print("Ego vehicle found")
                    ego_player = vehicle
                    break

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(1280))
        camera_bp.set_attribute('image_size_y', str(720))
        if camera_bp.has_attribute('gamma'):
            camera_bp.set_attribute('gamma', str(2.2))

        self.camera = self.world.spawn_actor(camera_bp,
                                             carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
                                             attach_to=ego_player,
                                             attachment_type=carla.AttachmentType.SpringArm)

        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: CameraManager._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))


def game_loop(args):
    world = None
    pygame.init()
    pygame.font.init()

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        if world.get_map().name != "Town04":
            client.load_world("Town04")
        else:
            client.reload_world()

        # world = client.get_world()
        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 1 / 20
        # world.apply_settings(settings)

        if args.replayfile is None or args.replayfile == '':
            print("please set replay file path using --replayfile args")
        else:
            client.replay_file(args.replayfile, 10, 0, 0)

        display = pygame.display.set_mode(
            (1280, 720),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0, 0, 0))
        pygame.display.flip()

        world = client.get_world()
        camera_manager = CameraManager(world)
        camera_manager.prepare_camera()

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            world.tick(20)
            camera_manager.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            _clear_all_vehicles(world)
        pygame.quit()


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        "--replayfile",
        type=str,
        default='',
        help=' replay file path'
    )
    args = argparser.parse_args()

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        print(error)


if __name__ == '__main__':
    main()
