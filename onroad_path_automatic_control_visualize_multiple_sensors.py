#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that render multiple sensors in the same pygame window

By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configure for any different number of sensors. 
To do that, check lines 290-308.
"""

import glob
import os
import sys
from matplotlib import pyplot as plt

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np


try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# tweak to where you put carla agents folder for necessary modules
sys.path.append('C:/carla/PythonAPI/carla/')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption

# Set visualization lifetime of markers to inf
LIFE_TIME = 10000
desired_speed = 50 #km/h


def draw_point(world, wp, option):
    if option == RoadOption.LEFT:  # Yellow
        color = carla.Color(128, 128, 0)
    elif option == RoadOption.RIGHT:  # Cyan
        color = carla.Color(0, 128, 128)
    elif option == RoadOption.CHANGELANELEFT:  # Orange
        color = carla.Color(128, 32, 0)
    elif option == RoadOption.CHANGELANERIGHT:  # Dark Cyan
        color = carla.Color(0, 32, 128)
    elif option == RoadOption.STRAIGHT:  # Gray
        color = carla.Color(64, 64, 64)
    else:  # LANEFOLLOW
        color = carla.Color(0, 128, 0)  # Green

    world.debug.draw_point(wp.transform.location + carla.Location(z=0.2), size=0.05, color=color, life_time=LIFE_TIME)


def draw_keypoint(world, location):
    world.debug.draw_point(location + carla.Location(z=0.2), size=0.1, color=carla.Color(128, 0, 128),
                           life_time=LIFE_TIME)
    string = "(" + str(round(location.x, 1)) + ", " + str(round(location.y, 1)) + ", " + str(round(location.z, 1)) + ")"
    world.debug.draw_string(location + carla.Location(z=0.5), string, True, color=carla.Color(0, 0, 128),
                            life_time=LIFE_TIME)


def show_route(route, world, grp):
    distance = 0
    X_route = np.zeros(1)
    Y_route = np.zeros(1)
    if route:
        for i in range(len(route) - 1):
            waypoint = route[i]
            waypoint_next = route[i + 1]
            interpolated_trace = grp.trace_route(waypoint, waypoint_next)
            for j in range(len(interpolated_trace) - 1):
                wp, option = interpolated_trace[j]
                # Calculate the Euclidean distance between the two waypoints
                if (abs(wp.transform.location.x - waypoint_next.x) < 0.2) and (abs(wp.transform.location.y - waypoint_next.y) < 0.2):
                    break
                wp_next = interpolated_trace[j + 1][0]
                draw_point(world, wp, option)
                distance += wp.transform.location.distance(wp_next.transform.location)
                X_route = np.append(X_route, wp.transform.location.x)
                Y_route = np.append(Y_route, wp.transform.location.y)
            draw_keypoint(world, waypoint)
        draw_keypoint(world, route[-1])
    return X_route, Y_route


class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        sampling_resolution = 3
        sampling_resolution_optimal_line = 0.2
        grp = GlobalRoutePlanner(world.get_map(), sampling_resolution_optimal_line)
        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        # Define the route using indices from the spawn points specified for TOWN 7
        route_1_indices = [58, 108, 113, 101, 83, 40, 87]
        # Get locations from the spawn point indices
        route_1 = [spawn_points[i].location for i in route_1_indices]
        # Visualize the route in UE4 editor and PyGame Window
        X_route, Y_route = show_route(route_1, world, grp)
        # Set the initial spawn point for the vehicle
        # Spawn point for TOWN 7
        spawn_point_1 = spawn_points[58]

        vehicle = world.spawn_actor(bp, spawn_point_1)
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True)
        # Give Traffic Manager control over the vehicle
        traffic_manager.set_path(vehicle, route_1)

        # Set parameters of TM vehicle control, we don't want lane changes
        traffic_manager.update_vehicle_lights(vehicle, True)
        traffic_manager.random_left_lanechange_percentage(vehicle, 0)
        traffic_manager.random_right_lanechange_percentage(vehicle, 0)
        traffic_manager.auto_lane_change(vehicle, False)
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        traffic_manager.ignore_signs_percentage(vehicle, 100)
        traffic_manager.distance_to_leading_vehicle(vehicle, 0)
        traffic_manager.set_desired_speed(vehicle, desired_speed)

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[2, 3], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)),
                      vehicle, {}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                      vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)),
                      vehicle, {}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)),
                      vehicle, {}, display_pos=[1, 1])

        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 0])
        SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])

        # Logging X and Y Data for comparison
        X = np.array([spawn_point_1.location.x])
        Y = np.array([spawn_point_1.location.y])
        last_location = spawn_point_1.location

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

            if vehicle and vehicle.get_transform().location.distance(
                    route_1[-1]) > 0.8 and vehicle.get_transform().location.distance(
                    last_location) > sampling_resolution:
                X = np.append(X, vehicle.get_transform().location.x)
                Y = np.append(Y, vehicle.get_transform().location.y)
                last_location = vehicle.get_transform().location

            # Stop the vehicle at the final destination
            if vehicle and vehicle.get_transform().location.distance(route_1[-1]) < 0.8:
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
                break



    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)
        time.sleep(0.5)
        plt.plot(-1 * X_route[2:], Y_route[2:], 'bo',
                 label='Optimal line')  # somehow when spawning a vehicle the coordinate (0,0) is added still unclear why???
        plt.plot(-1 * X[2:], Y[2:], 'ro',
                 label='Autopilot')  # somehow when spawning a vehicle the coordinate (0,0) is added still unclear why???
        plt.annotate('Start Point', xy=(-spawn_point_1.location.x, spawn_point_1.location.y),
                     xytext=(-spawn_point_1.location.x - 40, spawn_point_1.location.y),
                     arrowprops=dict(facecolor='black', shrink=0.05),
                     )
        plt.annotate('End Point', xy=(-route_1[-1].x, route_1[-1].y),
                     xytext=(-route_1[-1].x - 40, route_1[-1].y),
                     arrowprops=dict(facecolor='black', shrink=0.05),
                     )
        plt.legend()
        plt.show()
        np.savez('XY_automatic', X, Y)
        np.savez('XY_optimal_route', X_route, Y_route)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        # default='1280x720',
        default='1920x1080',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
