import carla
import math
import time
import pygame

def main():
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # Define custom off-road waypoints
        custom_waypoints = [
            # (1031.692, 657.369, 120.359),
            # (1033.042358, 655.785034, 120.373627),
            # (1034.369507, 654.176270, 120.387413),
            # (1035.671997, 652.542786, 120.401192),
            # (1036.946655, 650.886658, 120.414978),
            # (1038.192993, 649.208923, 120.428764),
            # (1039.410278, 647.510071, 120.442543),
            # (1040.598267, 645.790588, 120.456329),
            # (1041.756714, 644.051086, 120.470116),
            # (1042.884888, 642.291931, 120.483894),
            # (1043.983154, 640.513672, 120.497681),
            # (1045.050537, 638.716858, 120.511467),
            # (1046.087036, 636.902039, 120.525246),
            # (1047.092163, 635.069763, 120.539032),
            # (1048.065796, 633.220520, 120.552818),
            # (1049.007812, 631.354797, 120.566597),
            # (1049.917480, 629.473267, 120.580383),
            # (1050.794922, 627.576416, 120.594170),
            # (1051.639648, 625.664795, 120.607948),
            # (1052.451538, 623.739014, 120.621735),
            # (1053.230347, 621.799561, 120.635513),
            # (984.024353, 456.863922, 125.750000),
            (1072, 612, 120.700), (1092, 612, 120.700), (1112, 612, 120.700), (1132, 612, 120.700), (1152, 612, 120.700), (1172, 612, 120.700), (1192, 612, 120.700),

            (1212, 596.5, 120.700),
            (1192, 581, 120.700),(1172, 581, 120.700), (1152, 581, 120.700), (1132, 581, 120.700), (1112, 581, 120.700), (1092, 581, 120.700),



            (1072, 565.5, 122), (1092, 550, 120.700), (1112, 550, 120.700), (1132, 550, 120.700), (1152, 550, 120.700),(1172, 550, 120.700), (1192, 550, 120.700)
        ]

        # Convert to carla.Location
        waypoints = [carla.Location(x=wp[0], y=wp[1], z=wp[2]) for wp in custom_waypoints]

        # Visualize the waypoints
        for waypoint in waypoints:
            world.debug.draw_point(waypoint, size=0.1, color=carla.Color(255, 0, 0), life_time=60.0)
        print("Waypoints visualized.")

        draw_waypoint_lines(world, waypoints)


        # Spawn a vehicle at the first waypoint
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # Select any vehicle
        spawn_transform = carla.Transform(location=waypoints[0], rotation=carla.Rotation())
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)

        if vehicle is None:
            print("Failed to spawn the vehicle. Ensure the spawn location is valid.")
            return
        print("Vehicle spawned successfully.")

        # Move the vehicle along the waypoints
        follow_custom_path(vehicle, waypoints, world)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if 'vehicle' in locals() and vehicle:
            vehicle.destroy()
            print("Vehicle destroyed.")

def draw_waypoint_lines(world, waypoints):
    """Draw green lines connecting waypoints in order."""
    for i in range(len(waypoints) - 1):
        world.debug.draw_line(waypoints[i], waypoints[i + 1],
                              thickness=0.1,
                              color=carla.Color(0, 0, 255),  # Green color
                              life_time=60.0)  # Long enough to persist

def follow_custom_path(vehicle, waypoints, world, speed=10.0):
    """Move the vehicle along custom waypoints."""
    for i, waypoint in enumerate(waypoints):
        print(f"Driving to waypoint {i+1}/{len(waypoints)}: {waypoint}")
        drive_to_waypoint(vehicle, waypoint, speed, world)

def drive_to_waypoint(vehicle, waypoint, speed, world):
    """Drive the vehicle to a specific waypoint using proportional control with better heading alignment."""
    target_reached = False
    waypoint_timeout = 30  # Max time (in seconds) to reach a waypoint
    start_time = time.time()

    while not target_reached:
        vehicle_transform = vehicle.get_transform()
        current_location = vehicle_transform.location

        # Calculate distance to waypoint
        distance = current_location.distance(waypoint)

        # Stop condition: Close enough to waypoint
        if distance < 1.0:
            target_reached = True
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            print(f"Waypoint reached at {waypoint}.")
            break

        # Calculate direction and steering
        direction = waypoint - current_location
        target_yaw = math.atan2(direction.y, direction.x) * 180 / math.pi
        current_yaw = vehicle_transform.rotation.yaw

        # Calculate yaw error and limit it to [-180, 180]
        yaw_error = (target_yaw - current_yaw + 180) % 360 - 180

        # Introduce a tighter heading threshold
        if abs(yaw_error) < 5.0 and distance < 5.0:
            print("Alignment corrected.")
            target_reached = True
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            break

        # Apply proportional steering control
        steer = max(-1.0, min(1.0, yaw_error / 30.0))  # Adjust divisor for smoother steering
        throttle = max(0.5, min(0.6, distance / 10.0))  # Adjust throttle based on distance

        # Apply control
        vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))

        # Debug visualization
        world.debug.draw_string(current_location, '.', draw_shadow=False,
                                color=carla.Color(r=0, g=255, b=0), life_time=0.1)

        # Timeout condition to prevent infinite loop
        if time.time() - start_time > waypoint_timeout:
            print(f"Timeout reached for waypoint: {waypoint}")
            break

        time.sleep(0.05)



if __name__ == "__main__":
    main()
