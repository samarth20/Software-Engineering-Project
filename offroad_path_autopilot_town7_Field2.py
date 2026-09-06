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
            #Field2
            (9.6, 8.5, 0.8), (19.6, 8.5, 0.8), (29.6, 8.5, 0.8), (39.6, 8.5, 0.8), (49.6, 8.5, 0.8), (59.6, 8.5, 0.8),
            (69.6, 12.5, 0.8),

            (69.6, 22.5, 0.8), (59.6, 17.5, 0.8), (49.6, 17.5, 0.8), (39.6, 17.5, 0.8), (29.6, 17.5, 0.8),
            (19.6, 17.5, 0.8), (9.6, 17.5, 0.8),

            (9.6, 27.5, 0.8), (19.6, 27.5, 0.8), (29.6, 27.5, 0.8), (39.6, 27.5, 0.8), (49.6, 27.5, 0.8),
            (59.6, 27.5, 0.8), (69.6, 32.5, 0.8),

            (72.6, 45.5, 0.8), (59.6, 27.5, 0.8), (59.6, 37.5, 0.8), (49.6, 37.5, 0.8), (39.6, 37.5, 0.8),
            (29.6, 37.5, 0.8),
            (19.6, 37.5, 0.8), (9.6, 37.5, 0.8),

            (9.6, 47.5, 0.8), (19.6, 47.5, 0.8), (29.6, 47.5, 0.8), (39.6, 47.5, 0.8), (49.6, 47.5, 0.8),
            (59.6, 47.5, 0.8)
            ]

        # Convert to carla.Location
        waypoints = [carla.Location(x=wp[0], y=wp[1], z=wp[2]) for wp in custom_waypoints]
        # waypoints_1 = [carla.Location(x=wp[0], y=wp[1], z=wp[2]) for wp in custom_waypoints_1]

        # Visualize the waypoints
        for waypoint in waypoints:
            world.debug.draw_point(waypoint, size=0.2, color=carla.Color(255, 0, 0), life_time=80.0)
        print("Waypoints visualized.")

        draw_waypoint_lines(world, waypoints)


        # # Spawn a vehicle at the first waypoint
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # Select any vehicle
        # print(vehicle_bp)
        spawn_transform = carla.Transform(location=waypoints[0], rotation=carla.Rotation())
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)

        if vehicle is None:
            print("Failed to spawn the vehicle. Ensure the spawn location is valid.")
            return
        print("Vehicle spawned successfully.")

        # Move the vehicle along the waypoints
        follow_custom_path(vehicle, waypoints, world)
        # follow_custom_path(vehicle, waypoints_1, world)


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
                              life_time=80.0)  # Long enough to persist

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
