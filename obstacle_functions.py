# need the pip install coppeliasim-zmqremoteapi-client
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

# Create a client to connect to the ZeroMQ remote API server
client = RemoteAPIClient() # Defaults are host ='localhost', port=23000
sim = client.require('sim') # Get a remote object 'sim'

def get_obj_handle_zmq(name):
    try:
        handle = sim.getObject(name)
        print(f"{name}: OK!")
        return handle
    except Exception as e:
        print(f"Unable to obtain {name}.")
        return None

# Proximity sensor handle
prox_sensor_handle = get_obj_handle_zmq('/ProximitySensorName') # Placeholder

# Vision sensor handle
vis_sensor_handle = get_obj_handle_zmq('/VisionSensorName') # Placeholder

threshold_distance = 30 # Placeholder (meters or mm)
# Other controllable variables here


def color_detection():
    # Essentially the range of red
    # Lower bound for red (R: high, G: low, B: low)
    lower_red = np.array([150, 0, 0], dtype=np.uint8)
    # Upper bound for red (R: high, G: high, B: high, but within reason)
    upper_red = np.array([255, 100, 100], dtype=np.uint8)

    while True:
        try:
            # Image data
            img_bytes, resolution = sim.getVisionSensorCharImage(vis_sensor_handle)

            # Reshape the 1D byte array into a 3D image array (Height, Width, Channels)
            img = np.frombuffer(img_bytes, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)

            # Boolean mask for lower and upper bound
            lower_red_mask = np.all(img >= lower_red)
            upper_red_mask = np.all(img >= upper_red)
            # Combine masks (pixel is red if it meets both criteria)
            red_mask = np.logical_and(lower_red_mask,upper_red_mask)

            # Sum of red pixels
            red_pixels = np.sum(red_mask)

            # How many pixels needed to detect
            if red_pixels > 100:
                print("Obstacle detected")
            else:
                pass
        except KeyboardInterrupt:
            break

def proximity_detection():
    while sim.getSimulationTime > 0:
            try:
                result = sim.readProximitySensor(prox_sensor_handle)
                detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = result
                if result and detectionState:
                    # Distance from origin point of sensor
                    distance = (detectedPoint[0]**2 + detectedPoint[1]**2 + detectedPoint[2]**2)**0.5

                    if distance < threshold_distance:
                        print(f"Obstacle detected at {distance:.2f} meters!") # or mm?
                        # obstacle avoidance logic (PLACEHOLDER) 
                    else:
                        # normal movement (PLACEHOLDER)
                        pass
            except KeyboardInterrupt:
                break