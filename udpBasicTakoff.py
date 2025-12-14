# PATCH: Fix for Python 3.10+ compatibility with dronekit (MutableMapping issue)
import collections
import collections.abc
for name in dir(collections.abc):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))

from dronekit import connect, VehicleMode
import time

# Connect to the vehicle    
print("Connecting to vehicle on: 'udp:127.0.0.1:14550'")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == 'GUIDED':
    print(" Waiting for mode change ...")
    time.sleep(1)   
    
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)   

print("Armed")
vehicle.simple_takeoff(10)   

while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    if vehicle.location.global_relative_frame.alt>=9.5: 
        print("Reached target altitude")
        break
    time.sleep(1)    
    print("Test Passed: Vehicle took off successfully to 10 meters")

# Close vehicle object before exiting script
print("Closing vehicle connection")
vehicle.close()