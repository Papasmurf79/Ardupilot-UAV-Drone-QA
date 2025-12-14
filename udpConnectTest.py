# PATCH: Fix for Python 3.10+ compatibility with dronekit (MutableMapping issue)
import collections
import collections.abc
for name in dir(collections.abc):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))

from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle (in this case a UDP endpoint)
print("Connecting to vehicle on: 'udp:127.0.0.1:14550'")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

print("Vehicle connected")
print("Mode:", VehicleMode.name)
print("Armed:", vehicle.armed)
print("Location:", vehicle.location.global_frame)

# Close vehicle object before exiting script
print("Closing vehicle connection")
vehicle.close()
print("Vehicle connection closed")