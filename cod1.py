from pymavlink import mavutil

# MAVLink
drone = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
drone.wait_heartbeat()
print('Конект з дроном')