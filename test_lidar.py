import os
import ydlidar
import time
import sys

# Configure YDLidar
ydlidar.os_init()
laser = ydlidar.CYdLidar()

# Map the correct port (usually /dev/ttyUSB0 for the adapter)
port = "/dev/ttyUSB0"
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

print(f"Initializing LiDAR on {port}...")
ret = laser.initialize()

if ret:
    ret = laser.turnOn()
    if ret:
        print("LiDAR turned on successfully. Scanning...")
        scan = ydlidar.LaserScan()
        
        try:
            for i in range(20): # Scan for a short time
                r = laser.doProcessSimple(scan)
                if r:
                    print(f"Scan received: [{scan.stamp}] {scan.points.size()} points")
                    # Find min distance in front (-45 to 45 degrees)
                    min_dist = 999.0
                    for point in scan.points:
                        # Angle is in radians, convert to degrees
                        angle_deg = point.angle * 180.0 / 3.14159265
                        if -45.0 <= angle_deg <= 45.0 and point.range > 0:
                            if point.range < min_dist:
                                min_dist = point.range
                    if min_dist < 999.0:
                        print(f"  --> Closest obstacle in front: {min_dist:.2f} meters")
                else:
                    print("Failed to get scan data")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
            
        print("Stopping LiDAR...")
        laser.turnOff()
    else:
        print("Failed to turn on LiDAR.")
    laser.disconnecting()
else:
    print(f"Failed to initialize LiDAR. Please check if {port} exists and you have permissions (sudo or dialout group).")

