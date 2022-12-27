# Modify this file to decide which drivers are compiled

DRIVERS = [
    # 'imu/icm20689.c',
    'imu/mpu6000.c',
    # 'imu/bmi055.c',
    # 'mag/ist8310.c',
    'barometer/spl06.c',
    # 'gps/gps_m8n.c',
    # 'rgb_led/ncp5623c.c',
    # 'mtd/ramtron.c',
    # 'vision_flow/mtf_01.c',
    # 'airspeed/ms4525.c',
      'mtd/w25qxx.c',
]

DRIVERS_CPPPATH = []