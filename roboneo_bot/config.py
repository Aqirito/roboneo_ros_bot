def load_config():
    return {
        'events_url'           : 'URL_HERE', # will be provided by event Instructor
        'secret_key'           : 'SECRET_KEY_HERE', # will be provided by event Instructor. Dont share with anyone
        'robot_id'             : 'ROBOT ID HERE', # will be provided by event Instructor
        'detected_distance'    : 20.0, # distance to consider as obstacle
        'forward_speed'        : 0.7,  # Forward speed
        'turn_90deg_duration'  : 0.7,  # seconds to turn ~90 degrees
        'turn_180deg_duration' : 1.45,  # seconds to turn ~180 degrees example turn from right to left
        'pillar_scan_duration' : 2.7,  # seconds for 360° rotation
        'scan_cooldown'        : 5.0,  # seconds between scans
        'min_lidar_distance'   : 10.0, # minimum distance to consider as pillar
        'max_lidar_distance'   : 20.0, # maximum distance to consider as pillar
    }
