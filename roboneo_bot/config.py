def load_config():
    """
    Load configuration settings for the robot.
    Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
    """
    return {
        'events_url'            : 'https://pluto.kinabalumakers.dev/roboneo_2025/events', # will be provided by event Instructor
        'secret_key'            : 'D4WS34NNS57M', # will be provided by event Instructor. Dont share with anyone
        'robot_id'              : 'kinabalu_coders', # will be provided by event Instructor

        # Ultrasonic distance to consider as obstacle
        # Example within 0 to 17.0 cm
        'detected_distance'     : 17.0, 
        # Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
        # Example 0.5 = 127, -0.5 = -127, 1.0 = 255, -1.0 = -255
        'forward_speed'         : 0.7,  # Forward speed
        'reverse_speed'         : -0.5, # Reverse speed

        # Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
        'turn_speed'            : 1.0,  # Turning speed

        # Time durations for various maneuvers
        # Example 0.7 seconds to turn ~90 degrees
        'turn_90deg_duration'   : 0.7,  # seconds to turn ~90 degrees
        'turn_180deg_duration'  : 1.35,  # seconds to turn ~180 degrees example turn from right to left

        'floor_color_arena_border'      : 'Red', # the robot will avoid this color line, case sensitive 
        'floor_color_zone_one_color_0'  : 'Blue', # detect color to get points, case sensitive
        'floor_color_zone_one_color_1'  : 'Green', # detect color to get points, case sensitive
        
    
        ### **************** Pillar Detection Mode Settings ****************
        # Zone 2 color line to switch to Pillar Detection Mode
        # Example: Blue line to switch to Pillar Detection Mode
        ### ****************************************************************

        'floor_color_zone_two_entrance' : 'Black', # color of zone two, Pillar detection mode, case sensitive

        # This is used on Pillar scanning (Zone 2)
        # Example speed 0.5 mapped to 127.5
        'pillar_scan_turn_speed'        : 0.5, # Slow turn speed for pillar scanning

        # Time durations for pillar scanning maneuvers
        # Example 2.7 seconds for a full 360-degree rotation
        'pillar_scan_duration'          : 2.7,  # seconds for 360° rotation

        # Cooldown periods to prevent rapid state changes
        'scan_cooldown'                 : 5.0,  # seconds between scans

        # LiDAR distance thresholds for pillar detection (in cm)
        'min_lidar_distance'            : 150.0, # minimum distance to consider as pillar
        'max_lidar_distance'            : 160.0, # maximum distance to consider as pillar
    }
    