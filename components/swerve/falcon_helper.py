FALCON_PULSE_COUNT = 2048
FALCON_SLOT_IDX = 0
FALCON_MAX_RPM = 6380
FALCON_MAX_RPS = FALCON_MAX_RPM / 60

CANCODER_PULSE_COUNT = 4096
    
def mps_to_falcon(speed: float, distance_per_rotation: float = 1, gear_ratio: float = 1) -> float:
    """
    Converts a speed in meters per second to a Falcon 500-compatible [-1, 1] range.
    
    Gear ratio is the number of Falcon rotations per wheel rotation.
    
    This is a simple conversion function, performing no sanity checks. It is on the user to 
    ensure that output is safe and sane.
    """
    return speed / (distance_per_rotation / FALCON_MAX_RPS / gear_ratio)

def falcon_to_mps(speed: float, distance_per_rotation: float = 1, gear_ratio: float = 1) -> float:
    """
    Converts a Falcon 500 speed value in the range [-1, 1] to meters per second. 

    Gear ratio is the number of Falcon rotations per wheel rotation.
    
    This is a simple conversion function, performing no sanity checks. It is on the user to 
    ensure that output is safe and sane.
    """
    return (distance_per_rotation / FALCON_MAX_RPS / gear_ratio) * speed

def degrees_to_falcon(angle: float) -> float:
    """
    Converts an angle in degrees to a Falcon 500-compatible value.
    
    This is a simple conversion function, performing no sanity checks. It is on the user to 
    ensure that output is safe and sane.
    """
    return FALCON_PULSE_COUNT / 360.0 * angle

def degrees_to_CANCoder(angle: float) -> float:
    """
    Converts an angle in degrees to the scale comparable to a CANCoder.
    
    This is a simple conversion function, performing no sanity checks. It is on the user to 
    ensure that output is safe and sane.
    """
    return CANCODER_PULSE_COUNT / 360.0 * angle

