def get_angle_relative(prevAngle, newAngle):
    clamp = min(360, max(-360, newAngle)) 
    return -prevAngle + clamp 

def reset_zero_position(currAngle):
    return get_angle_relative(currAngle, 0) 
