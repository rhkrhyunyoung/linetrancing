import numpy as np

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

BEV_POINTS = np.float32([
    [50, 480],   
    [590, 480],  
    [250, 150],  
    [390, 150]   
])

PID_P = 0.5    
PID_I = 0.01   
PID_D = 0.1    

BASE_SPEED = 30      

TRACK_WIDTH = 0.5    

WHEEL_RADIUS = 0.08
