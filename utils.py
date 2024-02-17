

def scale_axis(input):
    scale = 1.0
    deadband = 0.381021
    slope = (1 -.059) /(1-.38)
    offset = -(1 -.059) /(1-.38) +1
    second = 0.57

	#linear outside of deadband
    if (input < -deadband):
        return scale * (slope * input - offset)
    
	# polynomial inside of deadband
    if ((-deadband < input) and (input < deadband)): 
        return scale * (1.0 / (pow(second, 2.0)) * pow(input, 3.0))

	#linear outside of deadband
    else :
        return scale * (slope * input + offset)
    

