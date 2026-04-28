'''
Continually updates steering angle to keep the APM centered in the lane.

Relies on detected lines from the line detector to find their vanishing point,
then calculates the angle between the vanishing point and the bottom center of the image.

Also calculates the lane center of the current frame to detect offset.

Combined, these two metrics are used to calculate the desired steering angle to keep the
APM centered in the lane

'''