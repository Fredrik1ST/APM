import cv2
import numpy as np
import time
# from matplotlib import pyplot as plt
# from scipy.optimize import minimize
# from scipy import stats
# from ltsfit.ltsfit import ltsfit


def line_detection(image, set_width, cut_top, angle_threshold=60, show_lines=False):
    def auto_canny(image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)*2.5
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # Battery box removalØ
        vertices = np.array([[330, 172], [340, 399], [591, 399], [444, 172]], dtype=np.int32)
        mask = np.ones_like(edged) * 255
        cv2.fillPoly(mask, [vertices], 0)
        edged = cv2.bitwise_and(edged, mask)
        # return the edged image
        return edged    

    
    height, width, _ = image.shape
    # print("Height: ", height, "Width: ", width)
    # Crop ROI and resize to 640px widt:
    


    height, width, _ = image.shape
    # print("Height: ", height, "Width: ", width)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # OPTIONAL: Apply Gaussian blur to reduce noise
    # gray = cv2.GaussianBlur(gray,(11,11),0)
 
    # Use Canny edge detection to find edges in the image
    edges = auto_canny(gray)
    from matplotlib import pyplot as plt
    """cv2.imshow("canny",edges)
    cv2.waitKey(0)"""

        
    # Normal Hough transform. Results in rho, theta
    lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=90)

    # Draw and classify all lines:
    left_rho_theta=[]
    right_rho_theta=[]
    # print(lines)



    if lines is not None:
        # print(lines)
        # break
        line_left = []
        line_right = []
        for line in lines:
            rho,theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if theta < angle_threshold*np.pi/180:
                line_left.append([x1, y1])
                line_left.append([x2, y2])
                cv2.line(image,(x1,y1),(x2,y2),(0,100,0),2)
                left_rho_theta.append([rho,theta])
            elif theta > (np.pi - angle_threshold*np.pi/180):
                line_right.append([x1, y1])
                line_right.append([x2, y2])
                cv2.line(image,(x1,y1),(x2,y2),(0,0,100),2)
                right_rho_theta.append([rho,theta])
            else:   # Not left or right lane:
                # cv2.line(image,(x1,y1),(x2,y2),(255,0,0),2)
                pass
    else:
        # print("Did not find any lines")
        return None
    
    if not (line_left and line_right):
        # print("Did NOT find BOTH lines")
        return None 
        
    
      
    # Define the function to optimize (sum of squared errors)
    def error_function(params, data):
        a, b = params
        x, y = data.T
        predicted = a * x + b
        error = np.sum((y - predicted)**2)
        return error

    def draw_line(a,b,rgb,width=2,image=image):
        cv2.line(image,(0,int(b)),(1000,int(a*1000+b)),rgb,width)

    def rho_theta_to_ab(rho, theta):
        if np.sin(theta) == 0:
            a = -np.cos(theta) / 0.001
            b = rho / 0.001
        else:
            a = -np.cos(theta) / np.sin(theta)
            b = rho / np.sin(theta)
        return a, b
 

    """if show_lines:
        cv2.imshow("alle streker", image)
        cv2.waitKey(0)"""


    line_left = np.array(line_left)
    line_right = np.array(line_right)   

    # Line segments defined with star(x1, y1) and end(x2, y2)
    x1=line_left[:,0]
    y1=line_left[:,1]
    x2=line_right[:,0]
    y2=line_right[:,1]



    # Choose line fitting method:
    # -------------------------------------------

    # Median(PREFERED for hough transform):

    lines = np.array(left_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a1,b1 = rho_theta_to_ab(rho,theta)
    draw_line(a1,b1,(0,0,200),3)


    lines = np.array(right_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a2,b2 = rho_theta_to_ab(rho,theta)
    draw_line(a2,b2,(200,0,0),3)

    """cv2.imshow("alle streker", image)
    cv2.waitKey(0)"""

    return [a1, b1, a2, b2]