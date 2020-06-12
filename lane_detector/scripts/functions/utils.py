#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from math import atan2


left_a, left_b, left_c = [],[],[]
right_a, right_b, right_c = [],[],[]

#puntos de recorte para ROI vista de pajaro (en %)

#top_left = (0.35,0.59)
#top_right = (0.68,0.59)
#bottom_left = (0.05, 0.9)
#bottom_right = (.94, 0.9)

top_left = (0.28,0.6)
top_right = (0.7,0.6)
bottom_left = (0.05, 0.9)
bottom_right = (.94, 0.9)

#punto = np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)])
punto = np.float32([top_left,top_right,bottom_left,bottom_right])

#Variables para la recta
#m_pendiente = 0
#b_recta = 0
y_recta = 280
#x_recta = 0

def mapping(left, right, point=0.5):
	
	mapped_point = point*(right-left) + left
	
	return mapped_point

    
def pipeline(img, s_thresh=(100, 255), sx_thresh=(15, 255)):
   
    img = np.copy(img)

    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    # h_channel = hls[:,:,0] no se usa
    
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 255
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 255
    
    # color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) no se usa 
    
    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(s_binary == 255) | (sxbinary == 255)] = 255

    return combined_binary

def perspective_warp(img, 
                     dst_size=(1280,720),
                     src=punto,
                     dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    
    src = src* img_size

    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    
    return warped

def inv_perspective_warp(img, 
                     dst_size=(1280,720),
                     src=np.float32([(0,0), (1, 0), (0,1), (1,1)]),
                     dst=punto):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def get_hist(img):
    hist = np.sum(img[img.shape[0]//2:,:], axis=0)
    return hist
    
def sliding_window(img, nwindows=20, margin=30, minpix = 0.1, draw_windows=True):
    global left_a, left_b, left_c,right_a, right_b, right_c 
    left_fit_= np.empty(300)
    right_fit_ = np.empty(8)
    out_img = np.dstack((img, img, img))*255

    histogram = get_hist(img)
    # find peaks of left and right halves
    midpoint = int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    
    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        if draw_windows == True:
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (100,255,255), 3) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (100,255,255), 3) 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        
        
#        if len(good_right_inds) > minpix:        
#            rightx_current = np.int(np.mean([leftx_current +900, np.mean(nonzerox[good_right_inds])]))
#        elif len(good_left_inds) > minpix:
#            rightx_current = np.int(np.mean([np.mean(nonzerox[good_left_inds]) +900, rightx_current]))
#        if len(good_left_inds) > minpix:
#            leftx_current = np.int(np.mean([rightx_current -900, np.mean(nonzerox[good_left_inds])]))
#        elif len(good_right_inds) > minpix:
#            leftx_current = np.int(np.mean([np.mean(nonzerox[good_right_inds]) -900, leftx_current]))


    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    left_a.append(left_fit[0])
    left_b.append(left_fit[1])
    left_c.append(left_fit[2])
    
    right_a.append(right_fit[0])
    right_b.append(right_fit[1])
    right_c.append(right_fit[2])
    
    left_fit_[0] = np.mean(left_a[-10:])
    left_fit_[1] = np.mean(left_b[-10:])
    left_fit_[2] = np.mean(left_c[-10:])
    
    right_fit_[0] = np.mean(right_a[-10:])
    right_fit_[1] = np.mean(right_b[-10:])
    right_fit_[2] = np.mean(right_c[-10:])
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0] )
    left_fitx = left_fit_[0]*ploty**2 + left_fit_[1]*ploty + left_fit_[2]
    right_fitx = right_fit_[0]*ploty**2 + right_fit_[1]*ploty + right_fit_[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 100]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 100, 255]
    cv2.line(out_img, (midpoint, img.shape[0]-100),(midpoint, img.shape[0]-100), (0, 0, 255),8)
    
    return out_img, (left_fitx, right_fitx), (left_fit_, right_fit_), ploty

def get_curve(img, leftx, rightx):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    y_eval = np.max(ploty)
    ym_per_pix = 30.5/img.shape[0] # meters per pixel in y dimension
    xm_per_pix = 3.7/img.shape[0] # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

    car_pos = img.shape[1]/2
    l_fit_x_int = left_fit_cr[0]*img.shape[0]**2 + left_fit_cr[1]*img.shape[0] + left_fit_cr[2]
    r_fit_x_int = right_fit_cr[0]*img.shape[0]**2 + right_fit_cr[1]*img.shape[0] + right_fit_cr[2]
    lane_center_position = (r_fit_x_int + l_fit_x_int) /2
    center = (car_pos - lane_center_position) * xm_per_pix / 10
    # Now our radius of curvature is in meters
    return (left_curverad, right_curverad)

def draw_lanes(img, left_fit, right_fit):

	height, width = img.shape[:2]

	ploty = np.linspace(0, height-1, height)
	color_img = np.zeros_like(img)
	
	left = np.array([np.transpose(np.vstack([left_fit, ploty]))])
	right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))])
	points = np.hstack((left, right))
	
	cv2.fillPoly(color_img, np.int_(points), (255,200,0))
	
	x1 = int( ( right[0][-1][0] + left[0][0][0] ) / 2 )
	y1 = int( ( right[0][-1][1] + left[0][0][1] ) / 2 )
	x2 = int( ( right[0][0][0] + left[0][-1][0] ) / 2 )
	y2 = int( ( right[0][0][1] + left[0][-1][1] ) / 2 )
	
	angle_lane = atan2((y2-y1), (x2-x1)) * 180/np.pi
	
	center_t = int(mapping(top_left[0], top_right[0])*width)
	center_b = int(mapping(bottom_left[0], bottom_right[0])*width)
	angle_car = atan2((height), (center_t-center_b)) * 180/np.pi
	steer_angle = -(angle_lane -angle_car)
	
	cv2.line(color_img, (x1, y1), (x2, y2), (0, 0, 255), 6)			#Esta es la linea central del carril (roja)
	
	#ajuste de parametros dependiendo roi
	cv2.line(color_img, (center_t, 0), (center_b, height), (0, 255, 0), 6)	#Esta es la linea central del vehiculo (verde)
	
		#Calcular el error lateral variando la altura en la imagen
	m_pendiente = (y2-y1)/(x2-x1)
	b_recta = y1 - m_pendiente*x1
	x_recta = (y_recta - b_recta)/m_pendiente
	#Dibuja la recta a la altura que queramos tomar el error lateral
	cv2.line(color_img, (190, y_recta), (420, y_recta), (0, 255, 255), 6)
	#Dibuja la recta de la base de la linea verde hasta el cruce de la altura con la linea roja para el error lateral
	cv2.line(color_img, (x_recta, y_recta), (center_b, height), (255, 0, 255), 6)
	cv2.imshow("dibujos", color_img)
	
	#El punto 2 es la parte de abajo de la imagen
	print('Error lateral en pixeles', x_recta-center_b)						#Esto la diferencia de las bases de las lineas
										#Es negativo cuando la camioneta esta del lado derecho
    #print('height' ,height)						
	#print(y1)								
										
	inv_perspective = inv_perspective_warp(color_img, dst_size=(width, height))
	inv_perspective = cv2.addWeighted(img, 1, inv_perspective, .7, 0)
	

	return inv_perspective, int(steer_angle), int(x_recta-center_b)

def image_roi(img, roi_pts):

	img_copy = img.copy()
	img_size = np.float32([(img.shape[1],img.shape[0])])
	roi_pts = roi_pts* img_size
	roi_pts = roi_pts.astype(int)
	mask = np.zeros(img_copy.shape[:2], np.uint8)
	cv2.drawContours(mask, [roi_pts], -1, (255, 255, 255), -1, cv2.LINE_AA)
	roi=cv2.bitwise_and(img_copy, img_copy, mask=mask)
	return roi
	
def avg_angles(warped):
	average = 0
	theta_sum = 0
	hough_lines = cv2.HoughLines(warped,1,np.pi/180,150)
	if hough_lines is not None:
		for lines in hough_lines:
			for rho, theta in lines:
				angulo = int(theta*180/np.pi)
				if angulo>90 :
					angulo = 90-(angulo%90)
				theta_sum += angulo
		average = theta_sum/len(hough_lines)
	
	return average*0.8
    
