import numpy as np 
import cv2

lower_b = np.array([0, 100, 50])
upper_b = np.array([360, 255, 255])
s_gradient = np.ones((500,1), dtype=np.uint8)*np.linspace(lower_b[1], upper_b[1], 500, dtype=np.uint8)
v_gradient = np.rot90(np.ones((500,1), dtype=np.uint8)*np.linspace(lower_b[1], upper_b[1], 500, dtype=np.uint8))
h_array = np.arange(lower_b[0], upper_b[0]+1)

for hue in h_array:
    h = hue*np.ones((500,500), dtype=np.uint8)
    hsv_color = cv2.merge((h, s_gradient, v_gradient))
    rgb_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)
    print(hsv_color)
    cv2.imshow('', rgb_color)
    cv2.waitKey(250)

cv2.destroyAllWindows()
