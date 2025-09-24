import cv2
import numpy as np

class LineDetector:
    def __init__(self):
        # --- HSV Color Ranges ---
        
        self.white_lower = np.array([0, 0, 0])
        self.white_upper = np.array([0, 0, 255])
        
        self.blue_lower = np.array([155, 10, 42])
        self.blue_upper = np.array([175, 20, 50])
        
        # self.blue_lower = np.array([155, 50, 40])
        # self.blue_upper = np.array([175, 255, 255])

    
        
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 30])

    def _detect_line(self, image, hsv_lower, hsv_upper):
        """
        Helper function to detect line
        """
        # Image ko HSV color space mein convert karo
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Diye gaye rang ka mask banao
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        processed_img = cv2.bitwise_and(image, image, mask=mask)
        processed_img = cv2.GaussianBlur(processed_img, (5,5), 0)

        # mask = cv2.erode(mask, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)
        # Mask mein se contours (shapes) dhoondho
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Sabse bada contour dhoondho
            largest_contour = max(contours, key=cv2.contourArea)
            # Uska center (centroid) calculate karo
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return processed_img, (cx, cy)
        
        return processed_img, None

    def detect_white_line(self, image):
        return self._detect_line(image, self.white_lower, self.white_upper)

    def detect_blue_line(self, image):
        return self._detect_line(image, self.blue_lower, self.blue_upper)

    def detect_black_line(self, image):
        return self._detect_line(image, self.black_lower, self.black_upper)