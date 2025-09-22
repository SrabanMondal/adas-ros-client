# packages/my_package/src/utils.py

import cv2
import numpy as np

class LineDetector:
    """
    Ek class jo Duckiebot ke camera feed se safed aur neeli lines detect karti hai.
    """
    def __init__(self):
        # --- HSV Color Ranges ---
        # Yeh values aapko apni lighting conditions ke hisaab se tune karni pad sakti hain
        # Safed rang ke liye HSV range
        self.white_lower = np.array([0, 0, 0])
        self.white_upper = np.array([0, 0, 0])
        # Neele rang ke liye HSV range
        self.blue_lower = np.array([0, 0, 0])
        self.blue_upper = np.array([0, 0, 0])

    def _detect_line(self, image, hsv_lower, hsv_upper):
        """
        Helper function jo diye gaye HSV range ke hisaab se line detect karti hai.
        Agar line milti hai, toh uska center (cx, cy) return karti hai, warna None.
        """
        # Image ko HSV color space mein convert karo
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Diye gaye rang ka mask banao
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
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
                return (cx, cy)
        
        return None

    def detect_white_line(self, image):
        """Safed line detect karti hai."""
        return self._detect_line(image, self.white_lower, self.white_upper)

    def detect_blue_line(self, image):
        """Neeli line detect karti hai."""
        return self._detect_line(image, self.blue_lower, self.blue_upper)