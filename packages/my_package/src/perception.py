import cv2
import numpy as np

class LanePerception:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height

    def _grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def _gaussian_blur(self, img, kernel_size=5):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def _canny(self, img, low=180, high=240):
        return cv2.Canny(img, low, high)

    def _region_of_interest(self, img):
        mask = np.zeros_like(img)
        rows, cols = img.shape[:2]
        bottom_left  = [cols*0.15, rows]
        top_left     = [cols*0.45, rows*0.6]
        bottom_right = [cols*0.95, rows]
        top_right    = [cols*0.55, rows*0.6] 
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(img, mask)

    def detect(self, image):
        """Run lane detection pipeline. 
        Returns processed image + lookahead point (x, y)."""

        # Preprocess
        gray = self._grayscale(image)
        blur = self._gaussian_blur(gray)
        edges = self._canny(blur)
        masked = self._region_of_interest(edges)

        # Hough Transform
        lines = cv2.HoughLinesP(masked, 1, np.pi/180, 20, np.array([]), 20, 180)

        if lines is None:
            return image, (self.width//2, self.height//2)  # fallback center

        left, right = [], []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2-y1)/(x2-x1+1e-6)
                if slope < 0:
                    left.append((x1, y1, x2, y2))
                else:
                    right.append((x1, y1, x2, y2))

        def avg_line(line_set):
            if len(line_set) == 0:
                return None
            x1s, y1s, x2s, y2s = zip(*line_set)
            return int(np.mean(x1s)), int(np.mean(y1s)), int(np.mean(x2s)), int(np.mean(y2s))

        left_line = avg_line(left)
        right_line = avg_line(right)

        # Midline
        if left_line and right_line:
            lx1, ly1, lx2, ly2 = left_line
            rx1, ry1, rx2, ry2 = right_line
            mid_x1 = (lx1+rx1)//2
            mid_y1 = (ly1+ry1)//2
            mid_x2 = (lx2+rx2)//2
            mid_y2 = (ly2+ry2)//2
            cv2.line(image, (mid_x1, mid_y1), (mid_x2, mid_y2), (0,255,0), 5)
            midline = (mid_x1, mid_y1, mid_x2, mid_y2)
        else:
            midline = (self.width//2, self.height, self.width//2, int(self.height*0.6))

        # Lookahead point (on midline, at 60% height)
        y_lookahead = int(self.height*0.6)
        x1, y1, x2, y2 = midline
        if (y2-y1) != 0:
            slope = (x2-x1)/(y2-y1)
            x_lookahead = int(x1 + slope*(y_lookahead-y1))
        else:
            x_lookahead = (x1+x2)//2

        lookahead_point = (x_lookahead, y_lookahead)
        cv2.circle(image, lookahead_point, 5, (0,0,255), -1)

        return image, lookahead_point
