import cv2
import numpy as np
import time
from collections import defaultdict

# Track OpenCV function calls and timing
opencv_stats = defaultdict(lambda: {"count": 0, "total_time": 0.0})


def track_opencv(func_name):
    def decorator(func):
        def wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            end = time.time()
            opencv_stats[func_name]["count"] += 1
            opencv_stats[func_name]["total_time"] += end - start
            return result

        return wrapper

    return decorator


# Wrap commonly used OpenCV functions
cv2.split = track_opencv("split")(cv2.split)
cv2.cvtColor = track_opencv("cvtColor")(cv2.cvtColor)
cv2.inRange = track_opencv("inRange")(cv2.inRange)
cv2.bitwise_and = track_opencv("bitwise_and")(cv2.bitwise_and)
cv2.bitwise_or = track_opencv("bitwise_or")(cv2.bitwise_or)
cv2.bitwise_not = track_opencv("bitwise_not")(cv2.bitwise_not)
cv2.morphologyEx = track_opencv("morphologyEx")(cv2.morphologyEx)
cv2.GaussianBlur = track_opencv("GaussianBlur")(cv2.GaussianBlur)
cv2.Sobel = track_opencv("Sobel")(cv2.Sobel)
cv2.Canny = track_opencv("Canny")(cv2.Canny)
cv2.findContours = track_opencv("findContours")(cv2.findContours)
cv2.drawContours = track_opencv("drawContours")(cv2.drawContours)
cv2.bilateralFilter = track_opencv("bilateralFilter")(cv2.bilateralFilter)
cv2.normalize = track_opencv("normalize")(cv2.normalize)
cv2.dilate = track_opencv("dilate")(cv2.dilate)
cv2.contourArea = track_opencv("contourArea")(cv2.contourArea)

# Camera settings
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FPS = 90

# Camera exposure settings
EXPOSURE = 1700

# Camera gain settings
GAIN = 20

# Camera white balance settings
BLACK_LEVEL_OFFSET = 0
WB_RED = 1000
WB_BLUE = 1850

SOBEL_KERNEL = 9

# Color detection ranges for different color spaces
HSV_BLUE_RANGE_1 = ([105, 65, 40], [135, 255, 255])
HSV_BLUE_RANGE_2 = ([15, 62, 10], [169, 220, 68])
HSV_RED_RANGE_1 = ([0, 140, 50], [10, 255, 255])  # Red wraps around in HSV
HSV_RED_RANGE_2 = ([160, 140, 50], [180, 255, 255])
HSV_YELLOW_RANGE = ([6, 20, 100], [30, 255, 255])

# Constants for filtering contours
SMALL_CONTOUR_AREA = 100

def runPipeline(frame, llrobot):
    try:
        llpython = [0, 0, 0, 0, 0, 0, 0, 0]

        # Convert to HSV and denoise
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_denoised = cv2.GaussianBlur(hsv, (5, 5), 0)
        hsv_denoised = hsv

        # Create masks for each color
        yellow_mask = cv2.inRange(
            hsv_denoised, np.array(HSV_YELLOW_RANGE[0]), np.array(HSV_YELLOW_RANGE[1])
        )

        sobel_kernel = SOBEL_KERNEL
    
        kernel = np.ones((5, 5), np.uint8)
        masked_frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)
        gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
    
        sobelx = cv2.Sobel(gray_masked, cv2.CV_32F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray_masked, cv2.CV_32F, 0, 1, ksize=sobel_kernel)
    
        magnitude = np.sqrt(sobelx**2 + sobely**2)
        magnitude = np.uint8(magnitude * 255 / np.max(magnitude))
    
        _, edges = cv2.threshold(magnitude, 30, 255, cv2.THRESH_BINARY)
    
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=3)
        edges = cv2.bitwise_not(edges)
        edges = cv2.bitwise_and(edges, edges, mask=yellow_mask)
    
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pieces = []
        for contour in contours:
            area = cv2.contourArea(contour)
            cv2.drawContours(masked_frame, [contour], 0, (255, 255, 255), 2)
            if area < SMALL_CONTOUR_AREA:
                continue
            pieces.append({"contour": contour, "area": area})

        pieces.sort(key=lambda x: x["area"], reverse=True)
        if len(pieces) > 1:
            return pieces[0]["contour"], masked_frame, llpython
        return [], masked_frame, llpython
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]
