import cv2
import numpy as np
import math
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

# Edge detection parameters - initial values
CANNY_LOW = 30
CANNY_HIGH = 255
BLUR_SIZE = 17
SOBEL_KERNEL = 3

# Color detection ranges for different color spaces
HSV_BLUE_RANGE_1 = ([105, 65, 40], [135, 255, 255])
HSV_BLUE_RANGE_2 = ([15, 62, 10], [169, 220, 68])
HSV_RED_RANGE_1 = ([0, 140, 50], [10, 255, 255])  # Red wraps around in HSV
HSV_RED_RANGE_2 = ([160, 140, 50], [180, 255, 255])
HSV_YELLOW_RANGE = ([6, 184, 100], [30, 255, 255])

# Constants for filtering contours
SMALL_CONTOUR_AREA = 200

# Minimum average brightness threshold (0-255)
MIN_BRIGHTNESS_THRESHOLD = 0


def calculate_angle(contour):
    if len(contour) < 5:
        return 0
    (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
    return angle


def draw_info(image, color, angle, center, index, area):
    cv2.putText(
        image,
        f"#{index}: {color}",
        (center[0] - 40, center[1] - 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        image,
        f"Angle: {angle:.2f}",
        (center[0] - 40, center[1] - 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        image,
        f"Area: {area:.2f}",
        (center[0] - 40, center[1] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )
    cv2.circle(image, center, 5, (0, 255, 0), -1)
    cv2.line(
        image,
        center,
        (
            int(center[0] + 50 * math.cos(math.radians(90 - angle))),
            int(center[1] - 50 * math.sin(math.radians(90 - angle))),
        ),
        (0, 255, 0),
        2,
    )


def separate_touching_contours(contour, min_area_ratio=0.15):
    x, y, w, h = cv2.boundingRect(contour)
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

    original_area = cv2.contourArea(contour)
    max_contours = []
    max_count = 1

    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 3)

    for threshold in np.linspace(0.1, 0.9, 9):
        _, thresh = cv2.threshold(
            dist_transform, threshold * dist_transform.max(), 255, 0
        )
        thresh = np.uint8(thresh)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid_contours = [
            c for c in contours if cv2.contourArea(c) > original_area * min_area_ratio
        ]

        if len(valid_contours) > max_count:
            max_count = len(valid_contours)
            max_contours = valid_contours

    if max_contours:
        return [c + [x, y] for c in max_contours]
    return [contour]


def nothing(x):
    pass


def process_color(frame, mask, color_name, color_bgr):
    blur_size = BLUR_SIZE
    sobel_kernel = SOBEL_KERNEL

    kernel = np.ones((5, 5), np.uint8)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)

    gray_boosted = cv2.addWeighted(gray_masked, 1.5, mask, 0.5, 0)

    blurred = cv2.GaussianBlur(gray_boosted, (blur_size, blur_size), 0)

    sobelx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=sobel_kernel)

    magnitude = np.sqrt(sobelx**2 + sobely**2)
    magnitude = np.uint8(magnitude * 255 / np.max(magnitude))

    _, edges = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)

    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=3)
    edges = cv2.bitwise_not(edges)
    edges = cv2.bitwise_and(edges, edges, mask=mask)

    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    return contours, hierarchy, gray_masked


def runPipeline(frame, llrobot):
    try:
        llpython = [0, 0, 0, 0, 0, 0, 0, 0]
        largest_contour = np.array([[]])

        # Convert to HSV and denoise
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get HSV value at center of frame
        center_y = frame.shape[0] // 2
        center_x = frame.shape[1] // 2
        center_hsv = hsv[center_y, center_x]

        hsv_denoised = cv2.GaussianBlur(hsv, (5, 5), 0)
        hsv_denoised = hsv

        # Create masks for each color
        blue_mask = cv2.inRange(
            hsv_denoised, np.array(HSV_BLUE_RANGE_1[0]), np.array(HSV_BLUE_RANGE_1[1])
        )
        blue_mask2 = cv2.inRange(
            hsv_denoised, np.array(HSV_BLUE_RANGE_2[0]), np.array(HSV_BLUE_RANGE_2[1])
        )
        red_mask1 = cv2.inRange(
            hsv_denoised, np.array(HSV_RED_RANGE_1[0]), np.array(HSV_RED_RANGE_1[1])
        )
        red_mask2 = cv2.inRange(
            hsv_denoised, np.array(HSV_RED_RANGE_2[0]), np.array(HSV_RED_RANGE_2[1])
        )
        #blue_mask = cv2.bitwise_or(blue_mask1, blue_mask2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        yellow_mask = cv2.inRange(
            hsv_denoised, np.array(HSV_YELLOW_RANGE[0]), np.array(HSV_YELLOW_RANGE[1])
        )

        def apply_color(image, mask, color):
            colored = np.zeros_like(image)
            colored[mask > 0] = color
            return colored

        red_area = apply_color(frame, red_mask, (0, 0, 255))
        yellow_area = apply_color(frame, yellow_mask, (0, 255, 255))
        blue_area = apply_color(frame, blue_mask, (255, 0, 0))

        colors_only = cv2.bitwise_or(red_area, yellow_area)
        colors_only = cv2.bitwise_or(colors_only, blue_area)

        # Process each color separately
        blue_contours, blue_hierarchy, blue_gray = process_color(
            frame, blue_mask, "Blue", (255, 0, 0)
        )
        red_contours, red_hierarchy, red_gray = process_color(
            frame, red_mask, "Red", (0, 0, 255)
        )
        yellow_contours, yellow_hierarchy, yellow_gray = process_color(
            frame, yellow_mask, "Yellow", (0, 255, 255)
        )

        all_contours = [
            (blue_contours, blue_hierarchy, blue_gray, "Blue", (140, 0, 0)),
            (red_contours, red_hierarchy, red_gray, "Red", (0, 0, 140)),
            (yellow_contours, yellow_hierarchy, yellow_gray, "Yellow", (0, 140, 140)),
        ]

        game_pieces = []

        for contours, hierarchy, gray, color, color_bgr in all_contours:
            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) < SMALL_CONTOUR_AREA:
                    continue

                for sep_contour in separate_touching_contours(contour):
                    mask = np.zeros(gray.shape, dtype=np.uint8)
                    cv2.drawContours(mask, [sep_contour], -1, 255, -1)

                    if cv2.mean(gray, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
                        continue

                    M = cv2.moments(sep_contour)
                    if M["m00"] == 0:
                        continue

                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    angle = calculate_angle(sep_contour)
                    area = cv2.contourArea(sep_contour)

                    cv2.drawContours(colors_only, [sep_contour], 0, color_bgr, 2)
                    draw_info(
                        colors_only, color, angle, center, len(game_pieces) + 1, area
                    )

                    game_pieces.append(
                        {
                            "index": len(game_pieces) + 1,
                            "color": color,
                            "position": center,
                            "angle": angle,
                            "area": area,
                            "brightness": cv2.mean(gray, mask=mask)[0],
                            "contour": sep_contour,
                            "hierarchy_level": (
                                "external" if hierarchy[0][i][3] == -1 else "internal"
                            ),
                            "difficulty": 0,
                        }
                    )

        # Edit these values please cookie monster
        CLOSE_DISTANCE = 0
        MEDIUM_DISTANCE = 0

        # unobstructed areas
        CLOSE_AREA = 3000
        MEDIUM_AREA = 2000
        FAR_AREA = 1000

        UPPER_GOOD_ANGLE = 100
        LOWER_GOOD_ANGLE = 80
        UPPER_MEDIUM_ANGLE = 80
        LOWER_MEDIUM_ANGLE = 60

        
        good_angle = False
        medium_angle = False
        bad_angle = False
        
        far_distance = False
        medium_distance = False
        close_distance = False
        
        far_area = False
        medium_area = False
        close_area = False

        no_obstruction = False
        some_obstruction = False
        full_obstruction = False

        def dist(position):
            return math.sqrt(position[0] ** 2 + position[1] ** 2)

        for game_piece in game_pieces:
            difficulty = 0
            expected_area = 0

            angle = game_piece["angle"]
            area = game_piece["area"]
            distance = dist(game_piece["position"])

            # Distance
            if distance < CLOSE_DISTANCE:
                close_distance = True
            elif distance < MEDIUM_DISTANCE:
                medium_distance = True
            else:
                far_distance = True

            # Area
            if area > CLOSE_AREA:
                expected_area = CLOSE_AREA
                close_area = True
            elif area > MEDIUM_AREA:
                expected_area = MEDIUM_AREA
                medium_area = True
            elif area > FAR_AREA:
                expected_area = FAR_AREA
                far_area = True

            # Angle
            if angle > LOWER_GOOD_ANGLE and angle < UPPER_GOOD_ANGLE:
                good_angle = True
            elif angle > LOWER_MEDIUM_ANGLE and angle < UPPER_MEDIUM_ANGLE:
                medium_angle = True
            else:
                bad_angle = True

            # Obstruction
            if close_distance:
                if close_area:
                    no_obstruction = True
                elif medium_area:
                    some_obstruction = True
                else:
                    full_obstruction = True

            elif medium_distance:
                if medium_area:
                    no_obstruction = True
                elif far_area:
                    some_obstruction = True
                else:
                    full_obstruction = True

            elif far_distance:
                if far_area:
                    no_obstruction = True
                else:
                    full_obstruction = True

            
            # Calculate difficulty and points based on conditions
            if full_obstruction:
                difficulty += 1000
            elif some_obstruction:
                difficulty += 100
            elif no_obstruction:
                difficulty += 10 - min(abs(area - expected_area), 10) # ensure this is smaller than the some obstruction case (100) (divide or subtract by a certain constant)

            if bad_angle:
                difficulty += 100
            elif medium_angle:
                difficulty += 50
            elif good_angle:
                difficulty += abs(90 - angle)

            if far_distance:
                difficulty += 100
            elif medium_distance:
                difficulty += 50
            elif close_distance:
                difficulty += abs(CLOSE_DISTANCE - distance) # ensure this is smaller than the medium distance case (50) (divide or subtract by a certain constant)

            game_piece["difficulty"] = difficulty

        game_pieces.sort(key=lambda x: x["difficulty"])

        if len(game_pieces) > 0:
            game_piece = game_pieces[0]
            llpython = [1, game_piece["position"][0], game_piece["position"][1], game_piece["angle"], 0, 0, 0, 0]
            largest_contour = game_piece["contour"]

        return largest_contour, colors_only, llpython

    except Exception as e:
        print(f"Error: {str(e)}")
        return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]


