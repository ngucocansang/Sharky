import cv2
import numpy as np

# Taking input from the webcam
vid = cv2.VideoCapture(0)

while True:
    # Capturing the current frame
    _, frame = vid.read()

    # Get the frame dimensions
    height, width, _ = frame.shape

    # Convert the frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for green color in HSV
    lower_green = np.array([35, 100, 100])  # Adjust as needed
    upper_green = np.array([85, 255, 255])  # Adjust as needed

    # Create a mask to extract green color
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Bitwise-AND mask with the original frame
    green_detection = cv2.bitwise_and(frame, frame, mask=mask)

    # Draw grid lines on the green detection frame
    grid_color = (255, 255, 255)  # White color for the grid
    thickness = 1  # Thickness of the grid lines

    # Vertical lines (split the frame into 3 sections horizontally)
    cv2.line(green_detection, (width // 3, 0), (width // 3, height), grid_color, thickness)
    cv2.line(green_detection, (2 * width // 3, 0), (2 * width // 3, height), grid_color, thickness)

    # Horizontal lines (split the frame into 3 sections vertically)
    cv2.line(green_detection, (0, height // 3), (width, height // 3), grid_color, thickness)
    cv2.line(green_detection, (0, 2 * height // 3), (width, 2 * height // 3), grid_color, thickness)

    # Find contours of the green object
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find the largest contour (assumes it's the green object)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding rectangle for the contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Calculate the center of the green object
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw a circle at the center of the green object
        cv2.circle(green_detection, (center_x, center_y), 5, (0, 0, 255), -1)  # Red dot

        # Determine position based on grid
        if center_x < width // 3:
            position = "Left"
        elif center_x > 2 * width // 3:
            position = "Right"
        else:
            position = "Center"

        if center_y < height // 3:
            position += " Top"
        elif center_y > 2 * height // 3:
            position += " Bottom"
        else:
            position += " Middle"

        # Display the position of the green object on the frame
        cv2.putText(
            green_detection,
            f"Position: {position}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),  # Green text
            2
        )

    # Display the green detection frame
    cv2.imshow("Green Detection", green_detection)

    # Display the original frame (optional)
    cv2.imshow("Original Frame", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Releasing the video capture object and closing all windows
vid.release()
cv2.destroyAllWindows()
