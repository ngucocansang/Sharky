import cv2
import numpy as np
import serial

# Initialize PySerial for communication
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)

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

    # Find contours of the green object
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Calculate the center of the green object
        center_x = x + w // 2
        center_y = y + h // 2

        # Determine robot movement
        if center_x < width // 3:
            command = "LEFT\n"
        elif center_x > 2 * width // 3:
            command = "RIGHT\n"
        else:
            command = "FORWARD\n"

        # Send the command to Arduino
        arduino.write(command.encode())
    else:
        # Stop the robot if no object is detected
        arduino.write("STOP\n".encode())

    # Display the frames
    cv2.imshow("Original Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
vid.release()
cv2.destroyAllWindows()
arduino.close()
