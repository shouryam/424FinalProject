import numpy as np
import cv2 as cv

camera = cv.VideoCapture(2)

# Checks that the camera has been opened
if not camera.isOpened():
    print("Unable to open camera\n\t1. Camera is not connected\n\t2. Camera port is different than specified one")
    exit()

line_color = (0, 0, 255)
line_width = 5

while True:
    cv.waitKey(1)
    # Capture frame-by-frame
    ret, frame = camera.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Get frame dimensions
    height, width, _ = frame.shape

    # Get line image from frame
    line_image = np.zeros_like(frame)

    # Draw lines on the frame
    cv.line(line_image, (0, height//2), (width//2, height//2), line_color, line_width)
    cv.line(line_image, (width//2, 0), (width//2, height//2), line_color, line_width)

    # Add weighted coloration
    line_image = cv.addWeighted(frame, 0.8, line_image, 1, 1)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    height, width, _ = hsv.shape

    print(hsv[height//2, width//2])

    cv.imshow("frame", line_image)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
camera.release()
cv.destroyAllWindows()
