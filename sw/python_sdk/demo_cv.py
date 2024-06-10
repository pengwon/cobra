import cv2
from mlx90640 import MLX90640
import numpy as np

temperature = None

def mouse_callback(event, x, y, flags, param):
    global temperature
    if event == cv2.EVENT_MOUSEMOVE:
        # Get the temperature value at the cursor position
        temperature = frame_re[y, x]

mlx = MLX90640()
frame = mlx.get_frame()

window_name = 'Cobra Scope'
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name, mouse_callback)

while True:
    mlx.get_frame(frame_buffer=frame)
    frame_re = np.reshape(frame, (24, 32))
    frame_re = cv2.resize(frame_re, (640, 480), interpolation=cv2.INTER_LINEAR)  # Resize the frame to 640x480

    # Normalize the frame to [0, 255] and convert it to 8-bit unsigned integer
    frame_reshaped = cv2.normalize(frame_re, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Draw the temperature value at the cursor position
    if temperature is not None:
        cv2.putText(frame_reshaped, f'Temperature: {temperature:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    frame_reshaped = cv2.applyColorMap(frame_reshaped, cv2.COLORMAP_JET)  # Apply a color map

    cv2.imshow(window_name, frame_reshaped)  # Display the frame

    # Break the loop if 'q' is pressed or the window is closed
    if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
        break

cv2.destroyAllWindows()