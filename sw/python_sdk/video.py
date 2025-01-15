import cv2
from mlx90640 import MLX90640
import numpy as np
import time

temperature = None
frame_re = None  # Initialize frame_re to avoid reference before assignment


def mouse_callback(event, x, y, flags, param):
    global temperature, frame_re
    if event == cv2.EVENT_MOUSEMOVE:
        # Ensure coordinates are within the frame bounds
        if (
            frame_re is not None
            and 0 <= x < frame_re.shape[1]
            and 0 <= y < frame_re.shape[0]
        ):
            temperature = frame_re[y, x]
        else:
            temperature = None


def create_color_bar(width, height, min_temp, max_temp):
    """
    Create a vertical color bar with specified width and height.
    Only shows min and max temperature labels.
    """
    color_bar = np.zeros((height, width, 3), dtype=np.uint8)
    for i in range(height):
        # Calculate the corresponding temperature
        temp = max_temp - (max_temp - min_temp) * (i / height)
        # Normalize the temperature to [0, 255] for the color map
        normalized_temp = int((temp - min_temp) / (max_temp - min_temp) * 255)
        color = cv2.applyColorMap(
            np.array([[normalized_temp]], dtype=np.uint8), cv2.COLORMAP_JET
        )
        color_bar[i, :, :] = color

    # Add the minimum temperature label at the bottom
    cv2.putText(
        color_bar,
        f"{min_temp}C",
        (5, height - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
    )
    # Add the maximum temperature label at the top
    cv2.putText(
        color_bar,
        f"{max_temp}C",
        (5, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
    )

    return color_bar


def main():
    global frame_re

    # Initialize MLX90640 sensor
    mlx = MLX90640()
    frame = mlx.get_frame()

    window_name = "Cobra Scope"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Using 'mp4v' codec
    output_width = 640 + 50  # Main frame width + color bar width
    output_height = 480
    desired_fps = 2  # Adjust FPS as needed
    out = cv2.VideoWriter(
        "output.mp4", fourcc, desired_fps, (output_width, output_height)
    )

    if not out.isOpened():
        print("Error: VideoWriter failed to open.")
        return

    # Create the color bar once since it's fixed
    color_bar = create_color_bar(width=50, height=480, min_temp=20, max_temp=300)

    frame_count = 0

    while True:
        # Capture frame from sensor
        try:
            mlx.get_frame(frame_buffer=frame)
        except Exception as e:
            print(f"Error capturing frame: {e}")
            break

        frame_re = np.reshape(frame, (24, 32))
        frame_re = cv2.resize(frame_re, (640, 480), interpolation=cv2.INTER_LINEAR)

        # Normalize and apply color map
        frame_normalized = np.clip(frame_re, 20, 300)
        frame_8bit = ((frame_normalized - 20) * (255 / (300 - 20))).astype(np.uint8)
        frame_colored = cv2.applyColorMap(frame_8bit, cv2.COLORMAP_JET)

        # Find the minimum and maximum temperature points
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(frame_re)

        # Map the min and max locations to the resized frame
        scale_x = frame_colored.shape[1] / frame_re.shape[1]
        scale_y = frame_colored.shape[0] / frame_re.shape[0]
        min_loc_scaled = (int(min_loc[0] * scale_x), int(min_loc[1] * scale_y))
        max_loc_scaled = (int(max_loc[0] * scale_x), int(max_loc[1] * scale_y))

        # Draw circles on the min and max temperature points
        cv2.circle(
            frame_colored, min_loc_scaled, 5, (255, 0, 0), 2
        )  # Blue circle for min
        cv2.circle(
            frame_colored, max_loc_scaled, 5, (0, 0, 255), 2
        )  # Red circle for max

        # Display the temperature values
        cv2.putText(
            frame_colored,
            f"Max: {max_val:.1f}C",
            (max_loc_scaled[0] + 10, max_loc_scaled[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        cv2.putText(
            frame_colored,
            f"Min: {min_val:.1f}C",
            (min_loc_scaled[0] + 10, min_loc_scaled[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
        )

        # Draw the temperature value at the cursor position
        if temperature is not None:
            cv2.putText(
                frame_colored,
                f"Temperature: {temperature:.1f}C",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

        # Combine the color bar with the frame
        frame_combined = np.hstack((frame_colored, color_bar))

        # Display the frame
        cv2.imshow(window_name, frame_combined)

        # Write the frame to the video file
        out.write(frame_combined)
        frame_count += 1
        # print(f"Frame {frame_count} written", time.time())

        # Break the loop if 'q' is pressed or the window is closed
        key = cv2.waitKey(1) & 0xFF
        if (
            key == ord("q")
            or cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
        ):
            break

    # Release everything if job is finished
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
