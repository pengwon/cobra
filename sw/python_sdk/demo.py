from mlx90640 import MLX90640
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

mlx = MLX90640()
frame = mlx.get_frame()
# for line in range(24):
#     print(" ".join([f"{value:.1f}" for value in frame[line * 32 : (line + 1) * 32]]))


# Create a figure for the plot
fig, ax = plt.subplots()

# Initialize a blank frame
img = ax.imshow(np.zeros((24, 32)), cmap="hot", interpolation="nearest")

# Update function for animation
def update(i):
    mlx.get_frame(frame_buffer=frame)
    frame_reshaped = np.reshape(frame, (24, 32))
    img.set_array(frame_reshaped)
    img.set_clim(10, 50)  # Set the color limits
    return img,

# Create an animation
ani = FuncAnimation(fig, update, frames=range(100), blit=True)

# Add a colorbar
plt.colorbar(img)

plt.show()


# # Assuming frame is a flat list with 768 elements (24x32)
# frame_reshaped = np.reshape(frame, (24, 32))

# # Generate a heatmap
# img = plt.imshow(frame_reshaped, cmap="hot", interpolation="nearest")

# # Add a colorbar
# plt.colorbar(img)

# plt.show()
