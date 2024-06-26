'''
Script for visualising how the radon transform works. The user is asked to draw an object. The radon transform is then
applied to this 2D object. The reconstructed image is also shown.
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D
from skimage.transform import radon, iradon

class ShapeDrawer:
    '''
    Gives the user a canvas to draw on and records the coordinates of the mouse when the left mouse button is pressed.
    These lines are used to create a 2D object (representing body tissues).
    '''
    def __init__(self, shape_size):
        self.shape_size = shape_size
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, shape_size)
        self.ax.set_ylim(0, shape_size)
        self.ax.set_aspect('equal', 'box')
        self.lines = []
        self.current_line = None
        self.is_drawing = False
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        '''
        Records a mouse click.
        '''
        if event.inaxes != self.ax:
            return
        self.is_drawing = True
        self.current_line = [(event.xdata, event.ydata)]
        self.lines.append(self.current_line)

    def on_motion(self, event):
        '''
        Saves the coordinates of the mouse when the left mouse button is pressed.
        '''
        if event.inaxes != self.ax:
            return
        if self.is_drawing:
            self.current_line.append((event.xdata, event.ydata))
            self.update_plot()

    def on_release(self, event):
        '''
        Records when the mouse button is released.
        '''
        if event.inaxes != self.ax:
            return
        self.is_drawing = False
        self.current_line = None

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(0, self.shape_size)
        self.ax.set_ylim(0, self.shape_size)
        self.ax.set_aspect('equal', 'box')
        for line in self.lines:
            self.ax.add_line(Line2D([p[0] for p in line], [p[1] for p in line], color='black'))
        self.fig.canvas.draw()

    def get_shape_array(self):
        '''
        Saves the coordinates of the drawn line in an ndarray.
        '''
        shape_array = np.zeros((self.shape_size, self.shape_size), dtype=int)
        for line in self.lines:
            line = np.array(line, dtype=int)
            line = np.clip(line, 0, self.shape_size-1)
            shape_array[line[:, 1], line[:, 0]] = 1
        shape_array = np.flip(shape_array, axis=0)
        return shape_array

    def show_shape(self):
        plt.show()


shape_size = 100  # Defines how big the canvas to be drawn on should be.
shape_drawer = ShapeDrawer(shape_size)  # Calls the shape drawing functionality.
shape_drawer.show_shape()

# After drawing, you can get the shape array by calling:
object = shape_drawer.get_shape_array()

# object_size = 100
# object = np.random.rand(object_size, object_size)
# object = np.zeros((object_size, object_size), dtype=int)
# object[25:50, 45:60] = 1q
# print(object)

# Test with the Shepp-Logan phantom:
# from skimage.data import shepp_logan_phantom
# from skimage.transform import rescale

# image = shepp_logan_phantom()
# object = rescale(image, scale=0.25, mode='reflect', channel_axis=None)

# Perform the Radon transform
theta = np.linspace(0., 180., max(object.shape), endpoint=False)
sinogram = radon(object, theta=theta, circle=True)

# Reconstruct the image using the inverse Radon transform (backprojection)
reconstructed_image = iradon(sinogram, theta=theta, circle=True)

'''
Display the original object, sinogram, and reconstructed image
'''
plt.figure(figsize=(14, 6))
plt.subplot(131)
plt.title('Original Object')
plt.imshow(object, cmap='gray')

plt.subplot(132)
plt.title('Sinogram (Radon Transform)')
plt.xlabel('Projection angle (deg)')
plt.ylabel('Projection position (pixels)')
plt.imshow(sinogram, cmap='gray', aspect='auto', extent=(0, 180, sinogram.shape[0], 0))

plt.subplot(133)
plt.title('Reconstructed Image')
plt.imshow(reconstructed_image, cmap='gray')

plt.show()

