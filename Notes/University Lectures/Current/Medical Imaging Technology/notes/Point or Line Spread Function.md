- By understanding the PSF and LSF of an imaging system, we can predict how the system will respond to any “ideal image” and assess its resolution capabilities. 
- These functions are crucial for designing and evaluating optical systems, such as cameras and microscopes, and are widely used in fields like astronomy, medical imaging, and electron microscopy

## Point Spread Function (PSF)
- The PSF describes how an imaging system responds to a point source or point object.
- Mathematically, if $O(x,y,z)$ is the object function representing a single point source, and $I(x,y,z)$ is the resulting image, then the PSF $h(x,y,z)$ relates the two by the convolution operation:$$I(x,y,z)=h(x,y,z)∗O(x,y,z)$$
- The PSF is essentially the image of a point object as produced by the system. It characterizes the blurring and spreading of the point source caused by the system’s imperfections

## Line Spread Function (LSF)
- The LSF is a simplification of the PSF to one dimension.
- It measures how an imaging system responds to a line source.
- The LSF is derived by integrating the PSF across two dimensions, leaving a function of one variable.
- Mathematically, if ( $L(x)$ ) is the LSF, it can be expressed as:$$L(x)=∫∫h(x,y,z)dydz$$
- The LSF is useful for analyzing the system’s resolution along a particular axis and is easier to measure and interpret than the full 3D PSF