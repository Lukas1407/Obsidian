![[Pasted image 20240508085452.png#invert|400]]
1. <mark style="background: #FFB86CA6;">Interaction with Energy Source:</mark>
	- An [[Medical Imaging Technology#Classification#Energy source|energy source]] (like X-rays, ultrasound waves, or magnetic fields) interacts with the object (the patient’s body).
		- Reflection
		- Absorption
		- Diffusion
	- During this interaction, a characteristic quantity emerges, such as:
		- <mark style="background: #FFB86CA6;">Attenuation</mark>: Reduction in energy intensity after passing through tissues. ^864773
		- <mark style="background: #FFB86CA6;">Reflectance</mark>: Energy reflected back from tissues.
		- <mark style="background: #FFB86CA6;">Source Activity</mark>: Radioactive decay in [[Nuclear Imaging]].
2. <mark style="background: #FFB86CA6;">Spatial Distribution:</mark>
	- The physical quantity that emerges is distributed in space and time, represented by the function $g(x,y,z,t)$
	- This function describes how the quantity varies at different points within the body and over time.

- <mark style="background: #FFB86CA6;">Static Grayscale Images:</mark>
	- For static images without color, the intensity of each point is represented by a gray level $I$
	- The gray level at a specific point $(x,y,z)$ is given by the function $I=g(x,y,z)$
	- This creates a 3D map of gray levels corresponding to different tissues’ characteristics.
- <mark style="background: #FFB86CA6;">Static RGB Color Images:</mark>
	- For color images, $I$ becomes a vector with three components representing red, green, and blue intensities.
- <mark style="background: #FFB86CA6;">Dynamic Images:</mark>
	- For images that change over time (dynamic images), the time variable $t$ is added.
	- The function becomes $g(x,y,z,t)$ or $g(x,y,t)$ for 3D or 2D dynamic imaging, respectively.
	- This allows for the visualization of changes in the physical quantity over time, such as blood flow or heart motion.
## For medical applications
- Read [[Fundamentals of Medical Imaging.pdf]] pages 36-39!!!