
## Forward Projection
- Forward projection in CT scanning is a fundamental concept that involves mapping the 3D structure of the patient’s body onto a 2D image through a series of mathematical transformations.
### Intuitive Approach
- We only assume one slice
- Imagine a simple object, or phantom, with regions of different densities placed within a nearly transparent background.
- These densities are measured on the Hounsfield scale, which is normalized to the density of water. On this scale, water has a value of 0, denser materials have positive values (bones), and less dense materials have negative values (air).
![[Pasted image 20240512083230.png#center|400]]
- For one acquisition angle, multiple X-ray pencil beams (indicated by red arrows) pass through the phantom.
- As these beams penetrate the phantom, they are attenuated based on the density of the tissues they pass through and the distance they travel within those tissues.
- <mark style="background: #FFB86CA6;">The detector measures the intensity profile</mark> $P(r)$, which is <mark style="background: #FFB86CA6;">inversely related to attenuation</mark>. A higher value on the intensity profile indicates stronger attenuation by the tissue.
- This <mark style="background: #FFB86CA6;">process is repeated from many different angles</mark> around the phantom.
![[Pasted image 20240512091028.png|300]]
- The <mark style="background: #FFB86CA6;">intensity profiles from all these angles are then compiled into a single diagram known as a</mark> [[Analytical Methods for Image Reconstruction in Tomography#Why Sinogram?|sinogram]].
![[test_gic-ezgif.com-crop.gif|400]]
- In the sinogram, lighter shades represent higher values in the intensity profile, indicating greater attenuation at a specific translation $r$ for the angle $\theta$
	- $r$ is the <mark style="background: #FFB86CA6;">distance from the detector</mark> to the origin
	- $\theta$ is the <mark style="background: #FFB86CA6;">projection angle</mark>
- Image space is defined in Cartesian coordinates $(x,y)$, while sinogram space is defined by the variables $\theta$ (the angle of X-ray beam acquisition) and $r$ (the detector’s position).
![[Pasted image 20240512083450.png#invert|400]]
- Mathematically, this transformation can be represented as a function mapping from one space to another
### Analytical Description (Radon Transform)
- The goal is to find a suitable transformation function $\mathcal{R}$: $$f(x,y)\rightarrow\mathcal{R}f(r,\theta)$$
- In medical applications, $f$ is the e attenuation coefficient function $\mu(x,y)$
- A new coordinate system $(r,s)$ is defined by rotating $(x, y)$ over the angle $\theta$
	- This rotation aligns the $r$-axis with the direction of the X-ray beam and the $s$-axis perpendicular to it.
![[Pasted image 20240512091749.png#invert|400]]
- This can simply be done using the rotation matrix:$$
\begin{bmatrix}r \\ s \end{bmatrix}
=
\begin{bmatrix} \cos(\theta) & \sin(\theta) \\ -\sin(\theta) & \cos(\theta) \end{bmatrix}
\begin{bmatrix} x \\ y \end{bmatrix}
$$
- $L_{r,\theta}$ is the source-to-detector line with distant $r$ and angle $\theta$
- As the beam penetrates the object, it is attenuated based on the tissues it encounters.
- The intensity measured by the detector, after attenuation, $I_{\theta}$ is as a function of $r$ is given by:$$\begin{align}I_{\theta}(r)=I_{0}*e^{-\int_{L_{r,\theta}}\mu(x,y)ds}\\ = I_{0}*e^{-\int_{L_{r,\theta}}\mu(r*\cos(\theta)-s*\sin(\theta),r*\sin(\theta)+s*\cos(\theta))ds} \end{align}$$
	- The attenuation profile is calculated by integrating the attenuation coefficient $\mu(x,y)$ along the path of the X-ray beam, as there are different materials encountered on the way
- Each intensity profile can be transformed into an attenuation profile:$$\begin{align} p_\theta(r)=-\ln I_{\theta}\frac{r}{I_{0}}\\ = \int_{L_{r,\theta}}\mu(r*\cos(\theta)-s*\sin(\theta),r*\sin(\theta)+s*\cos(\theta))ds \end{align}$$where $p_{\theta}(r)$is the projection of the function $\mu(x,y)$along the angle $\theta$
> [!tldr] Definition
>  For a function $f(x,y)$ defined in $ℝ^{2}$ with compact support the Radon transform $\mathcal{R}$ of $f(x,y)$ in the space $r \in ℝ$ and $\theta \in (0, 2\pi]$
>  $$\begin{align} p_{\theta}(r)=\mathcal{R}F(x,y)\\=\int_{-\infty}^{\infty}f(r*\cos(\theta)-s*\sin(\theta),r*\sin(\theta)+s*\cos(\theta))ds ) \end{align}$$

- The Radon transform offers a way of determining the total density of a certain function $f$ along a given line $L$. This line $L$ is determined by an angle $\theta$ from the x-axis and a distance $r$ from the origin
#### Example
![[radon_exampl.excalidraw]]
## Why Sinogram?
- <mark style="background: #FFB86CA6;">A sinogram is a representation of data obtained through the Radon transform from a two-dimensional slice of an object</mark>.
- It contains one-dimensional projection data $p(\theta, r)$ acquired from multiple different angles $\theta$
- Ea<mark style="background: #FFB86CA6;">ch point in the sinogram corresponds to the X-ray attenuation</mark> at a specific angle $\theta$ and radial distance $r$
### Discrete Sinogram
- In practical CT scans, we cannot continuously collect all possible projection data due to physical limitations of detectors and data acquisition.
	- Integrating from $-\infty$ to $\infty$ is not possible in practice
- Instead, data is collected at discrete intervals of angles and radial distances.
- The discrete sinogram $p(N\Delta r, M\Delta \theta)$ is a digitized version of the continuous sinogram.
- For a finite number of $M$ projections or views and a finite number of $N$ detector samples, $Δr$ is the detector sampling distance and $Δr$ is the rotation interval between subsequent views, the  discrete sinogram $p(NΔr,MΔθ)$ can be represented as a matrix with $M$ rows and $N$ columns.
#### Importance of $\Delta s$ and $\Delta r$
- The physical characteristics of the X-ray beam and detector array significantly impact image quality and resolution.
- The relationship between the finite X-ray beam width $\Delta s$ and the detector sampling distance $\Delta r$ is critical for accurate image reconstruction.
- The beam width $\Delta s$ affects resolution and the level of detail discernible in the resulting image.
- Properly balancing $\Delta s$ and $\Delta r$ helps avoid aliasing artifacts and ensures accurate image reconstruction.
- In order to avoid aliasing, the relationship between Δs and Δr can be mathematically expressed as:$$\frac{1}{\Delta r}> \frac{2}{\Delta s}\rightarrow\Delta r < \frac{\Delta s}{2}$$
## Back Projection
- Given a sinogram $p(r,\theta)$, how can we reconstruct the distribution $μ(x, y)$, or more generally the function $f(x, y)$?
### Overview
- Each projection $p(\theta, r)$ from the sinogram is spread back across the image plane at the angle $\theta$ it was taken.
- The value at each point $(x,y)$ in the reconstructed image is the sum of all projection values that intersect at that point.
### Mathematical Expression
- The back-projected value at point ( (x, y) ), denoted as ( f_b(x, y) ), is calculated by summing the contributions from all angles:$$f_b(x, y) = \sum_{m=1}^{M} p(x \cos \theta_m + y \sin \theta_m, \theta_m) \Delta\theta x$$Here,:
	- $M$ is the number of projections
	- $\Delta \theta$ is the angular increment between projections
	- $p(x \cos \theta_m + y \sin \theta_m, \theta_m)$ is the projection data at angle $\theta_m$ (in the discrete case)
### Process
- The process involves “smearing” the projection data back onto the image grid at the same angles they were acquired.
- This “smearing” integrates the data from all angles to build up an image that approximates the original cross-section of the scanned object.
### Example
![[Back-projection-examle.excalidraw|900]]
-> More projections lead to a better result
![[Pasted image 20240512120958.jpg|500]]
- Back Projection (smearing it on the x,y plane with the same $\theta$ they were acquired with. Then they are summed up together
![[Pasted image 20240512121002.png|500]]
### Star Artifacts
- By simply back-project each line of the sinogram  we smear bright pixels across the entire image instead of putting them exactly where they belonged. 
![[Pasted image 20240512121134.jpg|400]]
- <mark style="background: #FFB86CA6;">We obtain so called “star artifacts” around bright pixel regions</mark>
- The reconstruction can be <mark style="background: #FFB86CA6;">improved by increasing the number of projections</mark> (and relative back-projections) 
- <mark style="background: #FFB86CA6;">However</mark>, the smearing artifact will be still present around the edge of bright elements in the image -> <mark style="background: #FFB86CA6;">blurred image</mark>
### Problems
#### [[Point or Line Spread Function#Point Spread Function (PSF)|PSF]]
- The output is not perfectly reconstructed ([[Analytical Methods for Image Reconstruction in Tomography#Star Artifacts|Star Artifacts]])
- When an impulse response is computed on a back-projected image, it results in a [[Point or Line Spread Function#Point Spread Function (PSF)|Point Spread Function (PSF)]] that is spread out in the Cartesian plane.
- This [[Point or Line Spread Function#Point Spread Function (PSF)|PSF]] is akin to a “blurring” effect, where the reconstructed image appears smeared because each point in the original object spreads its influence over a wide area in the image.
- The original image is essentially convolved with a function that acts as a low-pass filter, reducing the sharpness and contrast of the image.
#### Discrete Nature of Measurements
- In practice, CT scans are discrete, meaning that data is collected at specific intervals rather than continuously.
- For each view, a projection line is drawn through each pixel, and the intersection of this line with the detector array is computed.
	- Easy/Exactly computable for 0, 45, 90, etc degrees, but not for in between
![[Pasted image 20240512122700.jpg#invert|150]]
- The corresponding projection value is then calculated by interpolation, typically using linear or nearest neighbor methods.
- This interpolation can introduce errors, especially if the sampling rate is not high enough to capture the true variation in X-ray attenuation across the object (aliasing).
## Central Slice Theorem
- We have seen, that we cannot invert from the [[Analytical Methods for Image Reconstruction in Tomography#Analytical Description (Radon Transform)|Radon Transform]] back to the original image because of the mentioned [[Analytical Methods for Image Reconstruction in Tomography#Problems|problems]]
- Idea: Radon transform the image, calculate the [[Fourier Transform]], and calculate the inverse [[Fourier Transform]]
![[University Lectures/Future/Medical Imaging Technology/notes/images/Diagram.svg#invert|300]]
> [!abstract] Definition: Central Slice Theorem
>  The one dimensional Fourier transform of a projected function (the Radon transform) is equal to the 2D-Fourier transform of the original function taken on the slice through the origin parallel to the line we projected our function on.

### Concept
1. Calculate the [[Analytical Methods for Image Reconstruction in Tomography#Analytical Description (Radon Transform)|Radon Transform]] for an angle $\theta$
2. Calculate the 1D [[Fourier Transform]] of that signal 
3. The 1D [[Fourier Transform]] corresponds to a single line in Fourier space at angle $\theta$
	- This is now frequency space! -> $k_x$ and $k_{y}$ represent the spatial frequencies in the $x$ and $y$ directions, respectively.
4. Do that for various $\theta$
5. Calculate the inverse [[Fourier Transform]] to obtain the original image back 
![[annotated_annotated_Pasted image 20240513082401|700]]
- The 1D [[Fourier Transform]] ($P_\theta(k)$) of the [[Analytical Methods for Image Reconstruction in Tomography#Analytical Description (Radon Transform)|Radon Transform]] ($p_\theta(r)$) is given by:$$P_\theta(k)=\int_{\infty}^{-\infty}p_{\theta}(r)e^{-2\pi i(k\cdot r)}dr$$
- To connect the 1D and 2D [[Fourier Transform]], we use the following relationships:
	- Let $F(k_x​,k_y​)$ denote the 2D [[Fourier Transform]] of the original object $f(x,y)$.
	- For a given angle $\theta$, the 1D [[Fourier Transform]] $P_{\theta}​(k)$ corresponds to a line in the 2D [[Fourier Transform]] space:$$P_\theta(k)=F(k_{x},k_{y})\text{ for }
  \begin{cases}
      k_{x}=k\cdot \cos(\theta) \\
      k_{y}=k\cdot \sin(\theta) \\
	  k=\sqrt{k_{x}^{2}+k_{y}^{2}} \\
	    \end{cases}  $$
- The 2D [[Fourier Transform]] of a function $f(x,y)$ is given by:$$F(k_{x},k_{y})=\int^{\infty}\int_{-\infty}f(x,y)e^{-2\pi i(k_{x}x+k_{y}y)}dxdy$$
### Problem of Direct Fourier Reconstruction
- When we acquire projections (such as X-ray projections in computed tomography), they provide information about the object’s attenuation along specific lines (rays) passing through it.
- In the Fourier domain, these projections correspond to <mark style="background: #FF5582A6;">radial frequency sampling</mark>.
- Each projection contributes to a specific radial line in the 2D Fourier Transform space.
- The challenge lies in the fact that we don’t have uniform sampling of spatial frequencies across the entire 2D Fourier domain.
	- Low frequencies (in the center of the 2D Fourier Space) occur often -> we have good approximation
	- High frequencies (in the corners of the 2D Fourier Space) occur only once per $\theta$ -> we poor good approximation
	- approximation will be less and less precise as a function of k -> we have high density of low frequencies and low density of high frequencies
- The 2D [[Fourier Transform]] is typically computed on a polar grid (due to the radial sampling from projections).
- To perform the inverse transform, we need to interpolate the 2D Fourier Transform onto a Cartesian grid.
- This interpolation process introduces some loss of precision.
- To address this challenge, we need to find a way to compensate for the drop in high-frequency sampling.
- At the same time, we want to reduce the contribution of low-frequency content (which dominates due to the central slice theorem).
- Strategies for compensation is [[Analytical Methods for Image Reconstruction in Tomography#Filtered Back-Projection|Filtered Back-Projection]]
### Filtered Back-Projection
- Calculate the Central Slice Theorem in [[polar Coordinates]]:$$f(x,y)=\int_{0}^{\pi}\int_{-\infty}^{\infty}P_\theta(k)|k|e^{i2\pi kr}dkd \theta$$
- -> The Fourier Transform of the Radon transform of a certain θ is multiplied by $|k|$ which, in the frequency domain represents a 1D ramp filter
	- This filter filters out the low frequencies and focuses more on high frequencies
#### Types of Filters
![[Pasted image 20240513085731.png#invert|400]]
- There is a wide range of filters to improve performance. 
- This can be achieved by cutting high frequencies beyond a maximum value (Ram-Lak filter), as they are likely noise. 
- Other filters aim to smooth high frequencies in order to suppress noise and prevent aliasing.
- These filters don't have their minimum at $(0,0)$ as that would completely filter out the low frequency
#### Process
- For all $\theta\in(0,\pi)$
	1. Calculate [[Analytical Methods for Image Reconstruction in Tomography#Analytical Description (Radon Transform)|Radon Transform]] $p_\theta$
	2. Calculate the [[Fourier Transform]] $P_{\theta}$ of $p_\theta$
	3. Apply a ramp filter to $P_{\theta}$
	4. Calculate the inverse [[Fourier Transform]] of the filtered $P_{\theta}$ thus obtaining a filtered projection $p^{*}_{\theta}$
![[Pasted image 20240513090331.png|400]]
#### Improved Process
- Because of the [[Convolution Theorem]], we can also convolve $p_\theta$ with the [[Fourier Transform]] of the ramp filter to achieve the same result 
![[annotated_Pasted image 20240513090634|600]]
