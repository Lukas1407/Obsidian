- [[Fundamentals of Medical Imaging.pdf]] pages 39-43
> [!abstract] Definition
> An iterative reconstruction starts with an assumption (for example, that all points in the matrix have the same value) and compares this assumption with measured values, makes corrections to bring the two into agreement, and then repeats this process over and over until the assumed and measured values are the same or within acceptable limits 


## Process 
1. Calculate the [[Analytical Methods for Image Reconstruction in Tomography#Analytical Description (Radon Transform)|Radon Transform]] $p_\theta$ from the image $f(x,y)$
2. Initialize the image we want to reconstruct $f^{*}(x,y)$
	- For example average, mean, median, etc. over the pixels from $p_{\theta}$
3. Compute $p^{*}_{\theta}$ from $f^{*}(x,y)$
4. Compare $p^{*}_{\theta}$ to $p_\theta$ and estimate the error between $f^{*}(x,y)$ and $f(x,y)$
### Example
![[annotated_Iterative Algorithms for Image Reconstruction in Tomography 2024-05-13 09.21.07.excalidraw|800]]
### Problems
- In the early years of CT the use iterative techniques were not feasible because of the following limitations: 
	1. It is difficult to obtain accurate ray sums because of quantum noise and patient motion. 
	2. Computationally expensive 
- Today, iterative reconstruction algorithms have resurfaced because of the availability of high-speed computing 
- All major CT manufacturers offer iterative reconstruction algorithms as of 2014. For example, while GE Healthcare and Philips Healthcare offer Adaptive Statistical Iterative Reconstruction (ASiR),
### Advantages 
- Reduce image noise 
- More accurate with limited data than back-projection
- minimize the higher radiation dose inherent in the filtered back-projection algorithm. 