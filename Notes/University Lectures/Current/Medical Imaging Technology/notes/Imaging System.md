> [!abstract] Definition
> The input to an Imaging System is the object to be imaged which interacts with the [[Medical Imaging Technology#Classification#Energy source|energy source]]. The output is the [[Image Signal|signal]] in $g(x,y,z,t)$ representation.


- Depending on the image modality the image blur can be due to 
	- focal spot/collimation 
	- detector 
	- image reconstruction 
	- filtering 
	- …
- $\rightarrow$ The response of the system is $$h_{\text{response}}(x,y,z,t)=h_{physics}(x,y,z,t)+h_{\text{detector}}(x,y,z,t)+h_{\text{sampling}}(x,y,z,t)+h_{\text{filter}}(x,y,z,t)+...$$
## A Imaging System is a [[Linear Time-Invariant (LTI) system|LTI System]]
### Linear System
- An imaging system <mark style="background: #FFB86CA6;">can often be approximated as a linear system</mark> within certain limits.
- Linearity implies that the system’s response to a scaled input is also scaled by the same factor.
	- If we double the intensity of an X-ray beam (input), the resulting image’s brightness (output) will also double.
	- Similarly, if we halve the intensity, the brightness will halve.
- This linearity assumption <mark style="background: #FFB86CA6;">holds well when they operate within a linear calibration range</mark>.
- However, <mark style="background: #FF5582A6;">outside this range, non-linear effects (such as saturation or clipping) may occur</mark>.
### Time-Invariant System
- An imaging system can also be characterized as a time-invariant system.
- <mark style="background: #FFB86CA6;">The physical properties of system components</mark> (e.g., lenses, sensors) themselves <mark style="background: #FFB86CA6;">do not change significantly during typical imaging operations</mark>.
- As a result, the system’s response to an input signal remains the same regardless of when it is applied.
### Practical Implications
- These property <mark style="background: #FFB86CA6;">simplifies analysis and modeling</mark>.
- Allow the use of concepts like convolution, Fourier transforms, and transfer functions.
