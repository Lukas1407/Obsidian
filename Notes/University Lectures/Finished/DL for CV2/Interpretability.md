Goals of Interpretability are to:
1. Understand how the model came to its predictions
2. Find out what the model or parts of the model have learned

# Attribution Methods
Solutions to Interpretability can be categorized into:
1. Pixel-level attribution: What pixels were most important for the predictions of the model
2. Lower layer attribution: What layers were most active for the prediction
3. Understanding single Neurons

## Pixel-level Attribution
### Perturbation
- Mask out parts of the image via a sliding-window
- Pass the image trough the model and calculate the predictions
- The region where the accuracy drops the most is the most important for the model
- Improvements:
	- Use multiple sliding-windows to get more detailed, non-rectangular regions of importance
	- Gaussian blur the masked out region instead of removing it/ setting it to black. This is a more realistic setting for the model
- Advantages:
	- Simple
	- Model agnostic
- Disadvantages:
	- Sensitive to the size and stride of the sliding-window
	- Only captures broad regions of interest
### Gradient
- Also known as the <mark style="background: #FFB86CA6;">saliency map</mark> of the neural network
The saliency map <mark style="background: #FFB86CA6;">shows which pixels have the most influence on the output</mark> of the network. By taking the derivative of the predicted class with respect to the pixel, we can measure how much the output changes when the pixel value changes slightly. T<mark style="background: #FFB86CA6;">his is a way of estimating the sensitivity of the network to each pixel</mark>.
- Calculate the gradient of predicted class w.r.t. to each pixel :
$$R_{ij}^{c}(x) =  \frac{\delta f^c}{\delta x_{ij}}(x)$$
- <mark style="background: #FF5582A6;">Problem</mark>:
	- <mark style="background: #FF5582A6;">Noisy</mark> because the derivative is not a smooth function of the pixel value. Small changes in the pixel value can cause large changes in the derivative, especially near the edges or corners of the image. This can result in saliency maps that are not very informative or interpretable.
	- <mark style="background: #FF5582A6;">Vanishing gradient</mark>, then the saliency map will not reflect their true importance for the prediction.
- Solution:
	1. Gradients Times Input:
		- Instead calculate:	$$R_{ij}^{c}(x) =  x_{ij} \times \frac{\delta f^c}{\delta x_{ij}}(x)$$
		- This way, the pixel also must be important for the important measure to be high
		- Mitigate "gradient saturation" and [[Visual Diffusion|visual diffusion]]
	2. Integrated Gradient
		- Interpolate between a baseline image $x'$, which my be completely black or random nose, and the original image $x$:		$$R_{ij}^{c}(x) = (x_{ij} - x'_{ij}) \times \int_{0}^{1}\frac{\delta f^{c}(x' - t(x - x'))}{x_{ij}}dt$$
		- ![[Pasted image 20240223151956.png]]
		- Mitigates "gradient saturation" and [[Visual Diffusion|visual diffusion]]
	3. Smoothed Gradient
		- Average over multiple noisy versions of the same image:	$$R_{ij}^{c}(x) = \mathop{\mathbb{E}}[R_{ij}^{c}(x+s)],$$where $s$ ~ $N(0,\sigma^2I_{n})$  
		- Helps with noise and [[Visual Diffusion|visual diffusion]]
- Advantages:
	- Model agnostic
	- Follows the structure of the model
	- Fast
- Disadvantages:
	- Noisy
	- Focuses more on details than on concepts

## Lower Layer Attribution
### Grad-CAM
- Instead of computing attributions on the pixel-level where they are noisy, compute them on higher layer feature maps: $$\frac{\delta f^{c}}{\delta A_{ij}^{k}}$$
- Obtain the feature importance $a_{k}^{c}$ for each feature map $A^{k}$ by averaging the gradients: $$a_{k}^{c} = \frac{1}{H*W}*\sum_{i=1}^{H}\sum_{j=1}^{W}\frac{\delta f^{c}}{\delta A_{ij}^{k}}(x)$$
- Weigh the feature map at each position with its weight, followed by ReLU to only get the parts which have positive influence to the importance:$$ReLU(\sum_{k=1}^{K}a_{k}^{c}*A_{ij}^{k})$$
## Importance of Units
### Unit Dissection
- Calculate the top 1% of activation values values of a unit $u$:$$t_{u}= \max_{t}(P(u_{ij}(x)>t))>0.01$$
- Segment the image using $t_{u}$ and masks from a segmentation network
- Compute the IOU per concept for all images and assign the class with the highest score to the unit 
- This gives us the concepts each unit is most sensitive to
- Blocking the units most sensitive to a concept, which is most important for a class, makes it non-predictable
- In the case of GANs, it makes the concept non-generatable

## Limitations of Attribution Methods
- Prone to inherit biases from the input image
- Some methods are less effective when applied to deeper or more "exotic" networks
- No analysis of groups of neurons
- No wildly excepted evaluation method
