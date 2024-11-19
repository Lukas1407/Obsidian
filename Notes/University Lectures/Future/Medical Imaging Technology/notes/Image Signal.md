- An image is nothing more than a n-dimensional signal where the dependent variable is the gray or color intensity
![[Pasted image 20240508103636.png|500]]
## Contrast
- The main aim of an image signal is to <mark style="background: #FFB86CA6;">allow the recognition of shapes and structures</mark> (patterns) by differentiating them from the background
- This is achieved through contrast!
- Contrast is the range between the brightest and the darkest intensity at the interface of two or more objects in an image
![[Pasted image 20240508103737.png|200]]
- Bones and soft tissue have high contrast in [[Computed Tomography (CT)|CT]] images
- Soft tissues have low contrast in [[Computed Tomography (CT)|CT]] images
- Given the roi and background b, the contrast is given by: $$\frac{\text{mean value}(roi)-\text{mean value}(b)}{\text{mean value}(b)}$$
### Improve Contrast
- It is important to have as high of contrast as possible as it is the main goal
- Can be done through post processing steps like histogram equalization
![[Pasted image 20240508104113.png|300]]