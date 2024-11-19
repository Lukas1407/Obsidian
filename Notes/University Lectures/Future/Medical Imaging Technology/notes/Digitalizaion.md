> [!tldr] Definition
>  The process of converting an analog signal to a digital one. It consists of 2 stages:
>  1. [[Digitalizaion#Sampling|Sampling]]
>  2. [[Digitalizaion#Quantization|Quantization]]

![[Pasted image 20240508105546.png#invert|500]]
## Sampling
- Sampling is the process of <mark style="background: #FFB86CA6;">converting a continuous signal into a discrete signal</mark>.
- It involves measuring the signal’s intensity at regular intervals, known as **sample points**.
- The rate at which we sample is critical and is guided by the **Nyquist-Shannon sampling theorem**. This theorem states that to accurately reconstruct a signal, <mark style="background: #FFB86CA6;">it must be sampled at least at twice the highest frequency</mark> present in the signal.
- In terms of images, sampling refers to selecting a finite number of pixels to represent the image, which corresponds to the **resolution**.
## Quantization
- Quantization follows sampling and is the process of <mark style="background: #FFB86CA6;">mapping the sampled values to a finite set of levels</mark>.
- In essence, it’s about converting the infinite precision of the samples into a form that can be stored digitally, which usually means mapping them to integer values.
- This step introduces <mark style="background: #FF5582A6;">quantization error</mark>, which is the difference between the actual sample value and the quantized value. 
	- The more levels we have, the smaller the error.
- For images, this means assigning a discrete value to each pixel, typically representing different shades of gray or color.
## 2D Digitization
![[Pasted image 20240508105727.png|500]]

| # bits | Dynamic Conversion     |
| ------ | ---------------------- |
| 1      | $2^1$ (black or white) |
| 2      | $2^2=4$                |
| ...    |                        |
| 8      | $2^8=256$              |
- The more bits, the higher the amount of gray values we can represent, the higher the quality of the image
	- It influences the [[Image Resolution#Radiometric Resolution|radiometric resolution]]