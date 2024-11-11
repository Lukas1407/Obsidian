- **Parametric Representation**:
    - A distribution is represented using a fixed set of parameters, like the mean and covariance for a Gaussian distribution.
    - **Advantage**: This form is often easier and more efficient to compute since parametric distributions like Gaussians have well-defined mathematical properties and closed-form solutions.
    - **Limitation**: It restricts the distribution type, so it may not capture complex, multi-modal (multiple peaks), or non-Gaussian shapes accurately.
- **Non-Parametric Representation**:
    - Instead of using a fixed form, non-parametric representations use a collection of samples (or hypotheses) drawn from the distribution. This is particularly useful for complex distributions where we don’t know the exact form.
    - **Advantage**: This method is more flexible, allowing representation of complex distributions that might be multi-modal or non-Gaussian.
    - **Example in Slide**: The two graphs illustrate distributions using samples. The more samples in a region (represented by the ticks at the bottom), the higher the estimated probability or weight for that interval.

![[Pasted image 20241111073332.png#invert|400]]
The two graphs on the slide show different distributions represented non-parametrically:
- The x-axis represents possible values of $x$.
- The black ticks along the x-axis show individual samples.
- The curves represent estimated probabilities based on these samples.
In the left graph, the distribution is smoother and has one primary peak (uni-modal), while the right graph is more complex and has multiple peaks (multi-modal). This illustrates the flexibility of non-parametric methods, which can represent various shapes and forms of distributions based purely on samples, without any need to assume a specific form like a Gaussian.