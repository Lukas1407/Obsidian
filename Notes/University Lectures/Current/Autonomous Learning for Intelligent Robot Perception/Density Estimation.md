1. **Given:**
   - Data points $\mathbf{x}_1, \mathbf{x}_2, \ldots, \mathbf{x}_N$ sampled from some unknown distribution $p(x)$.
   - The goal is to estimate the probability density $p(x)$ at a given point $x$.
2. **Concept:**
   - Consider a small region $R$ around $x$ with a volume $V$.
![[Untitled 4.png#invert|300]]
   - The probability of a sample falling into $R$ is approximately:
     $$
     \int_R p(x) dx \approx p(x) V
     $$
     Here, $\int_R p(x) dx$ represents the probability of $x$ being in region $R$, and $p(x) V$ gives the estimated value.
3. **Rewriting:**
   - Let $K$ be the number of data points that fall inside $R$. Then:
     $$
     \int_R p(x) dx \approx \frac{K}{N} \quad \text{and hence,} \quad p(x) \approx \frac{K}{NV}
     $$
   - This formula represents **density estimation**: dividing the count $K$ by the total number of samples $N$ and the volume $V$ of the region.
### **Kernel Density Estimation (Boxcar Kernel)**
1. **Improved Estimation:**
   - Instead of focusing on a fixed region $R$, density estimation can use **kernels** to weigh contributions from nearby points.
2. **Formula:**
   - A kernel-based estimator can be written as:
     $$
     p(x) = \frac{1}{N} \sum_{i=1}^N k_V(x - x_i)
     $$
   - Here, $k_V$ is the **boxcar kernel**, which assigns equal weight to all points within a fixed distance $V$ from $x$, and zero weight to points farther away.
![[Pasted image 20241125122410.png#invert|300]]
3. **Boxcar Kernel:**
   - The boxcar kernel is a simple rectangular function that defines a fixed region (the red-shaded area). It’s useful for basic estimation but can lead to sharp changes in $p(x)$.
### **Gaussian Kernel Density Estimation**
1. **Smoother Estimation:**
   - The boxcar kernel can lead to abrupt changes in the density estimate. A smoother alternative is the **Gaussian kernel**:
     $$
     p(x) = \frac{1}{N} \sum_{i=1}^N k_V(x - x_i)
     $$
     where $k_V$ is now a Gaussian function.
![[Pasted image 20241125122813.png#invert|300]]
1. **Gaussian Kernel:**
   - The Gaussian kernel assigns weights to data points based on their distance from $x$ using a normal distribution. Points closer to $x$ contribute more, and farther points contribute less.
2. **Effect of Gaussian Kernel:**
   - The red curve (Gaussian) shows a smoother density estimate compared to the boxcar kernel. It reduces sharp discontinuities and gives a better approximation for most real-world data distributions.
