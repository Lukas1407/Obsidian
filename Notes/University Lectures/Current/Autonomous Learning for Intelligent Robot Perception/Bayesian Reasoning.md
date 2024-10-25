- Suppose a robot stops in front of a door. It has a sensor (e.g. a camera) to measure the state of the door (open or closed). 
- **Problem**: the sensor may fail
![[Pasted image 20241025123002.png#invert|400]]
- **Question**: How can we obtain knowledge about the environment from sensors that may return incorrect results?
- **Answer**: using probabilities!
## Mathematical Formulation of the Example
- We define 2 random variables $z$ and $open$ where $z$ is "light on" or "light off"
- **Question:** What is $p(open|z)$
### Diagnostic Reasoning
   - **Diagnostic reasoning** refers to finding $p(\text{open} | z)$, meaning we want to determine the probability that the door is open, given some evidence $z$ (like a sensor reading).
   - Diagnostic reasoning moves from **effects** to **causes**. We observe the effect (sensor reading) and infer the cause (whether the door is open).
### Causal Reasoning
   - **Causal reasoning** refers to finding $p(z | \text{open})$, meaning we want to find the probability of observing the evidence $z$ (sensor reading), given that we know the door is open.
   - Causal reasoning moves from **causes** to **effects**. We assume the door is open and ask what the likelihood is of observing a specific sensor reading.
### Bayes Rule Application
   - Bayes' Rule helps connect diagnostic and causal reasoning. Even if we have **causal knowledge** (i.e., $p(z | \text{open})$), Bayes' Rule allows us to infer the diagnostic probability $p(\text{open} | z)$.
   $$
   p(\text{open} | z) = \frac{p(z | \text{open}) p(\text{open})}{p(z)}
   $$
   Where:
   - $p(z | \text{open})$: The likelihood of observing $z$ given the door is open.
   - $p(\text{open})$: The prior probability that the door is open.
   - $p(z)$: The total probability of observing $z$, calculated using the **law of total probability**:
     $$
     p(z) = p(z | \text{open}) p(\text{open}) + p(z | \neg \text{open}) p(\neg \text{open})
     $$
### Example with Numbers**
1. **Given Sensor Model**:
   - The sensor model provides us with probabilities for detecting whether the door is open or not:
     - $p(z | \text{open}) = 0.6$: The probability of the sensor reading $z$ given that the door is open.
     - $p(z | \neg \text{open}) = 0.3$: The probability of the sensor reading $z$ given that the door is **not** open.
2. **Prior Probability**:
   - We are given the **prior probability**:
     - $p(\text{open}) = 0.5$: The probability that the door is open (before observing the sensor reading).
     - $p(\neg \text{open}) = 0.5$: The probability that the door is not open.
3. **Applying Bayes’ Rule**:
   Using the provided data, we apply Bayes' Rule to compute $p(\text{open} | z)$, the probability that the door is open, given the sensor reading $z$.
   We first calculate $p(z)$ using the **law of total probability**:
   $$
   p(z) = p(z | \text{open}) p(\text{open}) + p(z | \neg \text{open}) p(\neg \text{open})
   $$
   Substituting the given values:
   $$
   p(z) = 0.6 \cdot 0.5 + 0.3 \cdot 0.5 = 0.3 + 0.15 = 0.45
   $$

   Now, apply Bayes' Rule:
   $$
   p(\text{open} | z) = \frac{p(z | \text{open}) p(\text{open})}{p(z)} = \frac{0.6 \cdot 0.5}{0.45} = \frac{0.3}{0.45} = \frac{2}{3} \approx 0.67
   $$
4. **Conclusion**:
   - After applying Bayes’ Rule, we find that the probability that the door is open, given the sensor reading $z$, is $0.67$ or $67\%$.
   - The evidence $z$ **raises the probability** that the door is open from the prior $0.5$ (50%) to $0.67$ (67%).

