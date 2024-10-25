## Sample Space ($S$)
   - A **sample space** is the set of all possible outcomes of a random experiment.
   - Examples:
     - **Coin toss**: The outcomes are heads ($H$) and tails ($T$), so the sample space is $S = \{H, T\}$.
     - **Distance measurement**: Here, the sample space could be all non-negative real numbers ($S = \mathbb{R}_0^+$), indicating that the measurement can take any positive value (including zero).
## Random Variable ($X$)
   - A **random variable** is a function that assigns a numerical value to each outcome in the sample space.
   - Example:
     - For a coin toss, the random variable $X$ might assign a value of $1$ to heads ($H$) and $0$ to tails ($T$). So, $H = 1$ and $T = 0$.
   - **Notation**: Random variables are typically denoted by uppercase letters (e.g., $X$), and their specific values are represented with lowercase letters (e.g., $X = x$).
### Discrete vs. Continuous Random Variables:
#### Discrete Random Variable
-  If the sample space ($S$) is **countable** (i.e., consists of distinct, separate values), the random variable is called discrete. An example is the outcome of a coin toss (heads or tails).
##### Example
- Suppose a robot knows that it is in a room, but it does not know in which room. 
- There are 4 possibilities: Kitchen, Office, Bathroom, Living room 
- Then the random variable Room is discrete, because it can take on one of four values. 
- The probabilities are, for example $$P(\text{Room = kitchen}) = 0.7$$$$P(\text{Room = office}) = 0.2$$$$P(\text{Room = batchroom}) = 0.08$$$$P(\text{Room = living room}) = 0.02$$
#### Continuous Random Variable: 
- If the sample space is **uncountable** (i.e., it contains an infinite number of values, such as the real numbers), the random variable is continuous. An example would be a measurement of distance or time.
##### Example
- A robot is moving forward 5 meters from a given starting point.
- The robot's position $X$ after moving forward is not deterministic—it varies slightly due to natural uncertainties (like sensor errors or movement precision).
- Therefore, $X$ is modeled as a **continuous random variable**.

- The robot's position $X$ is described by a **normal distribution** (also known as a Gaussian distribution), which is commonly used for modeling continuous variables with some uncertainty.
- The **normal distribution** is defined by two parameters:
     - $\mu$ (the mean), which represents the expected value (in this case, 5 meters, the robot's intended position).
     - $\sigma^2$ (the variance), which measures the spread or uncertainty around the mean. A larger variance means the position is more spread out (more uncertain), while a smaller variance indicates that the robot's position is closer to the mean (more precise).
###### Probability Density Function (PDF):
   - The **PDF** of the normal distribution is given by the equation:
     $$
     p(x) = \frac{1}{\sqrt{2\pi\sigma^2}} e^{-\frac{1}{2} \left(\frac{x - 5}{\sigma}\right)^2}
     $$
     - This equation describes how the probability density is distributed over different values of $x$.
     - The term $\mu = 5$ indicates the robot is expected to be at 5 meters.
     - The term $\sigma^2$ controls how wide the curve is (the variance). A higher $\sigma^2$ means more spread (more uncertainty), and a lower $\sigma^2$ means a more peaked distribution (less uncertainty).
![[University Lectures/Current/Autonomous Learning for Intelligent Robot Perception/images/Untitled.png#invert|300]]


   - The total probability over the entire range of possible values is 1, but the probability of any exact value (like $X = 5$) is zero because the probability is calculated over intervals, not individual points.
   - The probability of an exact value like $X = 5$ is the **area under the curve** at that single point, which is essentially a line. Since a line has no width, the area (and therefore the probability) is zero.
   - Mathematically, for continuous random variables:
     $$
     P(X = 5) = \int_5^5 p(x) dx = 0
     $$
   - Instead of asking for the probability that $X$ is exactly 5, we ask for the probability that $X$ falls within a range, for example, $P(4 \leq X \leq 6)$, where the probability is the area under the curve between 4 and 6.
###### Cumulative Distribution Function (CDF):
![[Pasted image 20241025125009.png#invert|300]]
   - In the graph, you see that the red-shaded area represents $P(X \leq 5)$, which is the **cumulative probability** up to $X = 5$.
   - This is calculated by integrating the PDF from $-\infty$ to 5. The cumulative probability gives the probability that the random variable $X$ takes a value less than or equal to 5.
### Probability for Discrete and Continuous Variables:
   - **Discrete Case**: For a discrete random variable, the total probability across all possible values $x$ must sum to 1:
     $$
     \sum_x p(X = x) = 1
     $$
     This means the sum of the probabilities of all possible outcomes of the random variable equals 1.
   - **Continuous Case**: For a continuous random variable, instead of a sum, we use an **integral** to ensure that the probability density over all possible values equals 1:
     $$
     \int p(X = x) dx = 1
     $$
     Here, $p(X = x)$ represents a **probability density function**, and the integral ensures that the total probability over all possible values is 1.
## Joint Probability
   - The **joint probability** of two random variables $X$ and $Y$ is the probability that both $X = x$ and $Y = y$ occur at the same time.
   - This is represented as $p(X = x \text{ and } Y = y)$ or $p(x, y)$.
   - In shorthand, you will often see it written as $p(x, y)$.
## Conditional Probability
   - **Conditional probability** is the probability of one event given that another event has occurred.
   - The **conditional probability** of $X = x$ given $Y = y$ is represented as $p(X = x | Y = y)$ or simply $p(x | y)$.
   - This can be calculated using the formula:
     $$
     p(x | y) = \frac{p(x, y)}{p(y)}
     $$
     This means that to calculate the conditional probability of $X$ given $Y$, you divide the joint probability of $X$ and $Y$ by the probability of $Y$.
## Independence
   - Two random variables $X$ and $Y$ are **independent** if knowing the value of one does not affect the probability of the other. Mathematically, this means that:
     $$
     p(x, y) = p(x) \cdot p(y)
     $$
   - This implies that the joint probability of independent events is the product of their individual probabilities.
### Conditional Independence
   - If $X$ and $Y$ are independent, the conditional probability simplifies to:
     $$
     p(x | y) = p(x)
     $$
     This shows that if $X$ and $Y$ are independent, the probability of $X$ is unaffected by $Y$.
### Conditional Independence given a 3rd Variable
Two random variables, $X$ and $Y$, are said to be **conditionally independent** given a third random variable $Z$, if:
$$
p(x, y | z) = p(x | z) p(y | z)
$$
This means that once you know the value of $Z$, the variables $X$ and $Y$ no longer provide any additional information about each other. In other words, knowing $X$ gives no extra information about $Y$ beyond what you already know from $Z$, and vice versa.
### **Explanation**:
- $p(x, y | z)$: This is the joint probability of $X$ and $Y$ given $Z$, which means it's the probability that both $X = x$ and $Y = y$ happen when $Z$ has a specific value.
- $p(x | z)$: This is the conditional probability of $X$, given $Z$, which means it's the probability of $X$ given that you already know the value of $Z$.
- $p(y | z)$: Similarly, this is the conditional probability of $Y$, given $Z$.
When $X$ and $Y$ are conditionally independent, their joint probability $p(x, y | z)$ can be **factored** into the product of their individual probabilities given $Z$. This is a key feature of conditional independence.
### **Equivalence**:
Now, we want to show that this is equivalent to:
1. $p(x | z) = p(x | y, z)$
2. $p(y | z) = p(y | x, z)$
#### Step 1: Start with the definition of conditional probability
We know that conditional probabilities are defined as:
$$
p(x | y, z) = \frac{p(x, y, z)}{p(y, z)}
\quad \text{and} \quad
p(x | z) = \frac{p(x, z)}{p(z)}
$$

#### Step 2: Joint Probability for Conditional Independence
From the definition of conditional independence, we have:
$$
p(x, y | z) = p(x | z) p(y | z)
$$
This can be rewritten as:
$$
p(x, y, z) = p(x | z) p(y | z) p(z)
$$
Now, this joint probability expresses how $X$, $Y$, and $Z$ are related when $X$ and $Y$ are conditionally independent given $Z$.

#### Step 3: Show that $p(x | z) = p(x | y, z)$

We begin by expanding $p(x | y, z)$:
$$
p(x | y, z) = \frac{p(x, y, z)}{p(y, z)}
$$
Substitute the joint probability $p(x, y, z)$ from Step 2:
$$
p(x | y, z) = \frac{p(x | z) p(y | z) p(z)}{p(y, z)}
$$
Since $p(y, z) = p(y | z) p(z)$, this simplifies to:
$$
p(x | y, z) = \frac{p(x | z) p(y | z) p(z)}{p(y | z) p(z)}
$$
The terms $p(y | z)$ and $p(z)$ cancel out, leaving us with:
$$
p(x | y, z) = p(x | z)
$$

This shows that $p(x | y, z) = p(x | z)$, which is one half of the equivalence.

#### Step 4: Show that $p(y | z) = p(y | x, z)$

Similarly, we can expand $p(y | x, z)$:
$$
p(y | x, z) = \frac{p(x, y, z)}{p(x, z)}
$$
Substitute the joint probability $p(x, y, z)$ from Step 2:
$$
p(y | x, z) = \frac{p(x | z) p(y | z) p(z)}{p(x, z)}
$$
Since $p(x, z) = p(x | z) p(z)$, this simplifies to:
$$
p(y | x, z) = \frac{p(x | z) p(y | z) p(z)}{p(x | z) p(z)}
$$
The terms $p(x | z)$ and $p(z)$ cancel out, leaving us with:
$$
p(y | x, z) = p(y | z)
$$

This shows that $p(y | x, z) = p(y | z)$, which completes the equivalence.
### **Intuition**:
Conditional independence can be thought of like this: Imagine you're trying to predict $X$ and $Y$ based on some additional variable $Z$ (which could be common knowledge or background information). Once you have the value of $Z$, knowing $X$ doesn't help you predict $Y$ any better, and vice versa. $X$ and $Y$ are independent **given** $Z$.
### **Real-world Example**:
Suppose $Z$ is the weather, $X$ is whether you'll bring an umbrella, and $Y$ is whether you'll wear a raincoat. Given the weather (whether it’s rainy or sunny), the decision to bring an umbrella and wear a raincoat might be **conditionally independent**—once you know the weather, knowing that you brought an umbrella doesn't tell you more about whether you'll wear a raincoat, because both decisions depend primarily on the weather.
## Sum Rule:
   - This rule is used to find the marginal probability of $X = x$ by summing over all possible values of $Y$:
     $$
     p(x) = \sum_y p(x, y)
     $$
   - The sum rule is crucial for calculating the overall probability of an event by considering all possible scenarios.
## Product Rule
   - The **product rule** expresses the joint probability of $X = x$ and $Y = y$ in terms of the conditional probability:
     $$
     p(x, y) = p(y | x) \cdot p(x)
     $$
   - This helps decompose the joint probability into simpler parts, such as a conditional probability and the marginal probability.

## Law of Total Probability:
   - The **law of total probability** allows you to calculate the marginal probability of $X = x$ by summing or integrating over all possible values of $Y$:
     - **For Discrete Variables**:
       $$
       p(x) = \sum_y p(x | y) \cdot p(y)
       $$
     - **For Continuous Variables**:
       $$
       p(x) = \int p(x | y) \cdot p(y) \, dy
       $$
   - This theorem is helpful when you don't directly know the probability of $X = x$, but you have information about the conditional probability of $X$ given $Y$, and the distribution of $Y$.
### Marginalization:
   - The process of summing or integrating over all possible values of $Y$ to obtain the marginal probability of $X = x$ is called **marginalization**.
   - This is used in many probabilistic models to simplify complex distributions by focusing on just one variable.

