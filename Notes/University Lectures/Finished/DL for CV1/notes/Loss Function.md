
## Loss Function for Regression
- [[Regression]] aims to minimize the distance between the predicted value $x_{i}$ and the target value $y_{i}$
### Mean Squared Error (MSE)
$$MSE=\frac{1}{n}\sum_{i=1}^{n}(x_{i}-y_{i})^2$$
- Advantages:
	- Due to the squaring, the loss is always positive; it does not matter if the prediction is above or below the target value
	- Due to the squaring, larger distances are penalized more 
- Disadvantages:
	- Sensitive to outliers, as they produce a large loss due to being squared
### Mean Absolute Error (MAE)
$$MAE=\frac{1}{n}\sum_{i=1}^{n}|x_{i}-y_{i}|$$
- Advantages:
	- Due to using the absolute values, the loss is always positive
	- Less sensitive to outliers than [[Loss Function#Mean Squared Error (MSE)|MSE]]
- Disadvantage:
	- As the average distance approaches 0, the function is undefined -> optimization with [[Gradient Descent]] is not possible
### Huber Loss
$$Huber =\left\{\begin{array}{ll} \frac{1}{n}\sum_{i=1}^{n}(x_{i}-y_{i})^{2}, & |x_{i}-y_{i}|\le \delta \\
         \frac{1}{n}\sum_{i=1}^{n}\delta*(|x_{i}-y_{i}|-\frac{1}{2}\delta), & |x_{i}-y_{i}|> \delta\end{array}\right.$$
 - If the absolute difference between the actual and predicted value is less than or equal to a threshold value, 𝛿, then [[Loss Function#Mean Squared Error (MSE)|MSE]] is applied. Otherwise — if the error is sufficiently large — [[Loss Function#Mean Absolute Error (MAE)|MAE]] is applied.
 - Advantages:
	 - Both of MSE and MAE


## Loss Function for Classification
- [[Classification]] aim to predict the correct label $y_{i}$ given the prediction $p_{model}(x_{i})$
- The probability of the correct label should ideally be 1 while the rest is 0
### Binary Cross-Entropy Loss (BCE)
$$BCE=-\frac{1}{n}\sum_{i=1}^{N}y_{i}*log(p_{model}(x_{i}))+(1-y_{i})*log(1-p_{model}(x_{i}))$$
- Also known as Log Loss
- Used for binary classification problems, where we only output 2 classes, true (1) and false (0)
- If the actual label is 1, the loss depends only on the logarithm of the predicted probability, and vice versa
- The logarithm function ensures that the loss is higher when the predicted probability is farther away from the actual label, and lower when it is closer
- The sum and fraction average the loss over $N$ samples
### Categorical Cross-Entropy Loss (CCE)
$$CCE=-\frac{1}{n}\sum_{i=1}^{N}\sum_{j=1}^My_{ij}*log(p_{model}(x_{ij}))$$
- Used when we have more than 2 class
- [[Loss Function#Binary Cross-Entropy Loss (BCE)|BCE]] is a special form of CCE, where the number of classes $M$ is 2
- The logarithm function ensures that the loss is higher when the predicted probability is farther away from the actual label, and lower when it is closer

## Loss function for Metric/Similarity Learning
- [[Metric Learning]] aims to minimize the distance between the representations of similar objects and maximize the distance between dissimilar ones
### Triplet Loss
- Input: 3 data points
	- An anchor $x_{a}$
	- A positive/similar sample $x_{p}$
	- A negative/dissimilar sample $x_{n}$
- Goal: Find a representation of $x_{a}, x_{p}, x_{n}$ such that the distance between $x_{a}$ and $x_{p}$ is small, and the distance between $x_{a}$ and $x_{n}$ is large
$$TL=\sum_{(a,p,n)\in T}max\{0, \alpha-||x_{a}-x_{n}||_{2}^{2}+||x_{a}-x_{p}||_{2}^{2}\}$$