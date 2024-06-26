The idea is to take repeated steps in the opposite direction of the gradient (or approximate gradient) of the function at the current point, because this is the direction of steepest descent. This way we will step towards a local minima.![[Pasted image 20240303132814.png#invert|300]]
A simple extension of gradient descent, [[Stochastic Gradient Descent]], serves as the most basic algorithm used for training most deep networks today.

In standard or "batch" gradient descent, the update function is as followed: $$w :=w-\eta \frac{1}{n}\sum_{i=1}^{n}\Delta Q_{i}(w)$$
