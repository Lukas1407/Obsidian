Idea: Given a few labeled plus a lot of unlabeled data, train a model that performs better that if it would have just been trained on the labeled data.

## Theoretical Assumptions
1. Continuity/Smoothness Assumption:
	- Points that are close to each other are more likely to share a label
	- This yields a preference for decision boundaries in low-density regions, so few points are close to each other but in different classes
2. Cluster Assumption:
	- Points in the same cluster are more likely to share a label (although data that shares a label may spread across multiple clusters)
	- → Makes possible feature learning with clustering algorithms
3. Manifold assumption:
	- The data lie approximately on a manifold of much lower dimension than the input space
	- → Instead of in input space, we can also work on lower dimensional feature space

## Proxy label Methods
Algorithms that produce proxy labels for the unlabeled data using the prediction function itself or some variant of it without any supervision.
- Proxy labels are used together with labeled data, providing additional training information, even if the produced labels are noisy or weak and do not reflect the ground truth

1. [[Self-Training]]
2. [[Co-Training]]
3. [[Consistency-based Methods]]