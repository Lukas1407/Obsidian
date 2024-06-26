- The most naive [[Semi-Supervised Learning#Proxy label Methods|proxy-label method]]

1. Pass the unlabeled data through the model
2. If the prediction exceeds a threshold $\tau$, add it together with the predicted label, the pseudo-label, to the training data

- Used after pre-training, as a fine-tuning step
- Can be done: 
	- Offline: pseudo-labels of the unlabeled data are updated in a certain frequency 
	- Online: In each iteration the unlabeled images in the current batch are newly „pseudo-labeled“

- The number of labeled and unlabeled images is quite different 
- Therefore it is important to balance the loss of the labeled and the unlabeled images:$$L=\frac{1}{n}\sum_{m=1}^{n}\sum_{i=1}^{C}L(y_{i}^{m},f_{i}^{m}) + \alpha(t)\frac{1}{n'}\sum_{m=1}^{n'}\sum_{i=1}^{C}L(y_{i}^{'m},f_{i}^{'m}),$$ with $\alpha$ being the weight scheduling between the 2 losses
- Advantage:
	- Simple
- Disadvantage:
	- Unlabeled images which do not exceed the threshold will never be used for training
	- Threshold has to be chosen carefully, a bad threshold leads to wrong predictions being accepted
	- Loss balancing has to be chosen carefully, a bad weighting may put too much emphasis on unlabeled data
	-  Accepting more and more pseudo-labeled images successively leads to a smaller influence of the annotated images
	- Training can lead to confirmation bias, i.e., overfitting to wrong pseudolabels, network predictions degenerate.
### Improvements
1. Network output-probabilities are poorly calibrated, using a fixed threshold $\tau$ for accepting pseudo-labels might be a bad strategy
	- Throttling: a variant of self-training which accepts pseudo-labels of the n unlabeled samples with the highest confidence to the labeled data, instead of all samples exceeding $\tau$
2. There exist many strategies to alleviate confirmation bias:
	- Restrict the batch sampling to contain a specified minimum number of annotated samples
	- [[MixUp Data Augmentation]]