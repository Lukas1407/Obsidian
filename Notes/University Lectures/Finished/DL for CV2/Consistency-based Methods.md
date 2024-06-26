Consistency Regularization:
	Aims that the model produces the same output for a sample even under perturbations, like [[Data Augmentation]] or noise.

In this type of methods, two roles are commonly created, either explicitly or implicitly: 
- The student
- The teacher guides the student to approximate its performance under perturbations. The perturbations could come from the noise of the input or the dropout layer, etc. A consistency constraint is then imposed on the predictions between two roles, and forces the unlabeled data to meet the smoothness assumption of semi-supervised learning.

## Methods
1. [[Mean Teacher]]