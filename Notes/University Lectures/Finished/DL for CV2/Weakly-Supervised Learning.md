Problem: Annotating data is costly and time consuming, depending on:
- Quality of the annotations (labels, bounding boxes, masks)
- Complexity of the instances
- Complexity of the domain
Idea: Use weaker, easier to obtain, annotations while bridging the gap to supervised learning with the target annotations.
	-> The labels are not the targets we want to learn!

Create constraints to narrow down the solution space:
- Priors: Assumptions about true statements independent of the given sample
	- Region proposals, super pixels
- Hints: Weaker supervision signal for the given sample
	- image-level label, single points, scribbles, bounding boxes ![[Pasted image 20240226082001.png]]
Challenges:
1. Co-occurrence:
	- When trained with image-level labels
	- Some labels co-occur with features not directly related to the object
2. Instance ambiguity
	- Image-level labels have no indication of the amount of instances
3. Network activation
	- NNs tend to activate primarily to the most discriminative regions 

## Object Detection
- [[Weakly Supervised Deep Detection Network (WSDDN)]]
- [[Online Instance Classifier Refinement (OICR)]]
- [[Instance-Aware, Context-Focused and Memory-Efficient WSOD]]

## Semantic Segmentation
- [[Simple does it]]
- [[Box2Seg]]
- [[Class Activation Map (CAM)]]
- [[Adversarial Erasing]]