- Goal: Assign target categories without any training data 
- How?: Use some form of auxiliary semantic information to link the known and unknown categories 
	- Auxiliary semantic information (also “side-information”, “semantic embedding”) represents observable distinguishing properties of objects

## Types of auxiliary semantic information
1. Semantic embedding of category names
	- Vectors representing the meaning of a category
	- For example with [[Word2Vec]], which results in words appearing in the same context being represented similarly
2. Attributes
	- Real-valued or binary
3. Textual descriptions of categories
4. Wikipedia & WordNet
5. Eye gaze

## Inductive Zero-Shot Learning
- Standard from of ZSL
- No access to the data of unknown classes
## Transductive Zero-Shot Learning
- $\approx$ ZSL + [[Semi-Supervised Learning]]
- Access to unlabeled data of the unknown classes
## Generalized Zero-Shot Learning
- Both seen and unseen categories are present at test-time 
- Goal: recognize unseen seen classes
- A Generalized ZSL models should be capable of: 
1. Standard classification of previously seen categories 
2. Knowledge transfer to new unseen classes (→e.g. through Zero-Shot learning) 
3. Discriminating between those two cases (→novelty detection)

## Evaluating Zero-Shot Learning
1. Inductive ZSL: Evaluate accuracy only on the unseen classes
2. Generalized ZSL: Evaluate accuracy on the seen and unseen classes
	- Problem: The performance is strongly affected depending on which categories are known and unknown and the accuracy on the seen classes in far better
	- Solution: Evaluate using the harmonic mean of the known and unknown accuracies:$$H=\frac{2*acc_{known}*acc_{unknwon}}{acc_{known}+acc_{unkown}}$$
## Models
1. [[CONSE]]
2. [[DeViSE]]
3. [[Direct Attribute Prediction (DAP)]]
4. [[Indirect Attribute Prediction (IAP)]]
5. [[Feature Generating Networks (f-CLSWGAN)]]