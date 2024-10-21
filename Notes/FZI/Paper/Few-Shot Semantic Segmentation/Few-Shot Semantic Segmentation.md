## Transfer Learning
- Use pre-trained backbone, trained on dataset with more samples
- Then train with the few samples of the new class
## Cross-Domain FSS
- Few-Shot (Semantic) Segmentation (FSS)
# Few-Shot
The model has access to only a few labeled examples (typically between 1 and 10) in the target domain.
-> Must efficiently adapt to new target domain
- Distinct classes between source and target domain

## N-Way K-Shot
- We have N new classes with K samples per new class

## Cross-Domain Few-Shot Segmentation (CD-FSS)
The training data and the test data come from different domains. For example, the training set might consist of annotated images from one type of environment (e.g. outdoor scenes), but the goal is to apply the model to another environment (e.g. medical images).

## Basic Approach
- Use pretrained backbone
- Only train segmentation head with (augmented) target samples
- Maybe enough depending on the number of samples
- Would work with Multi-Net with no changes

# Possible Models
In order of how good they fit
## RestNet
[Paper](https://arxiv.org/abs/2308.13469)
[Code](https://github.com/bupt-ai-cz/RestNet)
- considers not only inter-domain transfer but also the preservation of intra-domain knowledge
- Explicitly for CD-FSS
- 69.89AP for 5-shot (SOTA?)
- 64.74 AP for 1-shot
### Details
Semantic Enhanced Anchor Transformation (SEAT) and Intra-domain Residual Enhancement, work together to help the model achieve cross-domain few-shot semantic segmentation by addressing different aspects of the knowledge transfer and preservation process.
#### Semantic Enhanced Anchor Transformation (SEAT)
- Focuses on inter-domain knowledge transfer
- Transforms features from different domains into a domain-agnostic space, enabling better generalization across domains. 
#### Intra-domain Residual Enhancement
- Focuses on preserving intra-domain knowledge
- Residual connections ensure that the model retains useful features from the source domain, while cosine similarity ensures strong matching between support and query features.

## Singular Value Fine-tuning (SVF)
[Paper](https://arxiv.org/pdf/2206.06122)
[Code](https://github.com/syp2ysy/SVF)
- Fine-tunines a small part of parameters in the backbone
- Avoids overfitting, which is common in few-shot tasks due to limited data
- 67.14 AP on 1-shot
### Details
#### Singular Value Decomposition (SVD)
The pre-trained weights of the backbone are decomposed into three matrices using SVD:
- U: Captures semantic information from the input.
- S: The singular values, which control the importance of the semantic features.
- V: The transformation of the features to the output.
#### Fine-tuning Singular Values
Only the singular values are fine-tuned, while U and V remain frozen. This ensures that the core semantic information from the pre-trained model is preserved, while the fine-tuning of S allows the model to adjust to new classes and tasks without overfitting.
#### Preserving Semantic Information
The U and V matrices contain rich semantic information from the original pre-trained model. By keeping them frozen, SVF ensures that these important clues are not destroyed, allowing the model to maintain its performance on previously learned classes while adapting to new ones.





