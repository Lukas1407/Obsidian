Calculates semantic segmentation masks from image-level labels.

## Process
- Train a [[Convolutional Neural Network|CNN]] on classifying the image-level labels
- The last feature map of the [[Convolutional Neural Network|CNN]] has $n$ number of channels, with $n$ being the number of classes
- Followed by a global average pooling (GAP), which produces a $n$ dimensional vector, and a linear layer ![[Pasted image 20240226091942.png]]
- Remove the GAP and apply the linear layer to each spatial position of the last feature map
- Resize the resulting map to input image size
- This way, for each class, we get spatial activations → class activation maps
## Semantic Segmentation
- We can use the CAM which corresponds to the ground truth image-level label, threshold it, to obtain a semantic segmentation mask 
- We can use that to train a separate model for semantic segmentation
- Problem: Because the CNN was trained on classification, it mostly focuses on the most discriminative parts of the object → segmentation mask are very coarse