Solves the problem of [[Class Activation Map (CAM)]], that the produces segmentation masks only contain the most discriminative regions via an iterative process.

## Process
Repeat, until classification scores deteriorate: 
- Train classifier on image level labels 
- Calculate [[Class Activation Map (CAM)|CAMs]] and threshold them to determine most discriminative regions 
- Mask out these thresholded regions from the images in the dataset 
- Train a new classifier on the images with masked out regions 
Masked out regions of an image make up its segmentation mask, which capture more parts of the object than [[Class Activation Map (CAM)]]
Train a standard segmentation model supervised with these masks