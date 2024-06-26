> [!summary] Definition
>  Convolutional neural network (CNN) is a regularized type of feed-forward neural network that learns features via filters (or kernels). Vanishing gradients and exploding gradients are prevented by using regularized weights over fewer connections. For example, for each neuron in the fully-connected layer 10,000 weights would be required for processing an image sized 100 × 100 pixels. However, applying cascaded convolution (or cross-correlation) kernels, only 25 neurons are required to process 5x5-sized tiles. 

> [!info] 
>  Higher-layer features are extracted from wider context windows, compared to lower-layer features.
Are shift invariant.
Counter-intuitively, most CNNs are not invariant to translation, due to the downsampling operation they apply to the input.
Convolutional networks were inspired by biological processes in that the connectivity pattern between neurons resembles the organization of the animal [[Visual Cortex]].

## Convolutional Layer
- The core building block of a CNN
- It consists of a set of learnable filters (or kernels) that have a small [[Convolutional Neural Network#Receptive Field|receptive field]], but extend through the full depth of the input volume
- The filters are convolved across the input volume, producing a 2-dimensional activation map for each filter
- The activation maps indicate the presence and location of the learned features in the input
- The output of the convolutional layer is the stacked set of activation maps for all filters
- The convolutional layer has three main hyperparameters: the depth (number of filters), the stride (number of pixels the filter moves at each step), and the padding (number of zero-valued pixels added to the border of the input)
### Convolution Operation
- This is a mathematical operation that involves multiplying a filter (or kernel) with a patch of the input, and summing up the result into a single output value. 
- This process is repeated for every possible position of the filter on the input, producing an output matrix called a feature map or an activation map. 
- Convolution can be seen as a way of applying a filter to an input to extract some feature from it.
### Receptive Field
- The receptive field is the region in the input space that a feature in a convolutional neural network (CNN) is affected by
- In other words, it is the area of the input that influences the output of a particular feature
-  For example, in an image classification task, the receptive field of a feature in the first convolutional layer is the size of the filter (or kernel) that is applied to the input image. The receptive field of a feature in the second convolutional layer is the area of the input image that is covered by the filter in the first layer, multiplied by the stride (the number of pixels the filter moves at each step) and added by the size of the filter in the second layer. The receptive field of a feature in the final layer is the area of the input image that is covered by all the previous layers
- The receptive field size and location can be calculated using formulas and examples for fully-convolutional networks with different kernels and strides
#### Effective Receptive Field
- The receptive field across multiple layers
- For Example: 3 stacked 3x3 convolution layers have effective receptive filed of 7

## Pooling Layer
- This is a layer that performs a downsampling operation along the spatial dimensions of the input, reducing its size and making the representation more compact and invariant to small translations
- The pooling layer operates independently on every slice of the input and resizes it using a predefined spatial extent (e.g. 2x2) and a stride.
### Max Pooling
This type of pooling returns the maximum value of each patch of the input. It is useful for extracting the most prominent features in the input, such as edges or corners. Max pooling can also provide a form of translation invariance, as the exact location of the feature is not important as long as it is present in the patch.
### Average Pooling
This type of pooling returns the average value of each patch of the input. It is useful for smoothing the input and removing noise. Average pooling can also provide a form of translation invariance, as the average value is not affected by small shifts in the input.
### Global Pooling
This type of pooling reduces each slice of the input to a single value by taking the maximum, average, or sum of all the values in the slice. This is useful for making the output size independent of the input size, and for reducing the number of parameters in the network. Global pooling is often used as the final layer of a CNN to produce a vector of class scores or probabilities.

## Feed-Forward Layer
- Usually in the end for classification