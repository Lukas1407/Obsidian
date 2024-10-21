## An Overview of multi-task learning
- (MTL) aims to improve the performance of multiple related learning tasks by leveraging useful information among them
- A good recipe by exploiting useful information from other related learning tasks to help alleviate this data sparsity problem
- jointly learning multiple tasks is empirically and theoretically found to lead to better performance than learning them independently
- Similar to human learning, it is useful to learn multiple learning tasks simultaneously since the knowledge in a task can be utilized by other related tasks.
- For example, similar to MTL, transfer learning also aims to transfer knowledge from one task to another but the difference lies in that transfer learning hopes to use one or more tasks to help a target task while MTL uses multiple tasks to help each other
- We use multi-task supervised learning (MTSL). specifically Feature-based MTSL
## QuadroNet
- Computational resources for such applications are often constrained by cost, size and power requirements of the underlying hardware platform. Hence, using a single network to jointly solve multiple perception problems has emerged as a key strategy for satisfying the requirements of fast and accurate results in compute-constrained settings. At runtime, sharing large portions of the network among several tasks reduces the overall inference latency [16].
- However, naively combining multiple tasks into a single network can often lead to reduced accuracy for each task due to limited model capacity.
- Standard jointly-trained multi-task architectures typically have independent task-specific sub-networks that share only a feature extraction backbone. Such networks do not explicitly model inter-task relationships such as the coincidence of object/region boundaries and depth discontinuities. While joint training of these tasks results in improved accuracies for all tasks, we see still greater improvements in accuracy when enforcing cross-task consistency through our novel formulation.
## Feature-based MTSL
### Why Feature-based Multi-task Learning?
- **Shared Backbone**: The backbone of your model is used to extract features from the input data. These extracted features are then shared between two different task-specific heads (instance segmentation and semantic segmentation). This indicates that the model assumes that both tasks can leverage a similar or identical set of features, learned from the backbone.
- **Task-specific Heads**: The model has two heads with different parameters for each task, but the key part is that these tasks are not learning entirely independently. They are benefiting from a **shared feature extraction process**, which is exactly the goal of feature-based MTSL.
### **Feature Transformation Approach**
This approach learns the shared feature representation as a transformation (either linear or nonlinear) of the original input data. In your case, the backbone (which is a deep neural network) performs this transformation on the input data, resulting in a shared feature representation for both tasks. This representation is **nonlinear** because convolutional layers in the backbone typically involve nonlinear activation functions like ReLU.
## Applications in CV include
- Vehicle instance segmentation

## Problem with Multi-Net
- do not model the inter-task relationships explicitly, and hence their multi-task approach fails to perform much better than their baselines.