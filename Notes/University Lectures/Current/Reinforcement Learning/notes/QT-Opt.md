> [!summary] Definition
>  QT-OPT (Quality-Value T-Optimization) is a distributed Q-learning algorithm designed to handle continuous action spaces, which are common in complex real-world problems like robotic manipulation.

**Key Features**:
- **Distributed Learning**: QT-OPT utilizes distributed computing to handle large-scale data and complex models, making it suitable for tasks with high-dimensional input spaces, such as vision-based tasks.
- **Continuous Action Space**: It supports continuous actions, which are essential for applications like robotics, where actions are not discrete and require fine-grained control.
**Benefits**:
- **Scalability**: By leveraging distributed computing and offline training, QT-OPT can scale to large datasets and complex models.
- **Stability**: The algorithm is optimized for stability, which is crucial for learning from off-policy data and ensuring consistent performance.
- **Sample Efficiency**: QT-OPT has been shown to be more sample-efficient compared to other algorithms, which means it requires fewer interactions with the environment to learn effective policies.

-> Works ok for up to 40 dimensions... But only if you are Google (insane computation demands)