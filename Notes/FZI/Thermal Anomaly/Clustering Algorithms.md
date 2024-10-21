**Explanation**:
   - **K-Means**: Partitions data into $K$ clusters where each data point belongs to the cluster with the nearest mean.
   - **DBSCAN**: Density-based clustering that finds core samples of high density and expands clusters from them.

**Application**:
   - **K-Means**:
     1. Choose $K$ (number of clusters).
     2. Apply K-Means to temperature data.
     3. Identify outliers by finding points that are far from their cluster centers.
   - **DBSCAN**:
     1. Choose parameters $\epsilon$ (neighborhood radius) and $minPts$ (minimum points to form a cluster).
     2. Apply DBSCAN to the temperature data.
     3. Points not belonging to any cluster are considered anomalies.

     ```python
     from sklearn.cluster import KMeans

     kmeans = KMeans(n_clusters=3).fit(temperatures.reshape(-1, 1))
     distances = kmeans.transform(temperatures.reshape(-1, 1))
     anomalies = np.where(np.min(distances, axis=1) > threshold)  # Define your threshold
     ```
     ```python
     from sklearn.cluster import DBSCAN

     dbscan = DBSCAN(eps=0.5, min_samples=5).fit(temperatures.reshape(-1, 1))
     anomalies = np.where(dbscan.labels_ == -1)
```

## Paper k Means
K-means clustering is an unsupervised machine learning algorithm used to partition a dataset into a predefined number of clusters, denoted as $k$. The algorithm works by initializing $k$ centroids randomly and then iteratively refining their positions to minimize the within-cluster variance. Specifically, it assigns each data point to the nearest centroid based on the Euclidean distance, recalculates the centroids as the mean of the points in each cluster, and repeats this process until convergence, where the assignments no longer change or the centroids stabilize. The goal is to ensure that data points within the same cluster are as similar as possible while maximizing the difference between clusters. 
In the context of calculating thermal anomalies based on temperature measurements, using K-means clustering with a threshold of 3 and 3 clusters is highly appropriate. This approach allows for the identification of distinct patterns or groups in the temperature data, which might represent normal conditions, moderate anomalies, and extreme anomalies. By setting a threshold of 3, it aligns with the methodology of using z-scores, where values significantly deviating from the mean are flagged. The K-means algorithm helps to segregate these deviations into meaningful clusters, facilitating a more nuanced understanding of thermal behavior. This clustering technique provides a clear and structured way to categorize temperature measurements, ensuring that outliers are effectively identified and analyzed within the context of their cluster. This method enhances the robustness of anomaly detection, allowing for more accurate and reliable thermal analysis, particularly when dealing with large and complex datasets.

## Paper DBSCAN
DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is an unsupervised machine learning algorithm used for clustering data points based on their density. Unlike K-means, DBSCAN does not require specifying the number of clusters in advance. Instead, it relies on two parameters: epsilon ($\epsilon$) and the minimum number of samples ($\text{min\_samples}$). The epsilon parameter defines the maximum distance between two points to be considered neighbors, while the minimum number of samples parameter specifies the minimum number of points required to form a dense region, or a cluster. 
The algorithm works by identifying core points, which have at least $\text{min\_samples}$ points within their $\epsilon$-radius. Points that are within the $\epsilon$-radius of a core point but do not have enough neighbors to be core points themselves are labeled as border points. Points that are not reachable from any core point are considered noise. DBSCAN then clusters the core points and their reachable neighbors into distinct groups, effectively identifying areas of high density separated by areas of low density.
In the context of calculating thermal anomalies based on temperature measurements, using DBSCAN with $\epsilon = 0.5$ and $\text{min\_samples} = 5$ is particularly useful. This approach allows for the detection of anomalies based on the density of the measurements rather than their distribution alone. The choice of $\epsilon = 0.5$ ensures that only points within a close proximity are considered neighbors, which is crucial for identifying localized anomalies. The $\text{min\_samples} = 5$ parameter ensures that only clusters with a significant number of points are considered, filtering out noise and minor variations. This method is advantageous when dealing with irregular and non-spherical data distributions, as it can effectively identify clusters of varying shapes and sizes without the need for prior knowledge of the number of clusters. By focusing on the density of the data, DBSCAN is capable of revealing underlying structures and anomalies that might be missed by other clustering methods, providing a robust and flexible tool for thermal anomaly detection.

