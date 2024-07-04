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
