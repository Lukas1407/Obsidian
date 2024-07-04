**Explanation**:
   - The Z-score measures the number of standard deviations a data point is from the mean. It's a simple way to identify outliers in data.
   - Formula: $Z = \frac{(X - \mu)}{\sigma}$
     - $X$ is the temperature measurement.
     - $\mu$ is the mean temperature.
     - $\sigma$ is the standard deviation of the temperatures.

**Application**:
   - Calculate the mean and standard deviation of the temperature measurements.
   - Compute the Z-score for each measurement.
   - Define a threshold (e.g., $|Z| > 2$ or $|Z| > 3$) to identify anomalies. Measurements with Z-scores above this threshold are considered anomalies.

     ```python
     import numpy as np

     temperatures = np.array([/* your temperature data */])
     mean = np.mean(temperatures)
     std_dev = np.std(temperatures)
     z_scores = (temperatures - mean) / std_dev
     anomalies = np.where(np.abs(z_scores) > 2)  # Threshold
     ```
