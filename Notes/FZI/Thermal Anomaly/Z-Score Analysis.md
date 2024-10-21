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

## Paper
The z-score, also known as the standard score, is a statistical measure that describes a value's position relative to the mean of a group of values, measured in terms of standard deviations. Specifically, the z-score is calculated by subtracting the mean of the dataset from the individual value and then dividing the result by the standard deviation of the dataset. Mathematically, it is expressed as $z = \frac{(X - \mu)}{\sigma}$, where $X$ is the value, $\mu$ is the mean, and $\sigma$ is the standard deviation. The z-score effectively normalizes the data, allowing for the comparison of values from different distributions or the identification of outliers. In the context of calculating thermal anomalies based on n temperature measurements, using the z-score with a threshold of 3 is particularly sensible because it provides a standardized way to identify significant deviations from the average temperature. A z-score threshold of 3 indicates that any temperature measurement that lies more than three standard deviations away from the mean is considered an anomaly. This threshold is robust and statistically sound, as it ensures that only genuinely unusual temperature values are flagged as anomalies. This method thus ensures accuracy and reduces the likelihood of false positives in anomaly detection, which is crucial for reliable thermal analysis.

