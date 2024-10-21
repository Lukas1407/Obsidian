Cumulative Sum (CUSUM) is a sequential analysis technique used to detect shifts in the mean level of a time series. It is particularly useful for monitoring changes in the mean value of a process over time and is widely used in quality control.

#### Key Concepts

1. **CUSUM Chart**
   - A CUSUM chart accumulates the sum of deviations of the data points from a target value or mean.
   - It helps in identifying small shifts in the process mean that might be missed by other methods.

2. **CUSUM Calculation**
   - The CUSUM value at time $t$, $S_t$, is calculated as:
     $$
     S_t = \sum_{i=1}^{t} (T_i - \mu)
     $$
     where $T_i$ is the temperature measurement at time $i$ and $\mu$ is the target mean or reference value.
   - There are two types of CUSUM charts: **positive CUSUM** and **negative CUSUM**, which help detect upward and downward shifts, respectively.

3. **Decision Interval**
   - A decision interval (threshold) is set to determine when the CUSUM value indicates a significant shift in the mean.

#### Steps to Apply CUSUM

1. **Data Preparation**
   - Collect n temperature measurements $T_1, T_2, \ldots, T_n$.
   - Ensure the data is in a time series format with consistent time steps.

2. **Set Target Mean and Decision Interval**
   - Determine the target mean ($\mu$) and decision interval (threshold).

3. **Calculate CUSUM Values**
   - Calculate the cumulative sum for each time step.
   - Example in Python:
     ```python
     import numpy as np
     import matplotlib.pyplot as plt

     # Example temperature data (replace with actual data)
     temperature_data = np.random.normal(20, 5, 100)

     # Set target mean
     target_mean = np.mean(temperature_data)

     # Calculate CUSUM values
     cusum_pos = np.maximum.accumulate(np.cumsum(temperature_data - target_mean))
     cusum_neg = np.minimum.accumulate(np.cumsum(target_mean - temperature_data))

     # Plot CUSUM chart
     plt.plot(cusum_pos, label='Positive CUSUM')
     plt.plot(cusum_neg, label='Negative CUSUM')
     plt.axhline(y=0, color='black', linestyle='--')
     plt.legend()
     plt.show()
     ```

4. **Detect Shifts**
   - Identify points where CUSUM values exceed the decision interval, indicating a shift in the mean.

#### Example Application
Suppose you have hourly temperature measurements for an object over a month (720 hours).

1. **Data Preparation**: Ensure the data is hourly and covers the entire period.
2. **Set Target Mean and Decision Interval**: Calculate the mean of the temperature data.
3. **Calculate CUSUM Values**:
   ```python
   import numpy as np
   import matplotlib.pyplot as plt

   # Example temperature data (replace with actual data)
   temperature_data = np.random.normal(20, 5, 720)

   # Set target mean
   target_mean = np.mean(temperature_data)

   # Calculate CUSUM values
   cusum_pos = np.maximum.accumulate(np.cumsum(temperature_data - target_mean))
   cusum_neg = np.minimum.accumulate(np.cumsum(target_mean - temperature_data))

   # Plot CUSUM chart
   plt.plot(cusum_pos, label='Positive CUSUM')
   plt.plot(cusum_neg, label='Negative CUSUM')
   plt.axhline(y=0, color='black', linestyle='--')
   plt.legend()
   plt.show()
   ```

4. **Detect Shifts**: Identify significant deviations in the CUSUM values indicating potential anomalies.


