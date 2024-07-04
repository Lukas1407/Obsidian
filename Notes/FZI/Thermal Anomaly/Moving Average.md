**Explanation**:
   - The moving average smooths out short-term fluctuations and highlights longer-term trends or cycles in data.
   - There are different types of moving averages: simple, weighted, and exponential.

**Application**:
   - Select a window size (e.g., 10 measurements).
   - Compute the moving average for the temperature data.
   - Calculate the residuals (actual temperature - moving average).
   - Identify anomalies by setting a threshold for the residuals. Large residuals indicate anomalies.
     ```python
     window_size = 10
     moving_avg = np.convolve(temperatures, np.ones(window_size)/window_size, mode='valid')
     residuals = temperatures[window_size-1:] - moving_avg
     anomalies = np.where(np.abs(residuals) > threshold)  # Define your threshold
     ```