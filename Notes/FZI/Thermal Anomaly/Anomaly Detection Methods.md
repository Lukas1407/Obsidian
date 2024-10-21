## Statistical Methods
- [[Z-Score Analysis]]
- [[Moving Average]]
- **Mean and Standard Deviation**: Calculate the mean and standard deviation of the temperature measurements. Anomalies can be identified as readings that lie outside a certain number of standard deviations from the mean.
## ML Methods
- [[Clustering Algorithms]]
- [[Autoencoder]]
## Time Series Analysis
- [[Seasonal Decomposition]]
- [[Recurrent Neural Network (RNN)]]
- [[Autoregressive Integrated Moving Average (ARIMA)]]
- [[Exponential Smoothing State Space Model (ETS)]]
##  Pattern Recognition
- **Fourier Transform**: Apply Fourier Transform to analyze the frequency domain of the temperature data and identify unusual frequency components indicating anomalies.
- **[[Wavelet Transform]]**: Use wavelet transforms to detect sudden changes in the temperature data at different scales.
## Change Point Detection
- [[Cumulative Sum (CUSUM) for Detecting Shifts in Mean Level]]
- [[Bayesian Change Point Detection]]
## Threshold-Based Methods
- **Fixed Thresholds**: Define fixed upper and lower thresholds based on historical data or domain knowledge, and flag readings outside these thresholds as anomalies.
- **Dynamic Thresholds**: Use adaptive thresholds that change over time based on the behavior of the temperature data.

## Energy-Based Methods
- **[[Thermal Energy Analysis]]**: Calculate the thermal energy changes over time to detect sudden spikes or drops that could indicate anomalies.


## Panoptic Segmentation Integration
- Apply anomaly detection methods within the context of each object type. For example, temperature anomalies on a human should be treated differently from those on machinery.
## Time-Based Anomaly Detection
- Use time-series models to capture temporal dependencies (e.g., ARIMA, LSTM).
- Incorporate time-of-day and seasonal variations into the models.
- Decompose the temperature data into trend, seasonal, and residual components.
- Analyze residuals for anomalies considering time-based trends.
## Combining Both Approaches
- Apply multivariate anomaly detection techniques (e.g., Multivariate Gaussian Distribution, Isolation Forest) to account for multiple factors (temperature, object type, time).
## Other important information
- Use multivariate anomaly detection techniques that can handle multiple variables simultaneously.
### **Environmental Conditions**
- **Ambient Temperature and Humidity**: These can affect the thermal readings and should be accounted for to differentiate between actual anomalies and environmental influences.
- **Wind Speed and Direction**: These can impact the surface temperature of objects, especially in outdoor settings.
### **Object Metadata**
- **Material Properties**: Different materials have different thermal properties. Knowing the material of objects can help in setting appropriate anomaly detection thresholds.
- **Object State**: Information on whether an object is active or inactive (e.g., machinery running or stopped) can influence expected temperature ranges.
### **Location and Spatial Context**
- **Geographic Location**: Outdoor environments can have different thermal behaviors depending on the geographic location.
- **Proximity to Heat Sources**: Objects near known heat sources (e.g., heaters, sunlight exposure) can have naturally higher temperatures.
### **Temporal Context**
- **Seasonal Variations**: Temperature behavior can vary significantly with seasons. Accounting for these variations can help avoid false positives.
- **Daily Cycles**: Diurnal patterns, such as day and night cycles, can influence temperature readings.
### **Thermal Gradients and Patterns**
- **Thermal Gradients**: Sudden temperature gradients across an object can indicate anomalies like overheating.
- **Pattern Recognition**: Recognizing thermal patterns (e.g., regular heat dissipation) can help in identifying deviations.