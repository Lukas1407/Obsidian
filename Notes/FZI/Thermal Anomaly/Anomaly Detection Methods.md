## Statistical Methods
- [[Z-Score Analysis]]
- [[Moving Average]]
## ML Methods
- [[Clustering Algorithms]]
- [[Autoencoder]]
## Time Series Analysis
- [[Seasonal Decomposition]]
- [[Recurrent Neural Network (RNN)]]

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