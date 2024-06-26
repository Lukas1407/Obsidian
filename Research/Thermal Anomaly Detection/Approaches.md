
## knowledge-based
### threshold-based approach
This approach involves setting predefined limits or thresholds for temperature measurements. When these measurements exceed the set thresholds, it indicates a potential anomaly or fault condition, such as overheating or thermal runaway.
Here’s a simplified explanation of how it works:
1. **Establish Thresholds**: Based on historical data, lab tests, and engineering knowledge, specific temperature limits are set for the battery or system being monitored.
2. **Continuous Monitoring**: The system continuously measures the temperature of the battery or component in real-time.
3. **Anomaly Detection**: If the temperature readings cross the predefined thresholds, it triggers an alert indicating a possible thermal anomaly.
4. **Response and Mitigation**: Upon detection, the system can take corrective actions to mitigate the risk, such as cooling down the battery or shutting down the system to prevent damage.
The threshold-based method is straightforward and easy to implement. However, it has limitations, such as being less effective in adapting to varying operating conditions or changes due to battery aging. It may also result in false positives or negatives if the thresholds are not accurately set or updated over time.
## model-based
involve creating a mathematical or computational model that represents the normal operating conditions of a system, such as a battery pack. These models typically use real-time data, like voltage and temperature readings from battery cells, to predict what the normal behavior should be.
When the actual data deviates significantly from the model’s predictions, it can indicate an anomaly. For instance, if the temperature readings from a battery cell are higher than what the model predicts for the current operating conditions, it could suggest a thermal anomaly, which might be a precursor to issues like thermal runaway.
## data-driven
Data-driven methods rely on the statistical relationships between distributed measurements