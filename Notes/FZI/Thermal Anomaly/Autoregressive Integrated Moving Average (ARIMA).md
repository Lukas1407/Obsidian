- Model the temperature data using ARIMA to predict future values and detect anomalies when actual values deviate significantly from predictions.

ARIMA is a popular statistical method used for time series forecasting and anomaly detection. It captures different aspects of the data's time series structure through three components: autoregression (AR), differencing (I), and moving average (MA). Here’s a detailed explanation of each component and how they come together to model n temperature measurements.
#### Components of ARIMA
1. **Autoregression (AR)**
   - **Definition**: Autoregression uses the dependent relationship between an observation and a number of lagged observations (previous time steps).
   - **Notation**: AR(p) indicates that the model uses p lagged observations.
   - **Example**: For temperature $T$ at time $t$, an AR(1) model would be:
     $$
     T_t = \phi_1 T_{t-1} + \epsilon_t
     $$
     where $\phi_1$ is the parameter of the model, and $\epsilon_t$ is white noise.

2. **Integration (I)**
   - **Definition**: Integration involves differencing the observations (subtracting an observation from the previous one) to make the time series stationary, meaning its statistical properties (mean, variance) do not change over time.
   - **Notation**: I(d) indicates that the data have been differenced d times.
   - **Example**: If we difference the temperature series once (d=1), the transformed series becomes:
     $$
     T'_t = T_t - T_{t-1}
     $$

3. **Moving Average (MA)**
   - **Definition**: Moving average uses the dependency between an observation and a residual error from a moving average model applied to lagged observations.
   - **Notation**: MA(q) indicates that the model uses q lagged forecast errors.
   - **Example**: For temperature $T$ at time $t$, an MA(1) model would be:
     $$
     T_t = \epsilon_t + \theta_1 \epsilon_{t-1}
     $$
     where $\theta_1$ is the parameter of the model.

#### The ARIMA Model
- **Notation**: ARIMA(p, d, q)
  - **p**: Number of lag observations in the model (AR part).
  - **d**: Number of times that the raw observations are differenced to make the series stationary (I part).
  - **q**: Size of the moving average window (MA part).

The general form of an ARIMA model is:
$$
T'_t = c + \phi_1 T'_{t-1} + \phi_2 T'_{t-2} + \cdots + \phi_p T'_{t-p} + \theta_1 \epsilon_{t-1} + \theta_2 \epsilon_{t-2} + \cdots + \theta_q \epsilon_{t-q} + \epsilon_t
$$
where $T'_t$ represents the differenced temperature series, $c$ is a constant, and $\epsilon_t$ is white noise.

### Steps to Apply ARIMA to Temperature Measurements

1. **Data Preparation**
   - Collect n temperature measurements $T_1, T_2, \ldots, T_n$.
   - Ensure the data is in a time series format with consistent time steps.

2. **Stationarity Check**
   - Plot the time series and use statistical tests (e.g., Augmented Dickey-Fuller test) to check for stationarity.
   - If the series is not stationary, apply differencing until it becomes stationary. Record the number of differencing steps as $d$.

3. **Identify p and q**
   - Use Autocorrelation Function (ACF) and Partial Autocorrelation Function (PACF) plots to determine suitable values for p (lag for AR) and q (lag for MA).
   - ACF plot helps identify the q value by showing how the data is correlated with its past values.
   - PACF plot helps identify the p value by showing the partial correlation of the series with its own lagged values, removing the effect of intermediate lags.

4. **Fit the ARIMA Model**
   - Use a statistical software package (e.g., Python’s `statsmodels` library) to fit the ARIMA(p, d, q) model to the temperature data.
   - Example in Python:
     ```python
     from statsmodels.tsa.arima.model import ARIMA
     model = ARIMA(temperature_data, order=(p, d, q))
     model_fit = model.fit()
     print(model_fit.summary())
     ```

5. **Model Diagnostics**
   - Check the residuals of the model to ensure they behave like white noise (i.e., they are normally distributed with a mean of zero and no autocorrelation).
   - Use diagnostic plots and statistical tests to validate the model.

6. **Forecasting and Anomaly Detection**
   - Use the fitted model to forecast future temperature values.
   - Compare actual measurements with the forecasted values. Significant deviations from the forecast may indicate thermal anomalies.

### Example Application
Suppose you have temperature measurements for an object taken every hour for 100 hours, and you want to use ARIMA to detect anomalies.

1. **Plot the time series and check stationarity**: The plot shows an upward trend, indicating non-stationarity.
2. **Difference the data**: Apply first-order differencing (d=1) and recheck stationarity.
3. **Identify p and q using ACF and PACF plots**: Suppose ACF suggests q=1 and PACF suggests p=1.
4. **Fit the ARIMA(1, 1, 1) model**: 
   ```python
   from statsmodels.tsa.arima.model import ARIMA
   model = ARIMA(temperature_data, order=(1, 1, 1))
   model_fit = model.fit()
   print(model_fit.summary())
   ```
5. **Model diagnostics**: Ensure residuals are white noise.
6. **Forecast and detect anomalies**: Use the model to forecast future temperatures and identify measurements that deviate significantly from the forecast.

By following these steps, ARIMA helps in understanding and predicting temperature patterns, thereby facilitating the detection of thermal anomalies.

## Paper
The Autoregressive Integrated Moving Average (ARIMA) model is a widely used statistical technique for time series forecasting and analysis. ARIMA models are particularly effective for understanding and predicting future points in a series based on past observations. The model is composed of three key components: Autoregression (AR), Integration (I), and Moving Average (MA). The autoregressive part involves regressing the variable on its own lagged values, the integrated part involves differencing the observations to make the time series stationary, and the moving average part involves modeling the error term as a linear combination of past error terms. 
Mathematically, an ARIMA model is expressed as ARIMA(p,d,q), where $p$ is the number of lag observations included in the model (autoregressive part), $d$ is the number of times that the raw observations are differenced (integrated part), and $q$ is the size of the moving average window.
In the context of calculating thermal anomalies based on temperature measurements, using an ARIMA model with a threshold of 3.0 is highly appropriate. Temperature data is typically time-dependent, exhibiting trends and seasonal patterns. ARIMA's ability to model these characteristics makes it well-suited for forecasting and anomaly detection in such data. By setting a threshold of 3.0, we align with the z-score methodology, where values that deviate more than three standard deviations from the predicted value are considered anomalies. This threshold is robust and statistically significant, ensuring that only substantial deviations are flagged. 
ARIMA models provide a comprehensive framework for capturing the temporal dependencies in the data, offering accurate predictions of future temperatures. When these predictions significantly differ from actual observations, it indicates potential anomalies. This method is particularly useful in scenarios where historical data patterns are complex, as ARIMA can adapt to various trends and seasonal variations, providing a more accurate and reliable method for detecting thermal anomalies. Using ARIMA helps in identifying not just the anomalies but also in understanding the underlying patterns in the temperature data, leading to more informed decisions and insights.

