Bayesian Change Point Detection is a statistical method used to identify points in time where the statistical properties of a time series change significantly. It leverages Bayesian inference to determine the probability of a change point occurring at each time step.

#### Key Concepts

1. **Change Point**
   - A point in time where the statistical properties of the data (e.g., mean, variance) change.
   - Identifying change points helps in understanding shifts in the behavior of the time series.

2. **Bayesian Inference**
   - Bayesian methods use prior knowledge and observed data to update the probability of hypotheses.
   - In change point detection, it calculates the posterior probability of a change point at each time step.

3. **Modeling**
   - The data is modeled using different statistical properties before and after each potential change point.
   - The likelihood of the data given a change point is calculated and used to update the posterior probabilities.

#### Steps to Apply Bayesian Change Point Detection

1. **Data Preparation**
   - Collect n temperature measurements $T_1, T_2, \ldots, T_n$.
   - Ensure the data is in a time series format with consistent time steps.

2. **Choose Prior and Likelihood Models**
   - Select appropriate prior distributions and likelihood models for the temperature data.

3. **Apply Bayesian Change Point Detection**
   - Use Bayesian methods to calculate the posterior probabilities of change points.
   - Example in Python using `ruptures` library:
     ```python
     import numpy as np
     import ruptures as rpt

     # Example temperature data (replace with actual data)
     temperature_data = np.random.normal(20, 5, 1000)

     # Apply Bayesian change point detection
     model = "l2"  # Model for the residuals
     algo = rpt.Pelt(model=model).fit(temperature_data)
     result = algo.predict(pen=10)

     # Plot results
     rpt.display(temperature_data, result)
     plt.show()
     ```

4. **Analyze Results**
   - The change points identified by the algorithm indicate significant shifts in the statistical properties of the temperature data.

#### Example Application
Suppose you have hourly temperature measurements for an object over a month (720 hours).

1. **Data Preparation**: Ensure the data is hourly and covers the entire period.
2. **Choose Prior and Likelihood Models**: Select appropriate models based on the data characteristics.
3. **Apply Bayesian Change Point Detection**:
   ```python
   import numpy as np
   import ruptures as rpt

   # Example temperature data (replace with actual data)
   temperature_data = np.random.normal(20, 5, 720)

   # Apply Bayesian change point detection
   model = "l2"  # Model for the residuals
   algo = rpt.Pelt(model=model).fit(temperature_data)
   result = algo.predict(pen=10)

   # Plot results
   rpt.display(temperature_data, result)
   plt.show()
   ```

4. **Analyze Results**: Identify significant shifts in the statistical properties indicated by the change points.

