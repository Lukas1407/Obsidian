**Explanation**:
   - RNNs, especially LSTMs, are well-suited for time-series data as they can model temporal dependencies.
   - They can predict future values based on past sequences.

**Application**:
   - Train an RNN or LSTM on the temperature data to predict future temperatures.
   - Compare actual temperatures with predicted values.
   - Define a threshold for prediction errors. Large deviations between actual and predicted values are considered anomalies.
     ```python
     from keras.models import Sequential
     from keras.layers import LSTM, Dense

     model = Sequential()
     model.add(LSTM(50, activation='relu', input_shape=(n_steps, n_features)))
     model.add(Dense(1))
     model.compile(optimizer='adam', loss='mse')

     model.fit(X, y, epochs=200, verbose=0)
     predictions = model.predict(X)
     prediction_errors = y - predictions
     anomalies = np.where(np.abs(prediction_errors) > threshold)  # Define your threshold
     ```