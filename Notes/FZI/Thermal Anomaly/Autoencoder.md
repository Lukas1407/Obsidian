**Explanation**:
   - An autoencoder is a type of neural network used to learn efficient codings of data. It consists of an encoder to compress data and a decoder to reconstruct it.
   - When trained on normal data, the autoencoder will have low reconstruction error for normal data and high error for anomalies.

**Application**:
   - Train the autoencoder on normal temperature data.
   - Use the trained model to reconstruct the temperature data.
   - Calculate the reconstruction error for each measurement.
   - Define a threshold for reconstruction error. Measurements with errors above this threshold are anomalies.

     ```python
     from keras.models import Model, Sequential
     from keras.layers import Dense, Input

     input_dim = temperatures.shape[1]
     encoding_dim = 32

     autoencoder = Sequential()
     autoencoder.add(Dense(encoding_dim, activation='relu', input_shape=(input_dim,)))
     autoencoder.add(Dense(input_dim, activation='sigmoid'))

     autoencoder.compile(optimizer='adam', loss='mean_squared_error')
     autoencoder.fit(temperatures, temperatures, epochs=50, batch_size=256, shuffle=True)

     reconstructions = autoencoder.predict(temperatures)
     reconstruction_errors = np.mean(np.power(temperatures - reconstructions, 2), axis=1)
     anomalies = np.where(reconstruction_errors > threshold)  # Define your threshold
     ```