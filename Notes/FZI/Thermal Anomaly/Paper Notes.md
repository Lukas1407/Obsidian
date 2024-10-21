## Useful phrases
- Combining temporal and spatial aggregations
### About the Camera
www.flircameras.com
The FLIR Tau 2 thermal camera is an advanced longwave infrared (LWIR) thermal imager designed for high-performance applications, including unmanned vehicles, thermal weapon sights, and handheld imagers. The Tau 2 series includes models with different resolutions, including the Tau 2 640, which offers a resolution of 640x512 pixels using a VOx Microbolometer sensor with a pixel size of 17 µm. This camera operates within the spectral band of 7.5 - 13.5 µm, providing detailed thermal imaging capabilities with a noise equivalent temperature difference (NETD) of less than 50 mK at f/1.0, ensuring high sensitivity and accurate thermal detection​.
The Tau 2 is also equipped with robust environmental specifications, capable of operating in temperatures ranging from -40°C to +80°C and enduring shock pulses of 200 g with an 11 ms sawtooth profile. The camera's compact design, measuring 1.75 x 1.75 x 1.75 inches, and low power consumption of less than 1.2 W for the Tau 2 640 model, make it suitable for integration into various platforms.
-> Ideal for robot application

## A simple method based on the thermal anomaly index to detect industrial heat sources
The calculation of the Thermal Anomaly Index (TAI) as described in the provided text utilizes Wien's Law of Displacement and Planck's radiation function to differentiate thermal anomalies from normal areas. 
### Wien's Law of Displacement:
   - As the temperature of an object increases, the peak wavelength of its radiant emission shifts to shorter wavelengths. This means that hotter objects emit more radiation at shorter wavelengths compared to cooler objects.
### Brightness Temperature Comparison:
   - Brightness temperature is used to compare different areas. For thermal anomalies, the brightness temperature increases significantly in shorter wavelengths compared to normal areas.
   - By analyzing the brightness temperatures of various industrial areas like coking plants and steel plants, which operate at high temperatures, a pattern of irregular brightness temperature curves can be observed. In contrast, normal areas such as residential or vegetation areas show a more linear increase in brightness temperature with wavelength.
### Radiance Rescaling:
   - Digital Number (DN) values from the thermal infrared (TIR) bands are converted to spectral radiance using the radiance rescaling factors provided in the metadata.
   - Thermal infrared bands are specific segments of the infrared spectrum, each corresponding to a particular range of wavelengths. These bands are designed to capture radiance emitted from objects based on their thermal properties.
   - The conversion formula is:
     $$
     B_{\lambda} = M_L \times DN + A_L
     $$
     where $B_{\lambda}$ is the spectral radiance at wavelength $\lambda$, $M_L$ and $A_L$ are rescaling factors, and $DN$ is the quantized pixel value.
### Brightness Temperature Calculation:
- Brightness temperature or radiance temperature is not the actual physical temperature of the object but rather a measure of the radiative power emitted by the object at a specific wavelength. (https://en.wikipedia.org/wiki/Brightness_temperature)
- It is derived from the observed radiance and relates it to the temperature of an ideal black body (a perfect emitter) that would emit the same radiance. (https://en.wikipedia.org/wiki/Brightness_temperature)
   - Using Planck’s radiation function, the spectral radiance is converted to brightness temperature:
     $$
     B_{\lambda}(T) = \frac{C_1}{\lambda^{5}[exp(\frac{C_{2}}{\lambda T})-1]}
     $$
     where $C_1 = 1.191 \times 10^8 \text{ W} \cdot \mu\text{m}^{-4} \cdot \text{Sr}^{-1} \cdot \text{m}^{-2}$ and $C_2 = 1.439 \times 10^4 \mu\text{m} \cdot \text{K}$.
### Thermal Anomaly Index (TAI):
   - The TAI is calculated to emphasize the difference in brightness temperatures between short and long TIR bands:
     $$
     TAI = 100 \times \left( \frac{\max(T_{10}, T_{11}, T_{12}) - \max(T_{13} + T_{14})}{\max(T_{10}, T_{11}, T_{12}) + \max(T_{13}, T_{14})} \right)
     $$
     where $T_{10}, T_{11}, T_{12}, T_{13}, T_{14}$ are the brightness temperatures of the respective bands.

### Classifying Thermal Anomalies:
   - To classify an area as a thermal anomaly, the TAI and the brightness temperature in band 13 ($T_{13}$) are compared to thresholds.
   - The condition used is:
     $$
     TAI \gg T_0 + \text{mean}(T_{13})
     $$
     where $T_0$ is a threshold value (set as 1 K in the paper), and $\text{mean}(T_{13})$ is the average brightness temperature in band 13. Band 13 is chosen due to its higher atmospheric transmittance, making it more reliable for land surface temperature retrieval.
By using this method, thermal anomalies such as areas with extreme temperature increases (e.g., due to industrial activity or biomass burning) can be identified effectively. The TAI provides a quantitative measure that highlights significant differences in thermal emission, which can be used for various applications like monitoring industrial processes or detecting wildfires.
## Concept of Computer Vision Based Algorithm for Detecting Thermal Anomalies in Reinforced Concrete Structures
- automated computer-vision-based method for detecting thermal anomalies
The CV-based algorithm for detecting thermal anomalies involves a series of stages designed to automate the identification of regions with significant temperature deviations. Here's a detailed breakdown of each stage:
### 1. Initial Model Calibration
- **Thermal Image Processing**:
  - Thermal images are processed into two-dimensional matrices where each pixel corresponds to a temperature value.
  - A histogram of these temperature values is created with an initial bin width (BW).
- **Histogram Analysis**:
  - The histogram is analyzed to identify tendencies towards colder or hotter regions, influenced by the thermal properties of materials and seasonal heat transfer.
  - An Anomaly Bin (AB) and an Opposite Bin (OB) are defined based on the temperature tendency. For instance, in a cold scenario, AB would be the coldest bin, and OB the hottest, and vice versa for a hot scenario.
  - An Anomaly Threshold (AT) is set to mark temperatures that enter the AB.
### 2. Dynamic Calibration
- **Adjustment of Parameters**:
  - **Significance Bound**:
    - If the number of pixels in OB is less than a lower bound (α), these data points are considered irrelevant and filtered out.
    - The AT is then shifted by BW, and the neighboring bin becomes the new OB. This process repeats until OB has enough significant pixels (OB ≥ α).
  - **Peak Analysis**:
    - Multiple peaks in the histogram suggest irrelevant data (like the background sky). Mid-range temperatures are likely to contain relevant information.
    - Peaks and their neighboring pixels with less than 10% of the total pixels are filtered out.
  - **Bin and BW Adjustment**:
    - If the number of pixels in AB exceeds an upper bound (β), the number of bins (n) and BW are adjusted. The process repeats until AB has a manageable number of significant pixels (AB ≤ β).
- **Outcome of Dynamic Calibration**:
  - This calibration reduces the data to relevant thermal information, filtering out noise and irrelevant pixels.
### 3. Identification of Thermal Edges
- **Edge Detection**:
  - The Canny edge detector is used to identify thermal edges in the calibrated image.
  - This step highlights areas with significant temperature changes.
- **Filtering Irrelevant Edges**:
  - Otsu Thresholding is applied to a gray-level histogram to filter out irrelevant thermal edges.
  - This process retains edges that define significant thermal boundaries, such as the contour of structures and areas with sharp temperature transitions.
### 4. Leakage Segmentation
- **Defining Neighborhood Relationships**:
  - A neighborhood relationship is established to define the domain around each pixel, considering top, bottom, left, and right neighbors.
  - A Neighbors Size (NS) parameter is set to determine the extent of this neighborhood.
- **Identifying Leakage Regions**:
  - Data around thermal edges is filtered based on the adjusted AT.
  - This step aims to detect thermal anomalies indicative of issues such as water leakage.
- **Result**:
  - The algorithm successfully identifies thermal anomalies related to leakage by analyzing temperature deviations around identified thermal edges.
### Summary
The CV-based algorithm effectively identifies thermal anomalies through a systematic approach that includes initial calibration, dynamic adjustment of parameters, edge detection, and segmentation of leakage regions. The process involves filtering out irrelevant data and focusing on significant temperature changes to pinpoint areas of interest, such as water leaks in structures. This methodology ensures accurate and automated detection of thermal anomalies in various scenarios.
## Fusion of thermal imagery with point clouds for building façade thermal attribute mapping
- Annotates point cloud with temperature
The paper describes a comprehensive methodology for annotating a 3D point cloud with thermal image data, focusing on building façade inspections. The process involves several key steps, each addressing specific challenges related to the integration of thermal and RGB data.
1. **Point Cloud Generation**:
   - Both thermal and RGB point clouds are generated using a Structure-from-Motion (SfM) tool (Agisoft PhotoScan®). Due to the lower resolution and contrast of thermal images compared to RGB images, the thermal point cloud is noisier and sparser.
   - To improve the thermal image contrast and increase the number of tie points, a Wallis filter is applied before generating the thermal point cloud. The overlap requirements for thermal images are higher (80% horizontal and 60% vertical) compared to RGB images (60% horizontal and 40% vertical) to ensure sufficient data for point cloud generation.
2. **Point Cloud Registration**:
   - Initial coarse registration is achieved using the Fast Point Feature Histogram (FPFH) for feature extraction and Fast Global Registration (FGR) for alignment. FGR is preferred over RANSAC due to its robustness against noise and better handling of heterogeneous point clouds.
   - Further refinement of the registration is conducted using the Iterative Closest Point (ICP) algorithm, although ICP has limited success due to the varying densities and accuracies of the point clouds.
3. **Thermal-RGB Image Matching**:
   - After coarse registration, a fine registration process is performed using image matching techniques. The Radiation-Invariant Feature Transform (RIFT) is employed for feature detection, as it handles large nonlinear radiation differences better than traditional methods like SIFT or SURF.
   - Matched features between thermal and RGB images are identified using a normalized barycentric coordinate system (NBCS) and RANSAC, improving the inlier ratio and ensuring reliable correspondences.
4. **Image Pose Computation**:
   - For each thermal-RGB image pair, a two-step workflow is used: mono-plotting of RGB images to obtain 3D object points, followed by spatial resection of thermal images to refine their exterior orientation parameters.
   - The spatial subdivision of the façade point cloud ensures efficient processing by focusing on relevant sections for each image.
5. **Texture Mapping**:
   - The texture mapping process involves assigning thermal attributes to the 3D point cloud, with a global image pose refinement approach to minimize temperature discrepancies among overlapping images.
   - This refinement is crucial to achieve consistent and accurate thermal texture mapping, ensuring high geometric and radiometric accuracy.
The entire workflow is designed to overcome the limitations of thermal imagery, such as lower resolution and contrast, and to integrate these images effectively with high-resolution RGB point clouds for detailed building inspections
## Employing Robotics and Deep Learning in Underground Leak Detection
- describes the methodology for calculating thermal anomalies through a comprehensive approach involving robotic systems and deep learning. Here is a detailed explanation based on the information from the document:
### Critical Values Detection and Thermal Profiles
1. **Thermal Profiling and Data Collection**:
   - A low-cost MLX90614 thermopile infrared sensor is utilized to trace surface temperatures, detecting anomalies caused by leaks. The sensor is mounted on both manned and unmanned ground vehicles (UGVs), and positioning of temperature readings is done using GPS and odometry.
   - The temperature data is collected continuously, although GPS signal loss can occur, necessitating the use of a rotary encoder for accurate displacement measurement between readings【36:1†source】【36:3†source】.
2. **Statistical Analysis**:
   - **Critical Values Detection**: The robot surveys long distances, and an algorithm is used to automate the detection of thermal anomalies by identifying critical values in the surface temperature vs. distance graph. Critical values, which indicate potential leaks, are detected by finding points where the first derivative of the temperature changes sign.
   - A derivative filter is applied to the thermal profiles, with the first difference (T[n] - T[n-1]) being used to locate potential anomalies. Locations where the first difference exceeds a preset threshold are identified as possible leak points. For scenarios where the threshold is not met, a 2D convolutional neural network (CNN) is employed to compare the thermal profile shapes around critical values within defined windows【36:1†source】【36:2†source】.
### Convolutional Neural Network (CNN) and Image Processing
3. **Training the CNN**:
   - The training dataset for the CNN is built using a finite element method (FEMM) thermal model of underground leaks. The thermal distribution model incorporates various sources of thermal disturbances, including passive objects (e.g., concrete, sandstones) and active sources (e.g., leaking water, burning wood).
   - The model generates temperature readings which are processed into 2D single-channel images (30x30 pixels) for CNN input. This approach balances the number of true and false detections by duplicating the images of true detections【36:2†source】【36:5†source】【36:11†source】.
4. **Thermal Anomalies Detection**:
   - The CNN is trained over multiple epochs, with the dataset shuffled before each epoch to prevent overfitting. The CNN uses thermal profiles converted into 8-bit images to classify anomalies with an accuracy of 77.7%, despite an overfitting tendency observed during training.
   - The process includes dividing thermal profiles around critical values into image segments, allowing the CNN to learn and identify patterns indicative of thermal anomalies【36:5†source】.
## Anomaly Detection in Thermal Images Using Deep Neural Networks
The paper "Anomaly Detection in Thermal Images Using Deep Neural Networks" by Lile and Yiqun calculates thermal anomalies using a deep convolutional neural network (CNN) approach. The detailed steps for this process are as follows:
1. **Model Architecture and Training**:
    - The CNN model is inspired by architectures used for saliency prediction and is built on the VGG-16 network, which has been pretrained on the ImageNet dataset for object recognition.
    - Only the first three stages of the VGG-16 are used, and pooling layers after stage 3 are excluded to avoid unnecessary complexity.
    - A batch normalization layer is added to normalize feature values to zero mean and unit variance, aiding in the network's convergence.
    - The network includes upscaling layers to ensure the output map has the same size as the input image, followed by convolutional layers to refine predictions.
    - The model uses the mean squared error (MSE) as the objective function and employs stochastic gradient descent for optimization​(Lile and Yiqun - 2017 -…)​.
2. **Training Process**:
    - The model is trained on visible images (input) and their corresponding thermal images (output).
    - The training data consists of various categories of electrical equipment under normal operation.
    - During training, the model learns to predict thermal images from visible images by minimizing the average prediction error (MSE)​(Lile and Yiqun - 2017 -…)​.
3. **Anomaly Detection Pipeline**:
    - After training, the model predicts the normal thermal profile for an input visible image. If the equipment in the input image operates normally, the difference between the predicted and actual thermal images is minimal.
    - To detect anomalies, the predicted thermal image is used as a reference and compared to the actual thermal image.
    - The absolute difference between the predicted and actual thermal images (Delta-T values) is computed. Regions with Delta-T values exceeding a pre-specified threshold are identified as anomalies.
    - These anomalies are highlighted in red on the visible image to alert users​(Lile and Yiqun - 2017 -…)​.
## Thermal anomaly detection in walls via CNN-based segmentation
The paper by Park et al. (2021) presents a method for detecting thermal anomalies in building envelopes using a combination of thermal and visible images, along with deep learning techniques for image segmentation. The process for calculating thermal anomalies involves several key steps:
### 1. Target Domain Recognition
The first step is to recognize the target domains using image segmentation. The visible image is processed using a CNN model to segment the wall areas. This segmentation result, a mask matrix, is then resized to match the resolution of the thermal image using bilinear interpolation. The thermal image, represented as an \( n \times m \) matrix where each element denotes the temperature of a pixel, is then compared with the resized mask matrix to extract the thermal data of the wall. Null values are assigned to elements not in the wall domain to focus the analysis on relevant areas. 
### 2. Temperature Density Estimation and Anomaly Threshold Detection
The thermal anomaly detection algorithm is adapted from a method proposed by Asdrubali et al., modified for field-acquired thermal images which often exhibit multimodal temperature distributions due to varied materials and structures within the image. The kernel density estimation method is used to estimate the probability distribution of the temperatures in the target domain. A bandwidth of 0.1°C is used to detect extreme values sensitively.
When multiple local maxima are present in the temperature distribution, the two largest maxima are identified, and a minimum peak ratio (5% of the global maximum) is defined to filter out insignificant peaks. The temperature with the lowest probability between the two largest peaks is determined as the thermal anomaly threshold. Areas exceeding this threshold in winter conditions are considered anomalies.
### 3. Algorithm and Practical Application
The algorithm is applied through several stages including initial model calibration, dynamic calibration, and identification of thermal edges. The dynamic calibration involves adjusting the anomaly threshold and other parameters to filter out irrelevant data and refine the detection process. This is followed by edge detection using methods like the Canny edge detector and Otsu's Thresholding Method to isolate significant thermal edges indicative of anomalies.
### 4. Evaluation and Results
The effectiveness of the proposed framework is quantitatively evaluated using metrics such as Precision, Recall, and F₁ score, comparing results from entire thermal image analysis versus segmented wall analysis. The segmentation significantly improves detection accuracy by focusing on relevant areas and avoiding false positives from non-wall regions. 
The proposed method effectively combines deep learning for image segmentation with statistical methods for temperature distribution analysis, providing a robust approach for detecting thermal anomalies in complex field conditions.
## UAV-Based Thermal Anomaly Detection for Distributed Heating Networks
### 1. Data Acquisition
**Thermal Imaging**:
- The UAV equipped with a DJI Zenmuse XT2 camera, which has both thermal and optical sensors, captures thermal images of the district heating system (DHS).
- The thermal camera has a resolution of 640x512 pixels, and the images are captured at a flying height of 40 meters, resulting in a ground resolution of 5.2 cm for thermal images.
**Photogrammetric Processing**:
- The captured thermal images are processed using rigorous photogrammetric techniques to generate a thermal orthomosaic, which is an accurate, georeferenced image of the thermal data over the area of interest.
### 2. Preprocessing and Orthomosaic Generation
**Georeferencing**:
- The thermal orthomosaic is aligned with the geographic information system (GIS) data to limit the search space to areas around the DHS pipelines. This involves rasterizing the GIS data to the same ground resolution as the orthomosaic, creating a binary raster that indicates pipeline locations.
### 3. Anomaly Detection
**Hot Spot Detection**:
- **Blob Detection**: The primary method for detecting thermal anomalies involves using the Laplacian of Gaussian (LoG) blob detector in scale space. This detector identifies regions (blobs) in the thermal orthomosaic that are significantly hotter than their surroundings.
- Each detected blob is represented by its center coordinates (xc, yc) and scale (σ), which corresponds to the size of the blob.
**Elliptical Blob Representation**:
- The detected blobs are further processed to determine their major and minor axes by fitting an ellipse to the blob area. This is achieved using the eigenvalues and eigenvectors of the weighted Hessian matrix of the smoothed image.
### 4. Clustering and Segmentation
**Clustering of Temperature Values**:
- The temperature values within the area of each detected blob are clustered using the k-means algorithm with k = 3. This step divides the blob area into three segments, characterized by their mean temperatures (tlow, tmid, thigh).
- The purpose of using three clusters is to ensure a better separation of higher temperature values, making it easier to identify the hottest regions within each blob.
**Cluster Merging**:
- To address the issue of potential misclassification of pixels, a fusion process is applied. If the mean temperature of the middle cluster (tmid) is close to the highest cluster (thigh) and far from the lowest cluster (tlow), the middle and highest clusters are merged.
### 5. Final Anomaly Decision
**Threshold Comparison**:
- The final decision on whether a detected hot spot is an anomaly is made by comparing the temperature difference between the hottest region (thigh) and its surroundings (either tlow or tmid) to a predefined threshold (tanomaly).
- A hot spot is considered an anomaly if this temperature difference exceeds the threshold.
### 6. Post-Processing and False Alarm Reduction
**Filtering False Alarms**:
- The system identifies and filters out common false alarms such as heat sources related to buildings, street lights, cars, and other manmade objects. This filtering is aided by comparing detected anomalies with GIS building layers and a normalized digital surface model (DSM).
- Moving objects, such as cars and people, are also identified and excluded by comparing images from flights conducted on different days.
## Performance of signal processing techniques for anomaly detection using a temperature‑based measurement interpretation approach
The paper by Kromanis and Kripakaran (2021) presents a method for detecting thermal anomalies in structures using signal processing techniques on thermal image data. Here is a detailed explanation of the process described in the paper:
1. **Thermal Image Acquisition**: Thermal images of the target structure are acquired using a thermal camera. These images provide a two-dimensional representation of the surface temperature distribution of the structure.
2. **Image Preprocessing**: The acquired thermal images are preprocessed to enhance the quality of the data and reduce noise. This includes techniques such as filtering and normalization to ensure the thermal data is consistent and reliable for further analysis.
3. **Feature Extraction**: From the preprocessed thermal images, specific features are extracted that are indicative of thermal anomalies. These features include temperature gradients, spatial distribution patterns, and temporal changes in temperature. The feature extraction process aims to highlight areas in the thermal images that deviate from the expected thermal behavior of the structure.
4. **Anomaly Detection**: The core of the thermal anomaly detection algorithm involves analyzing the extracted features using signal processing techniques. The following steps are involved:
    - **Temperature Gradient Analysis**: The temperature gradient across the surface of the structure is calculated. Significant deviations from the average gradient may indicate the presence of an anomaly.
    - **Statistical Analysis**: Statistical measures such as mean, standard deviation, and variance are computed for the temperature data. Anomalies are identified by comparing these statistics against predefined thresholds.
    - **Thresholding**: Based on the statistical analysis, threshold values are set to classify regions of the thermal images as normal or anomalous. Regions with temperature values or gradients exceeding these thresholds are flagged as potential anomalies.
5. **Validation and Verification**: The identified anomalies are validated by cross-referencing with known structural features or by conducting additional inspections. This step ensures that the detected anomalies are not false positives caused by environmental factors or camera artifacts.
6. **Visualization**: The results of the anomaly detection process are visualized to provide a clear and intuitive representation of the thermal anomalies. This includes overlaying the detected anomalies on the original thermal images and generating heat maps to highlight areas of concern.
7. **Integration with Structural Monitoring Systems**: The detected thermal anomalies are integrated into a broader structural health monitoring system. This allows for continuous monitoring and assessment of the structure's condition, enabling timely maintenance and repair actions to be taken.
This comprehensive approach to thermal anomaly detection leverages advanced signal processing techniques to enhance the accuracy and reliability of structural health assessments based on thermal imaging data. The method provides a robust framework for identifying and addressing thermal anomalies, thereby contributing to the overall safety and longevity of structures.
