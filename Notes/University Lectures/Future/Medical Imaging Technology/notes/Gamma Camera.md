- The detector used for [[Scintigraphy]] and [[Single Photon Emission Computed Tomography (SPECT)|SPECT]]
## Components
![[Pasted image 20240528091020.png#invert|400]]
### Collimator
- Detects/selects the [[Gamma Ray|gamma rays]] coming from a specific direction of the imaged sample
- The collimator is made of heavy metal such as tungsten or lead 
- It consists of circular or hexonaidal septa, lika a "honeycomb 
- Several configurations exist
#### Parallel Hole
- This is the most common type of collimator used for imaging, especially in breast and cardiac imaging
- It consists of a series of parallel holes that are perpendicular to the detector crystals.
- These holes are designed to be focused to infinity, meaning they do not converge or diverge.
- As a result, the image produced is a true representation of the radioactive distribution in the body without any scaling or distortion.
- The size of the collimator determines the field of view (FOV) and, consequently, the size of the object that can be imaged.
![[Pasted image 20240528091337.png|100]]
#### Converging Hole
- The converging collimator is designed with holes that converge toward the body.
- Like the diverging collimator, it also allows for the projection of large structures onto a smaller field of view (FOV) crystal plate.
- However, it has a specific advantage: the ability to **zoom in on specific organs**.
- By converging the holes, the collimator focuses on a specific region of interest within the body.
- In the past, it was commonly used to better visualize small structures, but advancements in technology have led to its replacement by other collimators.
![[Pasted image 20240528091508.png|100]]
#### Diverging Hole
- The diverging collimator has holes that fan out away from the detector crystal towards the body.
- This design allows for the imaging of large structures over a smaller FOV crystal plate.
- The scaling factor varies depending on the anatomy being imaged and the spatial location within the image. There is more scaling at the borders of the image and less towards the center, which can lead to some distortion.
![[Pasted image 20240528091435.png|100]]
#### Pinhole
- Pin-hole collimators are specialized for imaging **very small organs**, such as the thyroid and parathyroid glands.
- These collimators have a **single hole** with interchangeable inserts that come in different aperture sizes (e.g., 3 mm, 4 mm, or 6 mm).
- Key features of pin-hole collimators:
    - **Significant Magnification**: Due to the small aperture, they produce an image with **significant magnification**.
    - **High Spatial Resolution**: The focused pin-hole allows for high spatial resolution, enabling the visualization of fine details.
    - **Geometric Distortion**: However, there is a trade-off. Pin-hole collimators introduce **geometric distortion**, particularly at the edges of the image.
- The distortion arises because the pin-hole captures gamma rays from different angles, leading to non-uniform magnification across the image.
![[Pasted image 20240528091539.png|100]]

### Scintillating Crystal 
- When a $\gamma$-ray strikes the crystal, photons of visible light are produced at a rate proportional to the energy of the $\gamma$ -ray
	- For each gamma ray there are multiple photons because it has higher energy
- It is located just behind the collimator device
#### Requirements 
- Capability of detecting ∼ 106 $\gamma$ photons per second
- Sensitive to specific energy windows (to reject scattered $\gamma$)
### Light Guide
- Is placed between the crystal and the PMT to guide the light to the PMT
![[Pasted image 20240528091902.png|100]]
### Photomultiplier Tube (PMT)
- Creates an electrical signal from the light (photons)#
- The light photons hit the photocathode and release photoelectrons with efficiency ranging between 10%-40%
- Each photoelectron is pulled by an electric field towards a dynode
- The PMT consists of multiple dynodes
- Each accelerated photoelectron that strikes the dynode surface produces several electrons
![[photomultiplier_tubes.webp#invert|500]]
- The total gain $G$ is given by $$G=\delta^{n}$$ with:
	- $\delta$ being the emission factor of the dynode
	- $n$ being the number of dynodes
### ADC
- Converts the analog signal into a digital signal
### Signal Processing
- A problem that arises when the [[Gamma Ray|gamma ray]] hits the crystal is that the produced photons do not travel in a straight line from the incoming [[Gamma Ray|gamma ray]]:
![[Gamma Camera 2024-05-28 09.32.01.excalidraw|200]]
- However we need to know where the original ray came from to correctly visualize the image
- This is done by using a network of photomultiplier tubes (PMTs), we can deduce the point of origin of the gamma ray as follows:
	- **Multiple PMTs**: The emitted light photons from a single scintillation event are detected by multiple PMTs arranged around the crystal.
	- **Sum of PMT Outputs**: The total energy $Z$ of the scintillation event is calculated as the sum of all the outputs from the PMTs.
- To find the position$(x,y)$ where the photon hits the detector, we use the center of mass of the signal. This is done by weighing the signal by the height of the signal, which implies that the signal comes from the middle of the scintillation event. The position is calculated using the following formulas:
	- For the x-coordinate: $$x = \frac{\sum_{i} x_i S_i}{\sum_{i} S_i}$$ 
	- For the y-coordinate: $$y = \frac{\sum_{i} y_i S_i}{\sum_{i} S_i}$$here, 
		- $i$ is the index of each PMP
		- $x_i$ and $y_i$ are the positions of the $i$-th PMT
		- $S_i$ is the integral of the PMT output over the scintillation duration. 
- By calculating the weighted average of the positions of the PMTs based on the intensity of their signals, we can pinpoint the location where the original gamma ray interacted with the crystal.
![[Pasted image 20240528094055.png#invert|500]]
- A signal amplitude analysis circuit $Z$ allows to reject all signals that are not within an energy "window" chosen by the operator. -> In this way it is possible to eliminate part of the radiation of "scatter"
## Disadvantage
- There are a lot of losses in effectiveness along the chain
- This makes the image blurry
![[Pasted image 20240528094511.png|200]]
- As well as the reconstruction of the position has a ~3 mm inaccuracy
## Exercise
![[Pasted image 20240528095956.png#invert|600]]
![[Pasted image 20240528100017.png#invert|600]]