> [!abstract] Definition
>  Dual energy radiography is a technique that <mark style="background: #FFB86CA6;">utilizes two different X-ray energy levels to differentiate between various types of tissues</mark>, particularly bone and soft tissue.

- Different materials [[Image Formation#^864773|attenuate]] X-rays to different extents. 
- At lower energies (less than 80 keV peak), there’s a higher contrast between bone and soft tissue compared to higher energies.
## Process
- This process involves <mark style="background: #FFB86CA6;">taking two images at different energy levels</mark> and then <mark style="background: #FFB86CA6;">applying weights to each image</mark> to either enhance or nullify certain signals:
	- To create a **“soft-tissue only” image**, the signal due to bone is nulled.
	- To create a **“bone only” image**, the signal due to soft tissue is nulled.
### Weighted Subtraction and Scaling
- This is the mathematical process used to separate the images. For example, to remove the bone signal, you might use the following formula:$$(High\_Energy \times w_H - Low\_Energy \times w_L) \times k_t$$
- By choosing appropriate weights $w_H$ and $w_L$ and a scaling factor $k_t$, you can cancel out the bone signal and leave only the soft tissue signal.
## Single Shot Technique
- Two [[Computed Radiography (CR)]] phosphor plates are used, with a copper filter sandwiched in between.
	- The <mark style="background: #FFB86CA6;">first plate detects a standard energy spectrum</mark>, with an average energy of about 58 keV.
	- The copper <mark style="background: #FFB86CA6;">filter absorbs lower X-ray energies</mark>, allowing fewer photons with higher energy to pass through.
	- The <mark style="background: #FFB86CA6;">second plate then detects these higher energy photons</mark>.
- Both low and high energy images are <mark style="background: #FFB86CA6;">captured at the same time</mark>, which helps to <mark style="background: #FFB86CA6;">eliminate motion artifacts</mark> that could occur if the patient moves during the imaging process.
- The energy separation between the two image sets is small, leading to a relatively low SNR for the tissue and bone images at typical patient exposures
## Flat Panel Detector and keV-switch
- A flat-panel detector with a fast readout capability is used, typically involving a thin-film transistor (TFT) flat-panel.
- <mark style="background: #FFB86CA6;">Sequential Acquisition</mark>:
    - The **first acquisition** is made with a high-energy beam (120 kVp), which is quickly detected by the flat-panel.
    - The **second acquisition** uses a lower energy beam (60 kVp), followed by another fast readout.
- This method provides a larger energy separation, which allows for a relatively high SNR for the images.
- <mark style="background: #FFB86CA6;">Due to the time delay (~230 ms) required to acquire the two images, there is a risk of motion artifacts</mark>, which can affect image quality
## Clinical Use
- A clinical Use of Dual Energy Radiography is the reduction of metal artifacts.
![[Pasted image 20240511112656.png|400]]