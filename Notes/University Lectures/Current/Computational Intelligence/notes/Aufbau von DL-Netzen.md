- oftmals Feedforward-Netze 
- auf hochdimensionale Eingangsgrößen (meistens Bilder) angepasst, Pixel sind Eingang des Netzes 
- Ausgang je nach Netz: Label für ganzes Bild, lokalisierte Objekte oder pixelweise Segmentierung 
- geschickte Verbindung der Layer mit Vorstrukturierung: 
	- Faltung (Convolution) mit gekoppelten Parametern und lokaler Zuweisung 
	- Downsampling: Max-Pooling zur Reduktion der Auflösung 
	- ReLU (weniger beteiligte Neuronen) 
	- "Fully Connected Layer" als MLP 
	- Wiederholung gleicher Layer
- Faltung (Convolution+Stride): Zusammenfassen benachbarter Regionen (Parameter: Wichtungsmatrizen der CNN - Schichten, Stride: Schrittweite der Faltungsmatrix) 
- Batch -Normalization: Verbesserung der Parameteroptimierung 
- Max–Pooling: Reduktion der Auflösung 
- voll -vernetzte Schichten: „Interpretation / Abstraktion“ der erhaltenen Datenströme