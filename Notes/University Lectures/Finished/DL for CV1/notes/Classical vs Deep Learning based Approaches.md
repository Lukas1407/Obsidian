## Classical Approaches
- Required a lot of handcrafted components for feature extraction
	- Color, Edges, [[Histogram of Oriented Gradients|HoG]], ...
- Which is hard to come up with
- And also leads to poor generalization

- Vorverarbeitung:
- Datenerfassung 
	- Datenannnotation 
	- Daten-Vorverarbeitung 
	- Daten-Augmentierung 
- Händische Merkmale 
	- Wissen über die Domäne 
	- Trial and error 
	- Merkmalsauswahl 
	- Merkmalsart-Auswahl 
- Merkmalstransformationen und -klassifikation 
	- Zusammenfassung von Merkmalen 
	- Mustererkennung aus Daten (z.B. Bayes, SVM etc.) 
- Anwendung
	- Inferenz auf der Basis unbekannter Daten

## Deep Learning Approaches
- The model learns to extract features
- No or little handcrafted components necessary
- Generally deeper models with more layers perform better, as they allow to extract more information from the data
- Advantages:  
	- Automatic
	- Fantastic performance in a lot of tasks
	- Straightforward architecture
	- Reasonable training time
	- Human-like or super-human behavior
- Disadvantages:
	- Need a lot of training data
	- Need [[Graphical Processing Unit|GPUs]] for training
	- Usually slower inference speed that classical approaches
	- Difficult to deploy
	- Difficult to debug