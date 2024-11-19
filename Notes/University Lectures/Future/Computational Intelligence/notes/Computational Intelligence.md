> [!summary] 
>  Computational Intelligence umfasst alle Gebiete, die sich mit der Nachbildung biologischer und menschlicher Problemlösungsstrategien im Computer beschäftigen.

- Computational Intelligence ist immer nur eine Variante unter anderen, nicht immer die Beste!
	- Meist sind speziell für das Problem entwickelte Verfahren besser
## Warum benötigt man CI?
- Lösen von Aufgaben, 
1. bei denen kein wissenschaftlich begründetes mathematisches Modell ("White-Box-Modell") oder Lösungsverfahren vorliegt, die der Mensch aber trotzdem irgendwie kann
		- -> Wenn "klassische" Methoden nicht das Problem lösen können
		- durch Lernen von Beispielen und/oder
		- durch Versuch und Fehler und/oder
		- regelbasiertes Expertenwissen (Fahrlehrer auf Beifahrersitz gibt Hinweise)
2. Bei denen andere Verfahren keine befriedigende Lösung finden
	- schlechte Lösungsqualität
	- zu lange Rechenzeiten

## Teilgebiete
1. [[Evolutionary Algorithms (EA)]]
2. [[Fuzzy-Systeme]]
3. Künstliche Neuronale Netze
4. [[Data Mining]]
### Weitere Gebiete
- hybride CI-Systeme:
	-  Neuro-Fuzzy-Systeme (aus der Mode)
	- Evolutionär optimierte Fuzzy-Systeme 
	- Evolutionär optimierte Künstliche Neuronale Netze 
- andere naturanaloge Verfahren ("Metaheuristiken"): 
	- [[Partikelschwarmoptimierung]] (PSO) 
	- Ameisenalgorithmen bzw. Ant Colony Optimization ACO
- Memetische Algorithmen (Kombination aus Evolutionären Algorithmen und lokalen linearen Suchverfahren)
### Verwandte Begriffe
- Soft computing: 
	- teilweise Synonym zu Computational Intelligence 
	- Aufzählung Fuzzy, Neuronale Netze, Evolutionäre Algorithmen 
	- Betonung approximativer Lösung für schwer berechenbare Probleme 
- Künstliche Intelligenz:
	- in letzter Zeit wieder allgemeiner interpretiert, insbesondere für autonome Entscheidungen von Maschinen symbolische Welt
		- Go, Schach, ChatGPT usw. 
		- subsymbolische Welt: autonomes Fahren, Sprachassistenten usw.

## Inhalt
### [[Optimierungsprobleme]]
- Viele Verfahren der Computational Intelligence haben mit Optimierungsproblemen zu tun:
	- als „Löser“: Evolutionäre Algorithmen 
	- als „Vermeider“: Viele Fuzzy-Systeme 
	- benötigen Lösungen von Optimierungsproblemen: 
	- Künstliche Neuronale Netze und einige Fuzzy-Systeme, da Parameter optimiert werden müssen
### Evolutionäre, Memetische und Naturinspirierte Algorithmen
#### Wieso biologisch inspirierte Algorithmen?
-  Genetische Informationen dienen als Bauplan für den Organismus
	- Kodiert für alle Organismen in Form von Desoxyribonukleinsäuren (DNA), die aus vier Basen (A, C, G, T) bestehen und als Basenpaare verbunden sind
	- Phänotyp nicht nur durch den Genotyp, sondern auch durch Umwelteinflüsse bestimmt (z.B. Ernährung)
- Bewertung durch Überleben (Selektion) in einer Umgebung und erfolgreiche Weitergabe der Erbinformation an die Nachkommen
	- Individuen einer und verschiedener Arten leben in einer gemeinsamen, sich ständig ändernden Umwelt, besser angepasste Individuen haben eine höhere Chance auf die Erzeugung von Nachkommen (Selektion)
	- Populationen mit unterschiedlichen Genotypen haben eine bessere Chance auf Anpassung bei sich verändernden Umgebungsbedingungen
- Veränderung der Erbinformation:
	- bezogen auf das einzelne Individuum: 
		- zufällige Mischung der elterlichen Erbinformation (Rekombination) 
		- zufällige Veränderung von (kleinen) Teilen der Erbinformation (Mutation) 
		- Umwelteinflüsse können die Erbinformation verändern (Epigenetik) 
		- Rekombination oft erfolgreicher als Mutation wegen geringerer Fehlerquote (deshalb dominiert geschlechtliche Vermehrung)
	- bezogen auf die Population: 
		- Partnerwahl basierend auf der Fitness (Selektion) 
		- räumliche Einschränkung bei der Partnerwahl Anpassungen 
		- erfolgen langsam im Verlauf von Generationen 
	- bezogen auf die kulturelle Evolution: 
		- Weiterentwicklung erlernbarer Fähigkeiten im Laufe einer Lebensspanne 
		- Weitergabe von erlernbaren Fähigkeiten an die nächste Generation
#### [[Evolutionary Algorithms (EA)|Evolutionäre Algorithmen]]
#### [[Memetische Algorithmen]]
#### Metaheursitische Verfahren
- Sind Algorithmen zur närungsweisen Lösung von Optimierungsproblemen
- Können auf beliebige Problemstellungen angewendet werden
- Gut für nicht konvexe, hochdimensionale Probleme (mit (viele) lokalen optima)
	- -> Gradienten basierte Verfahren sind hier schlecht
- Aber: Keine Garantie, dass das globale Optimum gefunden wird! -> nährungsweise
- Oft kann aber eine gute Nährungslösung gefunden werden
- Beispiele für Anwendungen:
	- Hyperparameter von NN 
	- Auslegung von Geometrien
	- Proteinfaltung
- Zu Metaheursitischen Verfahren gehören:
	1. [[Differential Evolution]]
	2. [[Genetische Programmierung]]
	3. [[Partikelschwarmoptimierung]]
	4. [[Simulated Annealing]]
### [[Fuzzy-Systeme]]
### Künstliche Neuronale Netze
- [[Biologisch Neuronales Netz]]
- [[Action Potential|Aktionspotential]]
- [[Artificial Neuronal Network (ANN)]]
- Die Struktur von [[Artificial Neuronal Network (ANN)]] kann manuell (mit einfachen Netzen beginnen und dann Anzahl der Neuronen in der verdeckten Schicht steigern) oder mit geeigneten Optimierungsverfahren (z.B. [[Evolutionary Algorithms (EA)]]) bestimmt werden
	- Wahl der Eingangsgrößen eines Systems (Wieviele? Welche?) 
	- Wahl der Ausgangsgrößen eines Systems (Wieviele? Welche?) 
	- Verbindungsstruktur 
	- Anzahl verdeckter Schichten 
	- Anzahl der Neuronen in der verdeckten Schicht 
	- Art der Aktivierungsfunktion 
	- Funktion zur Bestimmung des Zustands 
	- Struktur bestimmt Typ des Neuronalen Netzes, z.B. Multi-LayerPerceptron (MLP): feedforward, >=1 verdeckte Schicht, gewichtete Summe mit Absolutterm, Tansig- oder Sigmoid-Aktivierungsfunktion, ... 
- Typen von Lernverfahren
	- [[Supervised Learning]]
	- [[Unsupervised Learning]]
	- [[Reinforcement Learning]]
	- [[Semi-Supervised Learning]]
	- [[Selbstorganisiertes Lernen]]
- [[Multi-Layer-Perceptron (MLP)]]
- Verfahren zur Bestimmung der Parameter: 
	- garantieren keine optimale Lösung, alle genannten Verfahren können in lokalen Minima der Gütefunktion stagnieren 
	- Verfahren des steilster Abstiegs (Backpropagation) konvergiert oft extrem langsam 
	- Levenberg-Marquardt-Verfahren konvergiert viel schneller, erfordert aber die aufwändige Berechnung der Hesse-Matrix (u.U. Laufzeit- oder Speicherprobleme im Computer, insbesondere bei großen Netzen)
- [[Self-Organizing Map (SOM)]]
- Künstliche Neuronale Netze sind wegen ihrer Eigenschaft als "universelle Approximatoren" populär
-  besonders sinnvoll, wenn strukturelle Zusammenhänge unbekannt sind
- Rekurrente Netze stark umstritten 
	- Pro: Abbildung dynamischer Zusammenhänge 
	- Kontra: Konvergenzprobleme
### Deep Learning
- Warum Deep Learning, was ist bislang nicht möglich? 
	- Verallgemeinerung von implizitem Wissen 
	- Expertenwissen oft nicht verfügbar 
	- Verarbeitung von Bildern 
	- Automatische Objekterkennung und -bewertung
	- Entwurf unklar (Trial & Error)
- Ziel: Extraktion von Wissen aus großen Datenbanken
- Warum konnte sich Deep-Learning auf einmal entwickeln? 
	- Datensätze: automatisiertes Sammeln und Nutzen der Crowd -> große gelabelte Datensätze 
		- ImageNet, COCO, KITTI, MNIST, CIFAR etc. 
	- Hoch-performante Hardware: Entwicklung paralleler Prozessoren CPUs, GPUs, (TPUs), FPGAs, ZFProAI etc. 
	- Software: Bibliotheken und IDEs Tensorflow, Caffee, Pytorch, ROS etc. 
	- Einfluss: Unterstützung großer Unternehmen Entwicklung: Google, Amazon, Facebook, Uber, Anwendung: Daimler, ZF, … 
	- Algorithmische Fortschritte Backpropagation, Konvergenz der Optimierung Neue Strukturen: CNN, GAN etc.
- [[Classical vs Deep Learning based Approaches]]
- [[Typische Filter in der Bildverarbeitung]]
- Bewertung von Faltungsoperationen
	- Faltungen liefern offensichtlich interpretierbare Werte
		- -> mitunter erst nach mehrfacher Anwendung 
	- Art der Faltung (Filtermatrix) für sinnvolle Merkmale unbekannt, mehrere sinnvolle Merkmale möglich (z.B. horizontale + vertikale Kanten) 
		- -> Viele verschiedene Filtermatrizen implementieren und optimieren 
	- Jede Filtermatrix liefert ein Bild gleicher Größe, bei mehrfacher Anwendung und Verwendung verschiedener Filtermatrizen entstehen riesige Datenmengen. 
	- Möglichkeiten zur Komprimierung des Datensatzes sind notwendig!
- [[Aufbau von DL-Netzen]]
- Training
	- Anpassung der Gewichte, Trainingsparameter 
	- Dateneffizientes Lernen (z.B. [[Transfer Learning]]) 
	- Datensatzoptimierung und -erweiterung (z.B. Augmentierung) 
	- Erkennen von [[Challenges in Deep Learning#Overfitting|Overfitting]]
	- Mensch-Maschine-Schnittstelle (z.B. KaIDA) 
	- (Optimierung der Netzwerkstruktur) 
		- Regularisierung 
		- Dropout
	- Trainingsparameter: 
		- Lernrate 
		- Batch Size 
		- Gradientenberechnung
		- Initiale Gewichtung 
		- Abbruchkriterium
	- häufige Probleme: zu viele Parameter keine verlässliche Schätzung 
		- Idee: Erzeugen zusätzlicher Bilder beim Training oder beim Testen ("Test-Time-Augmentation") Maßnahmen: z.B. Rauschen, Drehen, Rotieren, Scharfzeichnen usw. 
		- Ziel: 
			- Training: kleinerer Klassifikationsfehler der Bilder durch größere Robustheit 
			- Testen: Validieren, ob Ergebnisse konsistent sind
- Manchmal keine annotierten Daten verfügbar oder verfügbarer Datensatz viel zu klein 
- Oder: Implizite Vorgabe zu lernender Daten gewünscht 
	- ->Vergrößerung oder Erstellung eines Lerndatensatzes durch synthetische Daten 
		- händisch 
		- Simulation mit physikalischen Modellen 
		- Synthetisierung durch neuronale Netze 
		- Z.B. BMBF-call „Erzeugung von synthetischen Daten für Künstliche Intelligenz“ 
	- Was, wenn immer noch zu wenig Daten verfügbar? -> [[Transfer Learning]]
- 
- Probleme: 
	- Semantische Unterschiede in Lern- und Testdaten (Klassifikation von getarnten Panzern) 
	- Zeitliche Abhängigkeiten (Mondphasen, winkende Fußgänger etc.) 
- Lösung: 
	- Datenaufnahme systematisieren (und evtl. automatisieren) 
	- Zufällige Aufteilung des Datensatzes, um zeitliche Biase zu verhindern 
	- Große Varianzen erhöhen Robustheit!
- Datensatzaufteilung zur Optimierung (oft: 60:20:20) 
- Lerndatensatz: Optimierung -> Erlernen von Mustern 
	- Parameter des Modells werden iterativ angepasst, zur Minimierung der Zielfunktion (Loss) 
- Validierungsdatensatz: Hyperparameterbestimmung 
	- Das Modell wird getestet, um Hyperparameter anzupassen und die Generalisierungsfähigkeit abzuschätzen. 
- Testdatensatz: Testen der Generalisierungsfähigkeit 
	- Durch Anpassung der Hyperparameter wird der Validierungsdatensatz implizit gelernt. Eine finale Aussage über die Generalisierungsfähigkeit liefert der Testdatensatz.
- Weniger Lerndaten -> Höhere Varianz in Parameterschätzung

- Beispiele für Netze: [[GoogLeNet]], [[ResNet]], [[VGGNet]]
- Rest in [[CI_6_67_DeepLearning.pdf]]