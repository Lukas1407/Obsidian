> [!summary] Definition
> Self-Organizing Map (SOM) sind eine Art von künstlichen neuronalen Netzen, die für [[Unsupervised Learning|unüberwachtes Lernen]] verwendet werden. Sie sind inspiriert von der Funktionsweise der menschlichen [[Cerebral Cortex|Hirnrinde]] und dienen dazu, hochdimensionale Daten auf eine niedrigdimensionale (meist ein- oder zweidimensionale) Karte zu projizieren 
- Synonyme: Kohonen-Karten, Self-Organizing Feature Map (SOFM)

## Prinzipien von Kohonen-Karten:
- Topografische Karten: Die räumliche Lage eines Neurons auf der Karte entspricht einem Bereich oder Merkmal in den Eingangsdaten.
- Dimensionalitätsreduktion: SOMs reduzieren die Dimensionalität der Eingangsdaten, indem sie diese auf eine Karte mit niedrigerer Dimension abbilden.
- Anordnung der Neuronen: Die Neuronen werden in einem ein- oder zweidimensionalen Gitter angeordnet, ähnlich der Anordnung in der Hirnrinde.
![[Pasted image 20240413100604.png#invert|400]]
## Funktionsweise
- Der Eingangsvektor $x$ hat $s$ Komponenten. Dementsprechend hat der Parametervektor des $j$-ten Ausgangsneurons $w_j$ die Dimension $s$. 
- Wird dem SOM ein Eingangsvektor vorgelegt, wird bei einem angelernten Netz lediglich eine Gruppe benachbarter Ausgangsneuronen aktiviert. 
- Berechnung des Zustands: 
	- über Distanz, meist Verwendung des Euklidischen Abstands: $z_j=||x-w_j ||$ 
	- Alternative: Verwendung des inneren Produkts bei normalisierten Gewichtsvektoren $w$ und Eingangsvektoren $x: z_j=x^{T}w_j$
- Ausgang des Netzes: Gewinnerneuron $j(x)$ (winner-takes-all-Prinzip) ergibt sich mit $j(x)=\arg\min z_j$
- Die Nachbarschaftsstruktur ist festgelegt, sodass benachbarte Neuronen ähnliche Muster repräsentieren

## Lernverfahren
- Das Netzwerk lernt durch Anpassung der Gewichte der Neuronen, um die Eingangsdaten besser zu repräsentieren.
- Die Gewichte werden so angepasst, dass sie wichtige Bereiche des Eingangsraums abdecken.
- Das Lernen erfolgt durch Verschieben der Gewichte des Gewinnerneurons und seiner Nachbarn in Richtung des aktuellen Eingangsvektors.
1. Festlegen der Dimension des Gitters und der Anzahl der Ausgangsneuronen 
2. zufällige Initialisierung der Gewichte $w_{SOM,i}$
3. zufällige Auswahl eines Datentupels $x_{n}$
4. Bestimmung des Gewinnerneuron wird für dieses Datentupel bestimmt 
5. Bestimmung der Nachbarn für das Gewinnerneuron
6. Gewinnerneuron (besonders stark) und dessen Nachbarn (etwas weniger) werden in Richtung von $x_n$ verschoben
7. Berechnung eines Gütekriteriums (basierend auf dem durchschnittlichen Abstand der letzten Datentupel zum Gewinnerneuron) 
8. Abbruch, wenn Güteanforderung erfüllt, sonst k=k+1 und Fortsetzen mit 3.

- Die resultierende Karte bietet eine topologieerhaltende Abbildung, bei der ähnliche Eingangsvektoren auf benachbarte Positionen auf der Karte abgebildet werden.
- Die Analyse der Karte kann Aufschluss über die Verteilung der Eingangsdaten geben, wie zum Beispiel die Anzahl der Datentupel pro Neuron

## Anwendungen 
- Problemtypen:
	- Dimensionsreduktion bei höherdimensionalen Merkmalsräumen: z. B. in der Spracherkennung, Klassifizierung von Phonemen (Transformation der durch Fourier-Transformation gewonnen Merkmale auf zweidimensionale Phonemkarte) 
	- Approximation und Visualisierung von höherdimensionalen nichtlinearen Zusammenhängen: z. B. Robotersteuerungen 
- Anwendungsfelder:
	- Sprachverarbeitung 
	- Bildverarbeitung 
	- Robotik/autonome Systeme 
	- Telekommunikation