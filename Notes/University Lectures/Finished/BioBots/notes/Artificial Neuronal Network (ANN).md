> [!summary] Definition
> Künstliche Neuronale Netze (KNN) sind stark vereinfachte technische Realisierungen zur Modellierung der Informationsverarbeitung im [[Brain|Gehirn]] und im [[Nervous System|Nervensystem]]. 

- Nutzung: 
	- Lernen und Verallgemeinern anhand von Beispielen 
	- Erkennen und Vervollständigen komplizierter Muster
- Kennzeichen sind lernfähige, dezentrale, parallele Strukturen aus einfachen Elementen (Prozesseinheiten (PE) bzw. Neuronen)

## Bestimmung des internen Zustands $z$
![[Pasted image 20240413083600.png#invert|300]]
### Gewichtete Summe mit Absolutterm
$$z(x,w)=w^{T}\cdot x+x_{0}$$
- Genutzt in z.B. [[Multi-Layer-Perceptron (MLP)]]
### Radiale Basisfunktion (RBF)
$$z(x,w)=e^{-w_{0}\cdot (x-w)^{T}(x-w)}$$
- Genutzt in z.B. [[Radial-Basis-Function Networks (RBF)]]
### Beliebige Distanz
$$z(x,w)=e^{-d^{2}(x,w)}$$
### Wettbewerbslernen
$$z(x)=\arg\max_{i}x_{i}$$

## Hauptunterschiede zu [[Biologisch Neuronales Netz]]
- Ein- und Ausgangsgrößen sind keine Zeitreihen von Spikes, sondern statisch (Äquivalent: höhere Werte, wenn Neuron öfters "feuert")  ^60d405
	- Das bedeutet, dass die Eingangswerte zu einem bestimmten Zeitpunkt eingegeben werden und das Netzwerk daraufhin einen Ausgangswert liefert, ohne dass eine zeitliche Abfolge von Signalen berücksichtigt wird.
- Zustand ist statisch, damit kein Zustand im Sinne einer Differenzialgleichung oder Differenzengleichung mehr 
	- -> Variieren nicht über die Zeit
- Das Verhalten eines künstlichen Neurons ist folglich statisch in dem Sinne, dass es für einen gegebenen Satz von Eingangswerten immer denselben Ausgangswert liefert.

## Verbindungsstrukturen
### Feed-Forward
- Die Schichten werden in Vorwärtsrichtung miteinander verbunden. Die Information breitet sich von der Eingabeschicht durch die verdeckte(n) Schicht(en) aus und führt zum Ergebnis in der Ausgabeschicht
![[Pasted image 20240413084139.png#invert|200]]
- Vorwärtsgerichtete Netze mit statischen Neuronen haben statisches Ein-Ausgangs-Verhalten.
### Laterale Verbindungen
- Innerhalb einer Schicht existieren Verbindungen zwischen den einzelnen Prozesseinheiten (PE)
![[Pasted image 20240413084208.png#invert|200]]
### Feedback
- Der Ausgang einer PE wird auf den eigenen Eingang zurückgeführt oder Ausgänge von PE werden über eine oder mehrere Schichten zurückgekoppelt
![[Pasted image 20240413084239.png#invert|200]]
- Laterale und rückgekoppelte Verbindungen führen oft zu zeitlichen Abhängigkeiten (meist als Zeitverzögerung um einen Abtastschritt), Netze werden dann als rekurrente Netze bezeichnet und haben dynamisches Ein-Ausgangs-Verhalten. 
- Rückkopplungen sind sowohl im Netz als auch außerhalb des Netzes (Ausgangsgrößen koppeln auf Eingangsgrößen zurück) möglich!