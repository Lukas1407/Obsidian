> [!abstract] Definition
> Radial-Basis-Funktions-Netze (RBF-Netze) sind eine spezielle Art von [[Artificial Neuronal Network (ANN)|künstlichen neuronalen Netzen]], die für Aufgaben wie Interpolation, Approximation und Mustererkennung verwendet werden. Es sind Feedforward-Netze.

- Gegeben sind $N$ Datentupel $(x_{n},y_{n})$ mit :
	- bekannten Eingangsgrößen $x_{n}$ 
	- und bekannten Ausgangsgrößen $y_{n}$ 
- Gesucht ist eine Funktion $y = f(x)$, die diese Punkte verbindet 
	- Interpolation: Funktion läuft genau durch diese Punkte 
	- Approximation/Regression: Funktion läuft in der Nähe dieser Punkte weil: 
		- auf $y$ Störungen liegen (Regression) 
		- oder der funktionelle Zusammenhang anders ist (Approximation) 
		- oder sowohl Störungen als auch andere funktionelle Zusammenhänge vorliegen (Approgression)
	- $f(\cdot)$ ist eine Linearkombination von radialsymmetrischen Basisfunktionen

- Jedes Neuron hat nur lokale Wirkung!
## RBF-Neuron
- Alle Neuronen der verdeckten Schicht sind RBF-Neuronen
![[Pasted image 20240413092624.png#invert|300]]
- Funktionen zur Bestimmung des Zustands der RBF-Neuronen sind Gaußfunktionen
- Jedes RBF-Neuron hat einen Parametervektor $w_{ij}$, der den Mittelpunkt der Basisfunktion kennzeichnet, und einen Parameter $\sigma$, der die Breite der Funktion definiert (analog zum Mittelwert bei Normalverteilung).
- lineare [[Activation Function|Aktivierungsfunktion]] $f(z) = m\cdot z$
- für jedes Neuron der RBF-Schicht gilt $$y_{i}^{(2)}=e^{-s_{i}^{(2)}}=exp\left(-w_{i0}^{(2)}\sum_{j}(x_{j}-w_{ij}^{(2)})^{2}\right)\text{ mit }w_{i0}^{(2)}=\frac{1}{2\sigma^{2}}$$
- Die RBF-Neuronen reagieren stärker, wenn die Eingangswerte dem Parametervektor ähnlich sind.

## Lernen in RBF-Netzen
- Lernen der optimalen Gewichte zum Ausgangsneuron $w_{1j}^{(3)}$ erfolgt über ein parameterlineares Schätzproblem, das oft mit der Methode der kleinsten Fehlerquadrate gelöst wird.
- - Die Platzierung der RBF-Neuronen ist ein nichtlineares Problem und kann durch verschiedene Algorithmen gelöst werdem:
	- schrittweise Erhöhung der Anzahl der RBF-Neuronen:
		1. Platzierung eines neuen Neurons am Datentupel mit dem größten Fehler 
		2. Lernen der optimalen Gewichte zum Ausgangsneuron für RBF-Netz 
		3. Abbruch, wenn Zielgütewerte erreicht, sonst Fortsetzen mit 1.
	- Clusterverfahren
	- Platzierung auf regelmäßigem Gitter