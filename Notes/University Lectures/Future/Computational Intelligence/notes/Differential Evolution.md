- Optimierung in mehrdimensionalen Suchräumen mit Fließkommazahlen
- Ein Lösungskandidat ist ein $\mathcal{D}$ dimensionaler parameter $$p= \left(\begin{array}{c} p_{1}              \\ ... \\ p_\mathcal{D} \end{array}\right)$$
- Populationsgröße: $n_{p}$, vernünftig zwischen $n_p=5\mathcal{D}$ und $n_{p}=10$$\mathcal{D}$

> [!success] Vorteile
>  - Gut geeignet für Parallelisierung 
>  - Leicht zu implementieren 
>  - Sehr wenige Kontrollparameter ($CR, F, n_{p}$) 
>  - Adaptiert „Skala“ auf der sich das Problem im Parameterraum abspielt, da existierende Lösungen adaptiert werden und keine ganz neuen entworfen werden

- Beispiele für Anwendungen: 
	- Optimierung von Hyperparametern 
	- Architektur in neuronalen Netzen Automatisierte Auslegung von Bauteilen
## Algorithmus
![[Pasted image 20240411152047.png|600]]
### Difference-vector based mutation
- 4 zufällig ausgewählte Elternteile: $p,p_{a},p_{b},p_{c}$
- Mutation: $$q=p_{a}+F*(p_{b}-p_{c})$$
- Differentialgewicht $F\in [0,2]$ gibt an wie stark die neue Lösung von dem alten ($p_{a}$) abweichen darf; typischerweise im Intervall $[0.4, 1]$
- -> Neues Elternpaar ist nun $p$ und $q$
### Crossover/Rekombination
- Crossover Wahrscheinlichkeit $CR\in[0,1]$
- $j\in[1,\mathcal{D}]$ ganzzahlige Zufallszahl für die zu ändernde Dimension
- $r_{i}\in [0,1]$ reelle Zufallszahl 
- Die neue Lösung ist dann gegeben durch $$u_{i} = \begin{cases}
q_{i} & \text{falls $r_{i}\le CR$ oder $i=j$ } \\
 p_{i} & \text{sonst}
\end{cases} $$
- $i=j$ gewärleistet, dass mind. einer der Parameter per crossover modifiziert wird
### Selektion
- Das bessere Individuum von $p$ und $u$ wird übernommen
- Wenn gleich gut: das neue Individuum wird übernommen, um die Heterogenität zu erhöhen

## Unterschied zu [[Evolutionary Algorithms (EA)|EA]]
![[Pasted image 20240412080826.png#invert|400]]
- Verwendet nicht nur Parametervektoren selbst, sondern Differenz zwischen Parametervektoren