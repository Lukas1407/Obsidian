- „Annealing“ = Erhitzung + anschließende kontrollierte langsame Abkühlung
	- Optimale Gitterstruktur bei möglichst niedriger Energie 
	- Je wärmer (je höher die Energie),  desto beweglicher die Teilchen, desto besser können sie sich noch „umsortieren“
![[Pasted image 20240412084127.png#invert|300]]
- Besonderheit: Zulassen eines vorübergehend schlechteren Zwischenzustands, um zum globalen Optimum zu gelangen
![[Pasted image 20240412084222.png#invert|300]]
- Löst Minimierungsproblem: Zielfunktion $E(p)$ wird minimal 
- Maximierungsproblem kann gelöst werden über Minimierung von z.B. $-E(p)$ oder $\frac{1}{E(p)}$
- Anwendungen 
	- Z.B. Regler-Parameter-Optimierung
- Nur eine Lösung wird zum Absuchen verwendet 
- Keine Geschwindigkeitsvorteile durch Parallelisierung
- Beispiele für Anwendungen: 
	- Reihenfolge für verschiedene Bearbeitungsschritte einer Produktion finden, die möglichst schnell ist 
	- Minimierung der Anzahl der zur Beschreibung notwendigen Parameter einer Geometrie
## Algorithmus
- Startlösung ist gegeben (z.B. durch vorherige Optimierung)
- Wähle im Umkreis (lokaler Suchraum) zufällig neuen Lösungskandidaten aus 
- Entscheidung Annahme neuer Kandidat über [[Simulated Annealing#Metropolis-Algorithmus|Metropolis-Algorithmus]] 
- -> Verschiebung des Suchraums
- Abkühlung bei jedem Schritt: $$T_{n+1}=\alpha*T_{n},\ \ \alpha\in[0,1)$$
	- $T$ reflektiert die Wahrscheinlichkeit, dass sich Ergebnis auch mal verschlechtern darf
	- -> Mit zunehmenden Zeitschritten wird $T_{i}$ kleiner, so werden immer unwahrscheinlicher schlechte Lösungen akzeptiert
### Metropolis-Algorithmus
- Wahl des neuen Lösungskandidaten $p_{n+1}$ in lokalem Suchraum um aktuelle Lösung $p_{n}$
- Berechne die "Energie" (Güte/Fitness/Zielfunktion) $E)p_{n+1}$
- Wenn $E(p_{n+1})< E(p_{n})$:
	- Akzeptiere neue Lösung $p_{n+1}$ immer
- Wenn $E(p_{n+1})> E(p_{n})$:
	- Akzeptiere neue, „schlechtere“ Lösung mit Wahrscheinlichkeit: $$p(\Delta E,T)=e^{-\frac{\Delta E}{T}}$$
	- Mit $\Delta E=E(p_{n+1})-E(p_{n})$
	- Je kleiner $\Delta E$ bzw. je höher $T$, desto höher die Wahrscheinlichkeit für einen Übergang in „schlechtere“ Lösung
	- -> aus der Physik: Boltzmann-Statistik
### Abbruchkriterien 
- Z.B. maximale Anzahl von Durchläufen 
- Ausreichend niedriges $E$ 
- Anzahl Zeitpunkte über die sich $E$ nicht mehr ändert
![[Hill_Climbing_with_Simulated_Annealing.gif#invert|400]]

## Unterschied zu [[Evolutionary Algorithms (EA)|EA]]
![[Pasted image 20240412095737.png#invert|400]]
- Keine „Population“ in dem Sinne, nur ein Individuum 
- „Nachkomme“ ist das Individuum selbst