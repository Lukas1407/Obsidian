> [!abstract] Definition
> Ein Optimierungsproblem besteht aus einer Zielfunktion $Q(p)$, die maximiert oder minimiert werden soll) und einer Menge zulässiger Lösungen $p\in M$. $M$ kann weiter [[Optimierungsprobleme#Restriktionen|Restriktionen]] beinhalten (-> restringiertes Problem), sonst freies Problem.

- Das Optimierungsproblem ist nicht immer gegeben, sondern muss u.U geschickt "gebaut" werden
	- -> Viele potentielle Möglichkeiten
- Manchmal bewertet nur der Mensch eine Lösung $p$
- Parameter können so transformiert werden, dass [[Optimierungsprobleme#Restriktionen|Restriktionen]] immer erfüllt werden (z.B. $p^2$ statt $p$ sichert Erfüllung der Restriktion $p\ge0$)

- Auswertung der Zielfunktion $Q(p)$ kann geschehen durch:
	- Direkter Berechnung aus $p$
	- Simulation auf Basis von $p$
	- Experimente auf Basis von $p$

- ![[No-Free-Lunch-Theorem]]

## Typische Optimierungsprobleme
- Finde den kürzesten Weg von A nach B 
- Finde den schnellsten Weg von A nach B (z.B. mit der Deutschen Bahn) 
- Finde die kostengünstigste Variante für ein Bauteil, das technische [[Optimierungsprobleme#Restriktionen|Restriktionen]] (Bauform, Festigkeit, Temperaturbeständigkeit, Masse) einhalten muss 
- Finde die leichteste Variante für ein Bauteil, das technische [[Optimierungsprobleme#Restriktionen|Restriktionen]] (Bauform, Festigkeit, Temperaturbeständigkeit) und Kostenlimits einhalten muss

## Design der Zielfunktion
- Sollte alle Anforderungen (Restriktionen, etc.) beinhalten, da sonst die Lösung zwar die Zielfunktion minimiert/maximiert, aber nicht die gewünschte Lösung liefert
- Die Art der Zielfunktion beeinflusst die Lösbarkeit -> es git leichter lösbare Zielfunktionen
	- Zielfunktion kann auf bessere Lösbarkeit optimiert werden, z.B. durch Vermeidung konstanter Werte bei [[Gradientenverfahren]]
- Enthalten oftmals widersprüchliche Forderungen, z.B. Qualität vs. Kosten. Hier kann man:
	- in eine Zielfunktion Q(p) integrieren, oft als Kosten oder als gewichtete Summe 
	- Vektorielle Zielfunktion Q(p) : Multikriterielle Optimierungen 
	- Realisierung über [[Optimierungsprobleme#Restriktionen|Restriktionen]]/Nebenbedingungen 
	- Kombination mehrerer Varianten
- Kann deterministisch sein oder Unsicherheiten enthalten

## Eigenschaften von Zielfunktionen
- Lineare Zielfunktionen:
	- kommen praktisch nur mit Restriktionen vor, sonst Optimum im Unendlichen 
	- mit Restriktionen: Lösungen liegen am Rand 
	- keine lokalen Optima 
- Quadratische Zielfunktionen: 
	- keine lokalen Optima, nur ein globales
	- mit Restriktionen: Lösungen am Rand oder im Optimum des freien Problems 
- Nichtlineare Zielfunktionen: 
	- häufig lokale Optima
	
- Mit zweidimensionalen Lösungen noch durch Visualisierung erfassbar, praktische Probleme oft hoch-dimensional
- Wenn Optimierungsprobleme mit mehrdimensionalen $p$ separierbar sind $Q = Q_1 (p_1 ) + Q_2 (p_2 ) + … + Q_q (p_q )$, können die Probleme nacheinander einzeln gelöst werden
	- -> In mehrere kleinere, einfach lösbarere Probleme unterteilt
	- -> Mehrere konkurierende Ziele -> [[Pareto-Front]]

## Restriktionen
- Auch genannt: Nebenbedingungen
- Ungleichungsrestriktionen, z.B.:
	- Verbot nicht-negativer Konzentrationen $p>0$
	- Stellbegrenzungen wie 0 bis 100% 
	- zulässige Wertebereiche, reell, ganzzahlig
- Gleichungsrestriktionen, z.B.: 
	- Systemverhalten 
- Viele kompliziertere Restriktionen können vorkommen, z.B.:
	- in Reihenfolgeproblemen: jeder ganzzahlige Wert $p=1…P$ muss genau einmal vorkommen

## Lösungsverfahren
- Wichtige Fragen zur Eignung des Lösungsverfahrens sind: 
	- Garantieren die Lösungsverfahren (fast immer) das Finden der optimalen Lösung bzw. der optimalen Lösungsmenge? 
	- Wenn nein, wie gut ist die Chance auf das Finden einer akzeptablen Näherungslösung? 
	- Wie hoch ist der Rechen- und Speicheraufwand bis zum Erreichen der optimalen oder einer akzeptablen Lösung? 
	- Wie kann Vorwissen integriert werden? 
	- Macht es Sinn eher das Optimierungsproblem umzuschreiben oder zu vereinfachen, dass es besser zu einem Lösungsverfahren passt?

- Exakte Verfahren für spezielle Optimierungsprobleme finden (fast) immer die optimale Lösung 
- Näherungsverfahren, die die optimale Lösung nicht finden. Z.B.:
	- Heuristiken: akzeptable Lösungsverfahren für spezielle Optimierungsprobleme 
	- Generische Verfahren für verschiedene Optimierungsprobleme:
		- Monte-Carlo-Verfahren
		- [[Evolutionary Algorithms (EA)|Evolutionäre Algorithmen]]
### Rastersuche
![[Rastersuche]]
### Monte Carlo Methode
![[Monte Carlo Optimierungsmethode]]
### Rastersuche vs Monte Carlo für Automatisiertes Design von kNNs
- Für optimale Wahl von Hyperparametern
	- Anzahl verdeckter Schichten: 1-3 
	- Anzahl der Neuronen in der verdeckten Schicht: 3-30 
	- Art der Aktivierungsfunktion: Tansig, Sigmoid, ReLU
	-  Trainingsalgorithmus: L-BFGS, [[Stochastic Gradient Descent]], [[Stochastic Gradient Descent#Adam|Adam]]
- Rastersuche 
	- Durchsuche alle möglichen Parameterkombinationen 
	- Insgesamt 756 Evaluationen 
- Monte Carlo Methode 
	- Ziehe zufällig Parameterkombinationen ohne zurücklegen 
	- Breche nach 250 Evaluationen (Ziehungen) ab
### Gradientenverfahren
![[Gradientenverfahren zur Lösung von Optimierungsproblemen]]
### Kleinste Fehlerquadrate
![[Kleinste Fehlerquadrate (engl.) Least Squares]]
### Pattern Search
![[Pattern Search]]
### Reperaturverfahren
- Eine Möglichkeit zum Umgang mit Restriktionen 
- Optimierungsproblem wird zunächst als freies Problem gelöst, nicht unbedingt auf einer zulässigen Lösungsmenge 
- Danach sind typische Strategien: 
	- Runden von reell-wertigen Lösungen, um ganzzahlige Lösungen zu erhalten 
	- Projizieren auf das nächstgelegene zulässige $p$ 
	- Ersetzen von unzulässigen Werten
### Spezielle Optimierungsprobleme
- Lineares Programm (engl. Linear Program, LP) 
	- Lineare Zielfunktion, $p$ kontinuierlich mit Restriktionen $p>0$ und zusätzlichen weiteren Ungleichungsrestriktionen 
	- Lösungsverfahren: Simplex (exakt, durchsucht „Ecken“) 
- Ganzzahliges (Lineares) Programm (engl. Integer Linear Program, ILP) 
	- Lineare Zielfunktion, $p$ ganzzahlig mit Restriktionen $p>0$ und zusätzlichen weiteren Ungleichungsrestriktionen 
	- Beispiel: Routenplanung = Traveling Salesman Problem (Kante ja/nein als Werte von $p$) 
	- Lösungsverfahren: Branch-and-Bound (exakt, bei großen Problemen u.U. sehr langsam), Heuristiken 
- Gemischt-ganzzahliges Programm (engl. Mixed Integer Linear Program, MILP)
	- Manche Variablen sind ganzzahlig, andere reell

## Surrogatfunktionen
![[Surrogatfunktionen]]