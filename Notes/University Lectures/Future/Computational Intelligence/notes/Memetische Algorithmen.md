> [!abstract] Definition
> Die Grundidee von memetischen Algorithmen ist es, die globalen Suchfähigkeiten von [[Evolutionary Algorithms (EA)|Evolutionären Algorithmen]]  mit lokalen Suchverfahren zu kombinieren. [[Evolutionary Algorithms (EA)|Evolutionären Algorithmen]]  sind gut darin, vielversprechende Regionen im Suchraum zu finden, aber sie konvergieren oft nicht effizient zu einem Optimum. Lokale Suchverfahren hingegen sind effizient in der Konvergenz zu einem Optimum innerhalb einer begrenzten Umgebung. Durch die Kombination beider Ansätze streben memetische Algorithmen danach, sowohl die Effizienz der globalen Suche als auch die Präzision der lokalen Optimierung zu verbessern.

- einfach einsetzbar bei reellwertigen $p$
- etwas mehr Nachdenken bei anderen Kodierungen
- können mit Reparaturverfahren zur Beachtung von Restriktionen kombiniert werden
## Varianten
1. in jeder Generation 
	- Memetische Algorithmen können in jeder Generation eines evolutionären Prozesses lokale Optimierungsverfahren anwenden.
	- Lamarcksche Evolution: Hierbei wird die lokal optimierte Lösung direkt in den Genotyp übernommen, was bedeutet, dass die Veränderungen, die durch lokale Suche entstehen, dauerhaft in den Nachkommen vererbt werden.
	- Baldwin-Evolution: Im Gegensatz dazu wird bei der Baldwin-Evolution die lokal optimierte Lösung nur in der Bewertung berücksichtigt, ohne den Genotyp dauerhaft zu verändern.
2. am Ende einer Suche nach der letzten Generation
	- Manchmal werden memetische Algorithmen erst am Ende einer Suche eingesetzt, insbesondere nach der letzten Generation, um die Lösungen weiter zu verfeinern.

## Lokale Suchstrategien für Reihenfolgen
Einzelne Positionen oder Gruppen von Positionen 
- um eine Position nach links oder rechts verschieben 
- nacheinander aus alle möglichen Positionen verschieben 
- ähnlich zu einem Mutationsoperator, deswegen manchmal auch „gezielte Mutation“ 
- Vorteil: kein „Übersehen“ von möglichen Lösungen
![[Pasted image 20240411122458.png#invert|500]]