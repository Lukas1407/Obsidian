> [!abstract] Definition
> Fuzzy-Systeme sind ein Teilgebiet der künstlichen Intelligenz, das sich mit der Verarbeitung von unscharfen und ungenauen Informationen befasst.

![[Pasted image 20240413073017.png#invert|500]]
- [[Fuzzifizierung]]
- [[Inferenz bei Fuzzy-Systemen]]
- [[Defuzzyfizierung]]
## Arten von Unschärfe
### Zufälligkeit (engl. randomness)
- Analyse durch Wahrscheinlichkeitstheorie
- Ereignisse sind wahr oder falsch, Auftreten ist zufällig 
- Beispiele: 
	- Würfel 
	- Wahrscheinlichkeitsdichtefunktion von Messfehlern
### Unschärfe (engl. fuzziness)
- Analyse durch [[Fuzzy-Logik]]
- Ereignisse sind nicht nur wahr oder falsch, Zwischengrößen sind zulässig
- Beispiele: 
	- "große Menschen"
	- "heiße Temperatur"
	- Weinschorle (Mischung aus Wein und Wasser)
## Anwendungen von Fuzzy-Systemen
- Für Fuzzy-Regelungen eventuell geeignet: 
	- Regelstrecken ohne bekanntes mathematisches Modell mit geringen Sicherheits- und Güteanforderungen, insbesondere bei mehreren Eingangsgrößen und bekannten menschlichen Regelstrategien 
	- Nichtlineare Regelstrecken mit nur an bestimmten Stützpunkten identifiziertem Modell (Fuzzy-adaptive Regler) – Unscharfe Umschaltung von Teilreglern 
- Für Fuzzy-Regelungen ungeeignet:
	- Regelstrecken mit bekanntem oder identifizierbarem linearen Modell 
		- -> Besser: Lineare Regler wie PID- und Zustandsregler
	- Regelstrecken mit bekanntem nichtlinearen Modell 
		- -> Besser: Nichtlineare Regler, z.B. basierend auf exakter Linearisierung oder Modellprädiktive Regler
- Fuzzy-Clustering (Vorlesung Datenanalyse für Ingenieure): 
	- Verallgemeinerung der Clusteranalyse 
	- für jedes Objekt werden Zugehörigkeitswerte zwischen Null und Eins zu allen Clustern (Klassen) ermittelt 
	- Zugehörigkeitswerte sind umso größer, je geringer der Abstand zum Referenzpunkt des jeweiligen Clusters ist 
	- unterschiedliche Abstandsmaße (z.B. euklidischer Abstand) und Referenzpunkte (z.B. Klassenmittelpunkt) können verwendet werden 
- Fuzzy-Klassifikation (z.B. für Fehlerdiagnose): 
	- Verallgemeinerung der Klassifikation 
	- für jedes Objekt werden Zugehörigkeitswerte zwischen Null und Eins zu allen Klassen ermittel
### Entwurf von Fuzzy-Systemen
- Befragung von Experten 
- Einfache Heuristiken (z.B. zum Entwurf von Zugehörigkeitsfunktionen) 
- Lernen aus Daten (Auswahl) 
	- Aufstellen und Bewerten von Regeln, z.B. auf Basis von Entscheidungsbäumen (Vorlesung "Datenanalyse für Ingenieure“) 
	- Clustern zum Entwurf von Zugehörigkeitsfunktionen (Vorlesung "Datenanalyse für Ingenieure") 
	- Optimieren von Fuzzy-Systemen mit Evolutionären Algorithmen 
	- für Takagi-Sugeno-Systeme (Fuzzy-Modelle oder Fuzzy-Regelungen): 
		- Aufteilen des Zustandsraums in geeignete Bereiche 
		- Identifizieren der Konklusionen mit Hilfe von linearen Methoden