> [!abstract] Definition
>   Die Fuzzifizierung ist ein zentraler Prozess in Fuzzy-Systemen, der präzise Eingangswerte in unscharfe Mengen umwandelt

## Linguistischer Term
- Dies sind qualitative Beschreibungen von Eigenschaften wie “WARM”, “KALT”, “HEISS”. 
- Sie werden verwendet, um komplexe oder vage Konzepte zu beschreiben, die schwer quantifizierbar sind.

## Zugehörigkeitsfunktion (ZGF) $\mu(x)$
- Eine Funktion, die jedem Wert $x$ einen Zugehörigkeitsgrad zwischen 0 und 1 zuordnet
- Dieser Grad gibt an, inwieweit der Wert $x$ dem linguistischen Term entspricht.
![[Pasted image 20240412104626.png#invert|400]]
## Standardpartitionen
- Eine Methode zur Aufteilung des Wertebereichs einer Variablen in unscharfe Mengen, wobei die Summe der Zugehörigkeitsgrade aller Mengen für jeden Wert immer Eins ergibt. 
- Es gibt immer nur maximal zwei benachbarte ZGFs mit Werten größer Null, was Überschneidungen begrenzt.
- Bei dreieckförmigen ZGFs sind die Randfunktionen oft trapezförmig, um den Bereich der vollständigen Zugehörigkeit (Zugehörigkeitsgrad 1) zu erweitern.
- Die Vorteile der Fuzzifizierung und der Wahl der ZGF-Parameter sind:
	- **Eindeutigkeit**: Die ZGFs können durch wenige Parameter wie Form, Breite und Zentrum beschrieben werden.
	- **Interpretierbarkeit**: Linguistische Terme sind intuitiv und ermöglichen eine einfache Interpretation der ZGFs.
	- **Rechenvorteile**: Einfache mathematische Operationen mit den Zugehörigkeitsgraden sind möglich.
## Wahl der ZGF-Parameter 
- Expertenwissen
	- Experten definieren, was unter den linguistischen Termen zu verstehen ist, und ihre Einschätzungen können in Standardpartitionen überführt werden.
- Einfache Heuristiken
	- Festlegung der kleinsten und größten möglichen Werte.
	- andere Stützpunkte so verteilen, dass entweder 
		- die Abstände zwischen den Stützpunkten gleich sind ("äquidistant") 
		- ungefähr gleich viele Messwerte ("Datentupel") pro Term zu erwarten sind ("äquifrequent")
- kompliziertere datenbasierte Entwurfsverfahren

## Beispiel für Fuzzifizierung
- Für die Eingangsgröße $T$ (Temperatur) mit dem Messwert 17.5°C könnten die Zugehörigkeitsgrade sein:
	- $\mu_{ZUWARM}$ = 0.0 (nicht zu warm)
	- $\mu_{ANGENEHM}$ = 0.5 (halbwegs angenehm)
	- $\mu_{ZU KALT}$ = 0.5 (halbwegs zu kalt)
- Für die Eingangsgröße $DT$ (Temperaturänderung) mit dem Messwert -0.3 K/min könnten die Zugehörigkeitsgrade sein:
	- $\mu_{POS}$ = 0.0 (keine positive Änderung)
	- $\mu_{NULL}$ = 0.7 (hauptsächlich keine Änderung)
	- $\mu_{NEG}$ = 0.3 (teilweise negative Änderung)

Diese Zugehörigkeitsgrade werden dann in der [[Inferenz bei Fuzzy-Systemen|Inferenzphase]] des [[Fuzzy-Systeme|Fuzzy-Systems]] verwendet, um Entscheidungen zu treffen oder Ausgaben zu generieren.