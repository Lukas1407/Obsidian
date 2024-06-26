> [!abstract] Definition
>  Nachbildung der Veränderungs- und Selektionsprozesse in der Natur

## Ausgangssituation für eine Ingenieur: 
- technisches Problem mit potenziellen u.U. auch schon realisierten Lösungen 
- erste Ideen zur Bewertung dieser Lösungen, noch nicht unbedingt quantitativ 
- Wie komme ich zu einem Optimierungsproblem, dass durch Evolutionäre Algorithmen lösbar ist? 
	- Wie kann ich eine mögliche Lösung $p$ (geschickt) kodieren? 
	- Wie kann ich ein Optimierungsproblem (geschickt) entwerfen? 
	- Woher bekomme ich die Werte der Zielfunktion $Q(p)$ und Aussagen über die Verletzung eventueller Restriktionen im Optimierungsproblem? 
		- Berechnung direkt aus den Werten von $p$ (leider selten) 
		- Simulation mit den Werten von $p$ als Parameter des Simulationsmodells? 
		- Experiment?
![[Pasted image 20240411082142.png#invert|700]]

## Grundbegriffe Evolutionärer Algorithmen (VDI 3550)
- **Individuum**: Träger der genetischen Information, die alle Werte einer Lösung enthält 
- **Population**: Menge von Individuen, die miteinander Nachkommen erzeugen 
- **Chromosom**: Genkette, meist hat ein Individuum genau ein Chromosom (alle Informationen in ein Gen gebündelt; nicht wie in der Realität)
- **Gen**: Element eines Chromosoms, das ein oder mehrere Entscheidungsvariable umfasst 
- **Allel**: Konkrete(r) Wert(e) eines Gens 
- **Genotyp**: Individuum, betrachtet auf der Ebene der Werte seines Chromosoms
- **Phänotyp**: Ausprägung der durch den Genotyp codierten Eigenschaften, dient der Bestimmung der Fitness 
- **Fitness**: Bewertung hinsichtlich der Reproduktionstauglichkeit, Lösungsqualität 
- **Generation**: Iteration des Verfahrens 
- **Elter, Eltern**: an der Reproduktion beteiligte Individuen 
- **Nachkommen (Kinder)**: aus den genetischen Informationen der Eltern gebildete Individuen, die für die nächste Generation bestimmt sind 
- **Genetische Operatoren**: für Mutation zur Veränderung der Allele, für Rekombination zum Austausch von Erbinformationen der Eltern 
- **Klon**: Identische Kopie eines Individuums
- **Elitismus**: Eigenschaft von Selektionsmethoden, die ein Überleben des besten Individuums garantieren
- **Strategieparameter**: Parameter des Verfahrens 
- **Diversität**: Maß für die Verschiedenartigkeit der Genome 
- **Selektionsdruck**: Verhältnis der Wahrscheinlichkeit der Auswahl des besten Individuums zur durchschnittlichen Selektionswahrscheinlichkeit aller Individuen des Selektionspools ("Selektionspool" $⊆$ Population)
- **Suchraum**: Definitionsmenge der zu optimierenden Entscheidungsvariablen 
- **Zielfunktion**: zu optimierende Funktion, ergibt die Fitness; auch Güteoder Qualitätsfunktion

## Basisalgorithmus mit binärer Codierung
![[Pasted image 20240411082613.png#invert|500]]
### Startpopulation
- kodiert das Optimierungsproblem als Population mit $\mu$ Varianten einer Lösung $p$ als Individuen
- einfachste und älteste Variante: $p_i$ sind binär, $p$ ist ein Bitstring
- Individuen werden zufällig generiert, um eine Vielfalt an Lösungen zu gewährleisten
- Beispiel zu Bitkodierung:
	- 2 Variablen/Dimensionen der Lösung mit je 8 Bit Auflösung:
		- Segment 1 (Bits 1-8): Repräsentiert die Geschwindigkeit eines Fahrzeugs.
		- Segment 2 (Bits 8-16): Repräsentiert die Farbe des Fahrzeugs.
	- Individuum 1: 00110000 10010011 
	- Individuum 2: 00000001 10000000 
	- ... 
	- Individuum $\mu$: 00011001 01100000
- Die binäre Darstellung muss zum Optimierungsproblem passen, weil sie die Grundlage für die genetischen Operatoren bildet, die in evolutionären Algorithmen verwendet werden. Hier sind einige Gründe, warum die Anpassung wichtig ist:
	1. **Konsistenz**: Wenn die binäre Kodierung nicht zum Problem passt, kann es zu Inkonsistenzen kommen. Zum Beispiel, wenn die binäre Darstellung einer Variablen nicht mit dem tatsächlichen Wertebereich übereinstimmt, kann dies zu unerwarteten Ergebnissen führen.
	2. **Genetische Operatoren**: Die genetischen Operatoren wie **Kreuzung** (Crossover) und **Mutation** basieren auf der Manipulation von Bitstrings. Wenn die binäre Darstellung nicht korrekt ist, können diese Operatoren nicht effektiv arbeiten.
	3. **Suchraum**: Die binäre Kodierung definiert den Suchraum, in dem die evolutionären Algorithmen nach Lösungen suchen. Wenn dieser Suchraum nicht korrekt abgebildet ist, kann dies die Effizienz des Algorithmus beeinträchtigen.
	4. **Optimierung**: Das Ziel ist es, eine optimale Lösung zu finden. Wenn die binäre Darstellung nicht zum Problem passt, kann der Algorithmus Schwierigkeiten haben, die besten Lösungen zu identifizieren.
### Partnerwahl
- wird nur für Rekombination benötigt, es müssen also pro Rekombination zwei Eltern aus der aktuellen Population gezogen werden
- Auswahl hängt meistens von der Fitness ab, z.B. nach Rang (bessere Individuum sollten höhere Chance haben)
### Nachkommen
- Erzeugung neuer Nachkommen durch:
#### Mutation: 
- ein Klon eines Individuums wird zufällig verändert 
- Individuum: `00110000 10010011`
- Nachkomme (nach Mutation): `00110010 10010011`
#### Rekombination: 
- die Erbinformation von zwei durch die Partnerwahl ausgewählten Eltern wird kombiniert
- Elternteil 1: `00110000 10010011`
- Elternteil 2: `11001100 01100110`
##### Ein-Punkt-Crossover
- Nachkomme: `00110000 01100110` 
- Hier wurden die ersten 8 Bits vom Elternteil 1 und die letzten 8 Bits vom Elternteil 2 übernommen
- Keine Rücksicht auf Variablenstruktur, müsste man separat einbauen! Z.B in dem man nur Ein-Punkt-Crossover für einzelne Variablen macht
##### Uniform-Crossover
- Nachkomme: `00000001 100000011
- Jedes Bit wird zufällig von einem Elter übernommen
##### N-Punkt-Crossover
- $n$ zufällige Werte wählen ($1$..Länge von $p-1$)
- dort Individuen auftrennen 
- Beispiel: für $n=3$ bit den zufälligen Werten: $3, \ 7,\ 8$
- Nachkomme: `00100001 10010011`
#### Rekombination + Mutation
- ein durch Rekombination entstandener Nachkomme wird noch mutiert
### Bewertung
- Auswertung der Zielfunktion für jeden Nachkommen (Eltern ja schon bekannt)
### Akzeptanz der Nachkommen
- $\mu$ beste Lösungen übernehmen
### Prüfung der Abbruchbedingungen
- Zeit-, Qualitäts-, oder Generationenlimit erricht? 
### Fazit
- binäre Kodierung umständlich, dafür Nachkommen erzeugen einfach
- beschränkte Auflösung: Optimum kann nicht-kodierbar sein
- globales Optimum wird nicht immer gefunden unterschiedliche Ergebnisse in Abhängigkeit von: 
	- der (zufällig) gewählten Startpopulation und 
	- den (zufälligen) genetischen Operatoren

## Basisalgorithmus mit verschiedenen Codierung
### Startpopulation
- Einige Individuen können auch nicht-zufällig gewählt werden, wenn z.B. schon Wissen über mögliche Lösungen bekannt ist
	- Durch Expertenlösungen mit einer Heuristik 
	- Durch Ergebnisse einer vorherigen Optimierung, z.B. mit einem Gradientenverfahren 
	- <mark style="background: #FF5582A6;">Aber</mark>: nicht zu viele, sonst wird die genetische Vielfalt beeinträchtigt, Faustregel nicht mehr als 20%
		- -> Man kann z.B. in ein lokales Optimum der zu glaubten Lösungen laufen
- Strategieparameter: Größe der Population $\mu$ 
	- zu große Populationen: große Rechenzeiten 
	- zu kleine Populationen: frühzeitiger Verlust der genetischen Vielfalt, damit fehlender (oder sich nur sehr langsam durch große Mutationen einstellender) Erfolg
### Partnerwahl
- Kann zufällig geschehen
	- Schlecht
- Kann proportional nach Wert der Zielfunktion geschehen
	- Gute Individuen pflanzen sich sehr oft fort -> deren genetisches Material kommt sehr oft in der neuen Generation vor -> Verringerung der genetischen Vielfalt -> lokales Optimum
- Kann rang-basiert nach Wert der Zielfunktion geschehen
	- linear abhängig vom Rang (je steiler, desto höherer Selektionsdruck) 
	- nichtlinear
	- -> Schlechte Individuen haben immer noch realistische Chance sich fortzupflanzen
- Kann durch Turnier-Selektion geschehen
	- das beste Individuum unter den zufällig gezogenen Teilnehmern eines Turniers
- Kann durch Roulette-Selektion (stochastic sampling with replacement) geschehen
	- jedes Individuum hat ein Bereich auf dem Roulett-Tisch
		- Bestimmt die Wahrscheinlichkeit mit der er sich fortpflanzt
	- so viele Zufallszahlen ermitteln wie Partner benötigt werden
![[Pasted image 20240411111643.png#invert|600]]
- Kann durch Stochastic universal sampling geschehen
	- Zufallszahl ermitteln und Zeiger mit konstantem Abstand auswählen
	- So viele Zeiger wie Partner benötigt werden
	- -> Es werden auch schlechtere Individuen ausgewählt
	- -> Kein Problem von Inzest wie bei stochastic sampling with replacement
![[Pasted image 20240411111853.png#invert|600]]
#### Wie erfolgt die Auswahl?
- aus der gesamten Population 
- aus einer speziellen Nachbarschaft („Deme“ = Subpopulation)
	- Partnerwahl und Akzeptanz von Nachkommen findet nur in dieser Subpopulation statt 
	- Überlappungen von Subpopulation ermöglichen Migration anderer Subpopulation
	- Verschiedene Modelle: separate Subpopulationen, Ring, Torus, Inseln usw. 
	- Vorteile: 
		- Erhalt genetischer Varianz durch Subpopulationen 
		- bessere Parallelisierbarkeit der Optimierung 
	- Nachteile: 
		- aufwändigere Implementierung 
		- von den meisten Softwaretools nicht unterstützt
- Beispiel Nachbarschaftsmodell: Ringstruktur
![[Pasted image 20240411112138.png#invert|600]]
- Unterschiedliche Lösungen können sich lokal in der Deme durchsetzen
- Gute Lösungen beeinflussen benachbarte Demen wegen der Überlappung und setzten sich somit hier auch fort
### Nachkommen
- Erzeugung von $\lambda$ potentiellen Nachkommen
- $\lambda$ sollte deutlich größer als $\mu$ sein
	- Empfehlung $\lambda$>7 $\mu$ 
- bei Auswahl der evolutionären Operatoren u.U. schon auch an die Erzeugung zulässiger Lösungen denken (Einhaltung von Restriktionen) 
	- Evolutionäre Operatoren, wie Mutation und Rekombination, müssen so angepasst werden, dass die resultierenden Lösungen innerhalb des zulässigen Lösungsraums liegen und alle vorgegebenen Einschränkungen erfüllen. 
	- Dies ist wichtig, um sicherzustellen, dass die Lösungen, die durch den evolutionären Algorithmus generiert werden, gültig und anwendbar sind.
- bei Chromosomen mit Segmentbedeutungen: bei den Operatoren Segmentgrenzen beachten
	- Segmentbedeutungen: z.B. die ersten 8 Bit beschreiben Variable 1, ...
	- In evolutionären Algorithmen wie GLEAM (General Learning Evolutionary Algorithm and Method) kann ein Chromosom aus mehreren Segmenten bestehen, wobei jedes Segment eine bestimmte Bedeutung hat, z.B. ein Teil einer Lösung oder eine Gruppe von Parametern repräsentiert. Wenn evolutionäre Operatoren wie Mutation und Rekombination angewendet werden, ist es wichtig, die Grenzen dieser Segmente zu respektieren. Das bedeutet, dass Änderungen innerhalb eines Segments durchgeführt werden sollten, ohne die Struktur oder die Bedeutung der anderen Segmente zu beeinträchtigen.
#### Genetische Operatoren für reellwertige $p$
- Mutation: 
	- Addition einer normalverteilten Zufallszahl (kein Bias!) 
		- -> Hohe Wahrscheinlichkeit in der Umgebung
	- CMA-ES (Covariance Matrix Adaptation Evolution Strategy):
		- identifiziert die Richtungen im Suchraum, die wahrscheinlich zu einer Verbesserung der Zielfunktion führen
		- passt die Schrittweiten dynamisch an, um eine effiziente Exploration und Exploitation des Suchraums zu ermöglichen
		- Die Kovarianzmatrix, die die paarweisen Abhängigkeiten zwischen den Variablen in der Verteilung darstellt, wird kontinuierlich angepasst
			- Diese Anpassung ermöglicht es der CMA-ES, eine Art zweiter Ordnung Modell der zugrundeliegenden Zielfunktion zu lernen, ähnlich der Approximation der inversen Hesse-Matrix in der quasi-Newton-Methode.
		- Die CMA-ES ist besonders nützlich, wenn die Zielfunktion schlecht konditioniert ist, da sie weniger Annahmen über die Zielfunktion macht und nur ein Ranking der Kandidatenlösungen benötigt
- Diskrete Rekombination:
	- Tausch von $p_i$-Werten von Individuen
- Intermediäre Rekombination:
	- gewichtetes Mitteln der Eltern:$$\begin{align} p_{N,i}=\beta_{i}*p_{E1,i}+(1-\beta_{i})*p_{E2,i} \\ \beta_{i}\in \left[0,1 \right] \end{align}$$
	- -> Führt dazu, dass sich Individuen angleichen! -> reduziert genetische Varianz
	- Lösung: $$\beta_{i}\in \left[-d,1+d \right]$$
		- -> Verhindert exaktes angleichen
#### Genetische Operatoren für Reihenfolgen
- Ziel: Auch nach Operator eine gültige Reihenfolge erhalten 
- Rekombination: 
	- Teilsequenz von Elter A übernehmen
	- Aufträge aus Elter B rausstreichen 
	- Teilsequenz in Elter B einfügen 
	- Viele Varianten mit ähnlichen Strategien 
	- Beispiel:
![[Pasted image 20240411115419.png#invert|500]]
- Mutation: 
	- Einzelne Aufträge oder Gruppen von Aufträgen 
		- durchschieben 
		- in der Richtung invertieren usw.
	- Beispiel:
![[Pasted image 20240411115403.png#invert|500]]
### Bewertung
- Auswertung der Zielfunktion und Restriktionen für alle neu erzeugten Nachkommen
- Umgang mit Restriktionen: 
	- Evtl. auch Lösungen zulassen, die Restriktionen knapp verletzten (constraint tolerance)
	- Erlaubt höhere genetische Varianz
	- Am Schluss muss natürlich die Restriktionen erfüllt sein
		- Z.B. durch filtern korrekte Lösungen finden
- Evtl. Begünstigung genetischer Varianz (genotypische Unterschiede zu besseren Individuen) bei ähnlichen Werten der Zielfunktion 
- Bei aufwändigen Bewertungen Lösungsarchiv führen
	- -> Schon mal erhaltene Lösungen müssen nicht nochmal berechnet werden
### Akzeptanz der Nachkommen
- $(\mu,\lambda)$-Strategie: 
	- Auswahl der besten $\mu$ Individuen unter den Nachkommen für die nächste Generation
	- -> <mark style="background: #FF5582A6;">Eltern werden nie ausgewählt!</mark>
	- -> Beste Individuen können verloren gehen
- $(\mu,\mu+\lambda)$-Strategie: 
	- Auswahl der besten $\mu$ Individuen unter den Eltern der vorherigen Generation und den Nachkommen für die nächste Generation 
	- beste Individuen bleiben (fast) immer erhalten
	- ohne Einschränkungen (elitär)
	- mit Einschränkungen: 
		- max. Alter der Eltern (nicht elitär) 
		- max. Anzahl Eltern (elitär für Anzahl>0) 
		- Nachkommen konkurrieren nur gegen ihre eigenen Eltern (elitär) 
		- Konkurrenz nur im Deme bei Populationsmodellen
### Prüfung der Abbruchbedingungen
- Häufig eine Kombination aus mehreren Bedingungen (Strategieparameter) 
- -> zum Reduzieren des Rechenaufwandes
- Qualitätslimit: Erreichen eines akzeptablen Wertes der Zielfunktion 
- Erkennung und Vermeidung von Stagnation 
	- Zeitlimit: Abbruch nach bestimmter Rechenzeit 
	- Generationslimit: Abbruch nach einer bestimmten Zahl der Generationen 
	- keine (wesentliche) Verbesserung seit einer bestimmten Anzahl von Generationen (MATLAB: stall generations)

## Individuen 
- Werte einer Lösung $p$ eines Optimierungsproblems müssen geeignet als Chromosom kodiert werden, dazu passen dann immer bestimmte Varianten von EAs: 
	- Bitstrings: Genetische Algorithmen (GA) 
	- Ganzzahlige Kodierung: Genetische Algorithmen (GA) 
	- Reelle Zahlen: Evolutionsstrategie (ES) 
	- Komplexere Strukturen wie Bäume, Programmcodebestandteile: Genetische Programmierung (GP) 
- Mögliche Erweiterung: Strategieparameter des ganzen EAs (z.B. Populationsgröße, Mutationswahrscheinlichkeit usw.) mit in Individuen kodieren und werden damit mit optimiert

## Geeignete Kodierungen
- Die Wahl der Codierung hängt stark vom spezifischen technischen Problem ab und davon, wie die Lösungen am besten repräsentiert werden können. Es ist wichtig, dass die Codierung gut zum Optimierungsproblem passt und eine effiziente Suche im Lösungsraum ermöglicht.
- <mark style="background: #FFB86CA6;">Binäre Codierung</mark>: Dies ist die einfachste Form der Codierung, bei der Lösungen als Bitstrings dargestellt werden. Sie eignet sich für Probleme, bei denen Entscheidungen als Ja/Nein oder An/Aus dargestellt werden können.
	- **Problem**: Optimierung der Konfiguration eines Kommunikationsnetzwerks.
	- **Kodierung**: Jedes Bit könnte den Zustand eines Schalters im Netzwerk repräsentieren, wobei `1` für “ein” und `0` für “aus” steht.
	- **Beispiel**: `11001010` könnte ein Netzwerk mit 8 Schaltern darstellen, wobei die Schalter 1, 2, 5 und 7 aktiviert sind.
- <mark style="background: #FFB86CA6;">Ganzzahlige und reelle Zahlen</mark>: Für Probleme, die eine höhere Präzision erfordern, wie z.B. die Optimierung von geometrischen Parametern, können ganzzahlige oder reelle Zahlen verwendet werden.
	- **Problem**: Auswahl der optimalen Anzahl von Servern in einem Rechenzentrum.
	- **Kodierung**: Jede Ganzzahl könnte die Anzahl der Server in einem bestimmten Cluster repräsentieren.
	- **Beispiel**: `[4, 10, 7]` könnte ein Rechenzentrum mit drei Clustern darstellen, wobei der erste Cluster 4 Server, der zweite 10 Server und der dritte 7 Server hat.
	- **Problem**: Optimierung der Flügelform eines Flugzeugs zur Verbesserung der Aerodynamik.
	- **Kodierung**: Reelle Zahlen könnten verschiedene geometrische Parameter wie Spannweite, Wölbung und Dicke des Flügels repräsentieren.
	- **Beispiel**: `[34.5, 0.32, 1.2]` könnte einen Flügel mit einer Spannweite von 34,5 Metern, einer Wölbung von 0,32 und einer Dicke von 1,2 Metern darstellen.
- <mark style="background: #FFB86CA6;">Gemischte Codierung</mark>: Manchmal ist eine Kombination aus binären, ganzzahligen und reellen Codierungen sinnvoll, um verschiedene Aspekte eines technischen Problems zu repräsentieren.
	- **Problem**: Entwurf eines autonomen Fahrzeugs mit verschiedenen Sensoren und Komponenten.
	- **Kodierung**: Eine Kombination aus binären, ganzzahligen und reellen Zahlen könnte verwendet werden, um den Typ, die Anzahl und die Spezifikationen der Sensoren und Komponenten zu kodieren.
	- **Beispiel**: `{Sensoren: [1, 0, 1], Anzahl: [2, 4], Spezifikationen: [0.75, 1.5]}` könnte ein Fahrzeug darstellen, das mit Radar (1), ohne LIDAR (0) und mit Kamera (1) ausgestattet ist, zwei Radarsensoren und vier Kameras hat, und die Spezifikationen beziehen sich auf die Genauigkeit der Sensoren.
- <mark style="background: #FFB86CA6;">Genetische Programmierung</mark>: Für noch komplexere Probleme, bei denen die Lösungen als Programme oder Bäume codiert werden können, wird die genetische Programmierung verwendet.
	- **Problem**: Automatisches Erstellen von Computerprogrammen, die bestimmte Aufgaben erfüllen.
	- **Kodierung**: Die Lösungen werden als Baumstrukturen kodiert, die aus Funktionen und Terminals (Variablen und Konstanten) bestehen.
	- **Beispiel**: Ein Baum könnte ein einfaches Programm darstellen, das die Summe zweier Zahlen berechnet, wobei die Knoten des Baums Operationen wie Addition und die Blätter die Eingabewerte darstellen.

## Evolutionäre Algorithmen für multikriterielle Optimierung
- **Multikriterielle Optimierung**: Bei dieser Art der Optimierung werden mehrere Zielkriterien gleichzeitig berücksichtigt, um eine Reihe von optimalen Lösungen zu finden, die als Pareto-optimal bekannt sind.
- Ergebnis ist immer eine ganze Population
- Bewertung muss darauf achten, alle Paretooptimalen Lösungen zu erhalten
- **NSGA-II**: Der Non-dominated Sorting Genetic Algorithm II ist ein bekannter Algorithmus zur Lösung multikriterieller Optimierungsprobleme, der darauf abzielt, alle Pareto-optimalen Lösungen zu erhalten.
- oftmals große Populationen sinnvoll

## Vereinfachungen gegenüber biologischen EA
Reduktion der Komplexität u.a. durch: 
- Verzicht auf mehrere Arten 
- extrem vereinfachte Umwelteinflüsse als „Bewertung“ 
- meist keine Unterscheidung in Geschlechter 
- Generationen meist zeitsynchron 
- oft Verzicht auf lokale Subpopulationen (hier gibt es aber Lösungen)

## Kommentare
- Strategien zur Adaption: Anpassung von Parametern wie Mutationswahrscheinlichkeiten, Schrittweiten usw. zur Laufzeit
- Vergleiche auf unsaubere Initialisierungen in der Nähe der optimalen Lösung prüfen (dann ist es einfach), z.B. für ein Individuum der Startpopulation
- Durch geschicktere Kodierungen bei binären Problemen, wie z.B. Gray-Kodes, können die Erfolgsaussichten von Operatoren verbessert werden, indem der Hamming-Abstand zwischen Lösungen optimiert wird.
- Fazit: 
	- gutes Universalverfahren für viele Probleme, aber kein Zaubertool 
	- relativ hoher Rechenaufwand 
	- Schwächen auf der „letzten Meile“ in der Nähe von Optima