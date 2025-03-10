## Begriffe
- **Strahlenoptik**: In der Computergrafik beschreibt die <mark style="background: #FFB86CA6;">Strahlenoptik die Ausbreitung von Lichtstrahlen</mark>, die <mark style="background: #FFB86CA6;">von Lichtquellen ausgehen und auf Oberflächen treffen</mark>. Diese Methode vereinfacht Lichtberechnungen, indem Licht als Strahl betrachtet wird, der sich linear ausbreitet und durch Reflexion, Brechung oder Absorption verändert werden kann.
- **[[Radiometrie]]**: Radiometrie ist die Wissenschaft, die sich mit der Messung von elektromagnetischer Strahlung befasst, also Licht und anderen Wellenlängen. In der Computergrafik beschreibt die Radiometrie den physikalischen Anteil des Lichts, wie Energie und Intensität. Hier werden Größen wie Strahlungsfluss (Energie pro Zeiteinheit) oder Strahlungsintensität verwendet.
- **Geometrische Optik als Standard in CG**: In der Computergrafik ist die geometrische Optik oft der Standard, da sie <mark style="background: #FFB86CA6;">Lichtstrahlen als gerade Linien behandelt und Phänomene wie Reflexion und Brechung anhand einfacher geometrischer Regeln beschreibt</mark>. Diese Vereinfachung macht die Berechnung von Lichtverhältnissen und Schatten in Szenen effizient.
## Reflektanzspektren
- Die meisten Farben sind eine Mischung aus (vielen) Wellenlängen
## Perzeption vs Messung
- visuelles System des Menschen:
	- das menschliche Auge kann die <mark style="background: #FFB86CA6;">spektrale Zusammensetzung von Licht nicht erfassen</mark>
	- das Auge <mark style="background: #FFB86CA6;">macht eingeschränkte Messungen</mark>
	- das Auge <mark style="background: #FFB86CA6;">passt sich (physikalisch) den äußeren Umständen</mark> an
	- … das Gehirn in vielfältiger Weise ebenso… 
- Wahrnehmungsphysiologie : <mark style="background: #FFB86CA6;">Untersuchung der Funktionsweise der sinnlichen Wahrnehmung</mark> 
	- Sinnesrezeptoren, Signalübertragung, … bis hin zur Verarbeitung der Reizinformation im Zentralnervensystem 
	- der Wahrnehmungsprozess ist in seiner Gesamtheit (noch) nicht vollständig verstanden!
## Trichromatisches Farbsehen
Das trichromatische Farbsehen beschreibt die Art und Weise, wie das menschliche Auge Farben wahrnimmt. Es basiert auf drei Arten von Zapfen im Auge, die auf unterschiedliche Wellenlängen des Lichts reagieren:
1. **S-Zapfen**: Empfindlich für <mark style="background: #FFB86CA6;">kurzwelliges (blaues) Licht</mark>.
2. **M-Zapfen**: Empfindlich für <mark style="background: #FFB86CA6;">mittelwelliges (grünes) Licht</mark>.
3. **L-Zapfen**: Empfindlich für <mark style="background: #FFB86CA6;">langwelliges (rotes) Licht</mark>.
Jeder dieser Zapfentypen reagiert auf einen bestimmten Bereich des Lichtspektrums, aber ihre <mark style="background: #FFB86CA6;">Empfindlichkeitsbereiche überlappen sich teilweise. Dadurch können wir ein breites Spektrum an Farben wahrnehmen</mark>.
![[Pasted image 20241104080512.png|400]]
In der Grafik sind die Empfindlichkeitskurven für die drei Zapfentypen dargestellt. Die Höhe jeder Kurve zeigt, wie stark jeder Zapfentyp auf eine bestimmte Wellenlänge reagiert. <mark style="background: #FFB86CA6;">Das menschliche Auge nimmt Farben wahr, indem es die Intensitäten der Reaktionen dieser drei Zapfentypen kombiniert</mark>. Die **Luminanz** (Helligkeit) ist dabei eine Funktion, die alle drei Zapfen kombiniert und so die wahrgenommene Helligkeit des Lichts beschreibt.
- Bei <mark style="background: #FFB86CA6;">unterschiedlichen Wellenlängen und Intensitäten kann das Auge trotzdem die gleiche Antwort liefern</mark> -> [[Metamerismus]]
### Reizantwort
- Die Reizantwort (z.B. $s$) ist gegeben als: $$s = \int S(\lambda)P(\lambda)$$ wo $P(\lambda)$ das <mark style="background: #FFB86CA6;">Spektrum der Lichtquelle</mark> und $S(\lambda)$ die $S$-<mark style="background: #FFB86CA6;">Kurve</mark> ist.
### Der Prozess der Farbwahrnehmung durch Licht und Reflexion
![[Pasted image 20241104080930.png|600]]
- **Lichtquelle**: Das Licht enthält ein Spektrum verschiedener Wellenlängen.
- **Reflexion**: Das Objekt reflektiert bestimmte Wellenlängen in unterschiedlichem Maße, abhängig von seiner Farbeigenschaft. Die Reflexionskurve zeigt, welche Wellenlängen bevorzugt reflektiert werden.
- **Multiplikation**: Das reflektierte Spektrum ergibt sich durch die Multiplikation der ursprünglichen Lichtquelle mit der Reflexionskurve des Objekts.
- **L-Zapfen (rot)**, **M-Zapfen (grün)** und **S-Zapfen (blau)** reagieren jeweils auf bestimmte Wellenlängenbereiche.
- Durch die **Multiplikation** der reflektierten Wellenlängen mit den Empfindlichkeitskurven erhält man die Antworten jedes Zapfentyps auf das reflektierte Licht.
- **Integration**: Diese <mark style="background: #FFB86CA6;">Antworten werden dann summiert (integriert), um die Gesamtantwort</mark> jedes Zapfentyps zu erhalten, was letztlich zur Wahrnehmung einer spezifischen Farbe führt.
### Sättigung und Intensität
<mark style="background: #FFB86CA6;">Die Sättigung eines Farbtons hängt davon ab, wie breit oder schmal das Spektrum der Wellenlängen ist, das die Zapfen stimuliert</mark>. Ein sehr schmalbandiges Licht führt zu <mark style="background: #FFB86CA6;">gesättigten Farben</mark> (stark definierte Farben), während ein <mark style="background: #FFB86CA6;">breiteres Spektrum zu weniger gesättigten, aber helleren Farben führt</mark>.
![[Pasted image 20241104081345.png#invert|300]]
