## Begriffe
- **Strahlenoptik**: In der Computergrafik beschreibt die Strahlenoptik (Ray Optics) die Ausbreitung von Lichtstrahlen, die von Lichtquellen ausgehen und auf Oberflächen treffen. Diese Methode vereinfacht Lichtberechnungen, indem Licht als Strahl betrachtet wird, der sich linear ausbreitet und durch Reflexion, Brechung oder Absorption verändert werden kann.
- **[[Radiometrie]]**: Radiometrie ist die Wissenschaft, die sich mit der Messung von elektromagnetischer Strahlung befasst, also Licht und anderen Wellenlängen. In der Computergrafik beschreibt die Radiometrie den physikalischen Anteil des Lichts, wie Energie und Intensität. Hier werden Größen wie Strahlungsfluss (Energie pro Zeiteinheit) oder Strahlungsintensität verwendet.
- **Geometrische Optik als Standard in CG**: In der Computergrafik ist die geometrische Optik oft der Standard, da sie Lichtstrahlen als gerade Linien behandelt und Phänomene wie Reflexion und Brechung anhand einfacher geometrischer Regeln beschreibt. Diese Vereinfachung macht die Berechnung von Lichtverhältnissen und Schatten in Szenen effizient.
## Reflektanzspektren
- Die meisten Farben sind eine Mischung aus (vielen) Wellenlängen
## Perzeption vs Messung
- visuelles System des Menschen:
	- das menschliche Auge kann die spektrale Zusammensetzung von Licht nicht erfassen
	- das Auge macht eingeschränkte Messungen
	- das Auge passt sich (physikalisch) den äußeren Umständen an
	- … das Gehirn in vielfältiger Weise ebenso… 
- Wahrnehmungsphysiologie : Untersuchung der Funktionsweise der sinnlichen Wahrnehmung 
	- Sinnesrezeptoren, Signalübertragung, … bis hin zur Verarbeitung der Reizinformation im Zentralnervensystem 
	- der Wahrnehmungsprozess ist in seiner Gesamtheit (noch) nicht vollständig verstanden!
## Trichromatisches Farbsehen
Das trichromatische Farbsehen beschreibt die Art und Weise, wie das menschliche Auge Farben wahrnimmt. Es basiert auf drei Arten von Zapfen im Auge, die auf unterschiedliche Wellenlängen des Lichts reagieren:
1. **S-Zapfen**: Empfindlich für kurzwelliges (blaues) Licht.
2. **M-Zapfen**: Empfindlich für mittelwelliges (grünes) Licht.
3. **L-Zapfen**: Empfindlich für langwelliges (rotes) Licht.
Jeder dieser Zapfentypen reagiert auf einen bestimmten Bereich des Lichtspektrums, aber ihre Empfindlichkeitsbereiche überlappen sich teilweise. Dadurch können wir ein breites Spektrum an Farben wahrnehmen.
![[Pasted image 20241104080512.png|400]]
In der Grafik sind die Empfindlichkeitskurven für die drei Zapfentypen dargestellt. Die Höhe jeder Kurve zeigt, wie stark jeder Zapfentyp auf eine bestimmte Wellenlänge reagiert. Das menschliche Auge nimmt Farben wahr, indem es die Intensitäten der Reaktionen dieser drei Zapfentypen kombiniert. Die **Luminanz** (Helligkeit) ist dabei eine Funktion, die alle drei Zapfen kombiniert und so die wahrgenommene Helligkeit des Lichts beschreibt.
- Bei unterschiedlichen Wellenlängen und Intensitäten kann das Auge trotzdem die gleiche Antwort liefern -> [[Metamerismus]]
### Reizantwort
- Die Reizantwort (z.B. $s$) ist gegeben als: $$s = \int S(\lambda)P(\lambda)$$ wo $P(\lambda)$ das Spektrum der Lichtquelle und $S(\lambda)$ die $S$-Kurve ist.
### Der Prozess der Farbwahrnehmung durch Licht und Reflexion
![[Pasted image 20241104080930.png|400]]
- **Lichtquelle**: Das Licht enthält ein Spektrum verschiedener Wellenlängen.
- **Reflexion**: Das Objekt reflektiert bestimmte Wellenlängen in unterschiedlichem Maße, abhängig von seiner Farbeigenschaft. Die Reflexionskurve zeigt, welche Wellenlängen bevorzugt reflektiert werden.
- **Multiplikation**: Das reflektierte Spektrum ergibt sich durch die Multiplikation der ursprünglichen Lichtquelle mit der Reflexionskurve des Objekts.
- **L-Zapfen (rot)**, **M-Zapfen (grün)** und **S-Zapfen (blau)** reagieren jeweils auf bestimmte Wellenlängenbereiche.
- Durch die **Multiplikation** der reflektierten Wellenlängen mit den Empfindlichkeitskurven erhält man die Antworten jedes Zapfentyps auf das reflektierte Licht.
- **Integration**: Diese Antworten werden dann summiert (integriert), um die Gesamtantwort jedes Zapfentyps zu erhalten, was letztlich zur Wahrnehmung einer spezifischen Farbe führt.
### Sättigung und Intensität
Die Sättigung eines Farbtons hängt davon ab, wie breit oder schmal das Spektrum der Wellenlängen ist, das die Zapfen stimuliert. Ein sehr schmalbandiges Licht führt zu gesättigten Farben (stark definierte Farben), während ein breiteres Spektrum zu weniger gesättigten, aber helleren Farben führt.
![[Pasted image 20241104081345.png#invert|300]]
