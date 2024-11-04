Die Radiometrie ist ein wichtiger Teilbereich der Computergrafik, der sich mit der Messung und Berechnung der physikalischen Eigenschaften des Lichts befasst, um realistische Beleuchtungseffekte zu erzeugen. Die zugrunde liegende Motivation ist es, die physikalische Realität möglichst genau zu simulieren, insbesondere die Reflexion und Verteilung des Lichts auf verschiedenen Oberflächen.
#### Grundprinzipien
1. **Strahldichte**: In der Radiometrie beschreibt die Strahldichte (gemessen in W/m²sr), wie viel Lichtenergie in eine bestimmte Richtung von einer Oberfläche reflektiert wird und einen Betrachter erreicht. Dies ist besonders wichtig, um realistische Reflexionen darzustellen, da unterschiedliche Blickwinkel und Beleuchtungsrichtungen verschiedene Helligkeiten erzeugen.

2. **Flussdichte**: Die Flussdichte oder Bestrahlungsstärke (gemessen in W/m²) beschreibt, wie viel Lichtenergie pro Fläche auf eine Oberfläche auftrifft. Sie misst die Menge an einfallendem Licht unabhängig von der Richtung und ist entscheidend für die Berechnung von Schatten und die Intensität des einfallenden Lichts.
#### Photonentheorie des Lichts
Licht kann als Fluss von "Photonen" verstanden werden, die sich entlang von Strahlen bewegen. Ein Photon ist das kleinste Lichtquantum und hat Eigenschaften wie:
- **Geschwindigkeit** (abhängig vom Medium),
- **Wellenlänge** (entsprechend der Farbe),
- **Frequenz** und
- **Energie** (definiert durch \( E = hf \), wobei \( h \) das Plancksche Wirkungsquantum ist).
Diese Eigenschaften helfen dabei, die Energie und den Einfluss des Lichts auf die Wahrnehmung realistisch zu berechnen. Die Wellenlänge bestimmt beispielsweise, ob das Licht für das menschliche Auge sichtbar ist (ca. 380–700 nm) und welche Farbe es wahrgenommen wird.
#### Radiometrie und Wahrnehmung
Die Radiometrie unterscheidet sich von der Photometrie dadurch, dass sie physikalische Messungen des Lichts unabhängig vom menschlichen Auge berücksichtigt. Die Photometrie hingegen berücksichtigt die Empfindlichkeit des menschlichen Auges gegenüber verschiedenen Wellenlängen. Beispielsweise erscheinen bestimmte Farben heller oder dunkler, selbst wenn sie dieselbe Strahlungsenergie besitzen, da das menschliche Auge unterschiedliche Wellenlängen unterschiedlich stark wahrnimmt.
![[Pasted image 20241031110834.png|500]]
In der Grafik zeigt das sichtbare Spektrum, dass Licht nur einen kleinen Teil des elektromagnetischen Spektrums ausmacht und jede Wellenlänge einer spezifischen Farbe entspricht. Dies ist wichtig, um die verschiedenen Farbtöne und Intensitäten in Computergrafiken präzise darzustellen.

#### Energie und Leistung

Die Strahlungsleistung (gemessen in Watt, W) beschreibt, wie viel Energie eine Lichtquelle pro Zeit abgibt. Ein Beispiel wäre eine 100-Watt-Glühbirne, die Photonen mit einer bestimmten Frequenz und Energie emittiert. Die Leistung kann auf spezifische Wellenlängenbereiche beschränkt werden, wie z.B. zwischen 500 nm und 600 nm, um den Energiefluss innerhalb eines bestimmten Spektralbereichs zu analysieren.
