## RGB-Farbraum
Der **RGB-Farbraum** ist ein Farbmodell, das auf den Primärfarben **Rot**, **Grün** und **Blau** basiert. Diese Farben werden in verschiedenen Kombinationen und Intensitäten gemischt, um eine Vielzahl von Farben zu erzeugen:
- Jede der drei Primärfarben wird durch eine <mark style="background: #FFB86CA6;">spezifische Wellenlänge</mark> dargestellt (<mark style="background: #FFB86CA6;">z.B. 645 nm für Rot, 525 nm für Grün und 444 nm für Blau</mark>).
- Die Farben im RGB-Farbraum können als Kombination von Anteilen der Primärfarben beschrieben werden, etwa $C = rR + gG + bB$, wobei $r$, $g$ und $b$ Tristimuluswerte sind, die in einem Bereich von 0 bis 1 liegen.
- Der RGB-Farbraum ist der Standard für **Bildschirme und elektronische Anzeigen**, da sie Licht ausstrahlen und somit Farben durch additive Mischung erzeugen.
- Die **Luminanzapproximation** $Y = 0.3r + 0.59g + 0.11b$ wird verwendet, um die wahrgenommene Helligkeit zu berechnen.
## CMY(K)-Farbraum
Der **CMY-Farbraum** ist das Gegenstück zum RGB-Farbraum und basiert auf den Farben **Cyan**, **Magenta** und **Gelb**:
- Dieser Farbraum arbeitet mit <mark style="background: #FFB86CA6;">subtraktiver Farbmischung</mark> und wird für Drucker und Maltechniken verwendet, da Pigmente und Tinten Licht absorbieren statt es zu emittieren.
- Jede Primärfarbe des CMY-Modells absorbiert ein Drittel des Spektrums:
  - **Cyan** absorbiert Rot.
  - **Magenta** absorbiert Grün.
  - **Gelb** absorbiert Blau.
- In der Praxis wird oft eine vierte Farbe, <mark style="background: #FFB86CA6;">Schwarz (K), hinzugefügt</mark>, um tiefere Schwarztöne zu erzeugen und die Menge an Tinte zu reduzieren.
- Die <mark style="background: #FFB86CA6;">Formel für Schwarz im CMYK-Modell ist</mark> $K = \min(C, M, Y)$, und die anderen Farben werden entsprechend angepasst.
![[Pasted image 20241104082517.png|100]]
## HSV-Farbraum
Der **HSV-Farbraum** beschreibt Farben basierend auf **Farbton (Hue)**, **Sättigung (Saturation)** und **Helligkeit (Value)**:
- **Hue (Farbton)**: Der Farbwinkel auf dem Farbkreis (z.B. Rot, Grün, Blau).
- **Saturation (Sättigung)**: Die Intensität oder Reinheit der Farbe.
- **Value (Helligkeit)**: Die Helligkeit der Farbe, wobei 0 Schwarz und der maximale Wert eine helle Farbe ist.
- Das HSV-Modell wird oft als Zylinder oder Kegel dargestellt und ist <mark style="background: #FFB86CA6;">besonders nützlich für Benutzeroberflächen, da es die Farbwahl intuitiver macht.</mark>
- Es gibt eine einfache <mark style="background: #FFB86CA6;">Konvertierung zwischen RGB und HSV</mark>, wobei $V = \max(r, g, b)$, $S = (\text{max} - \text{min}) / \text{max}$, und der Farbton $H$ basierend auf dem größten Wert der RGB-Komponenten berechnet wird.
![[Pasted image 20241104082531.png|200]]

## XYZ-Farbraum
Der **XYZ-Farbraum** wurde von der **CIE (Commission Internationale de l'Eclairage)** entwickelt, um einen <mark style="background: #FFB86CA6;">standardisierten Farbraum</mark> zur Verfügung zu stellen, <mark style="background: #FFB86CA6;">der alle wahrnehmbaren Farben beschreibt und eine einfache Konvertierung zwischen verschiedenen Farbräumen ermöglicht</mark>. Hier sind die Hauptpunkte:
1. **Ziel des XYZ-Farbraums**:
   - Der XYZ-Farbraum dient als Standard für die Konvertierung zwischen Farbräumen und als Referenz zur Beschreibung aller für das menschliche Auge wahrnehmbaren Farben.
   - Er <mark style="background: #FFB86CA6;">basiert auf den sogenannten Color Matching Functions</mark> und <mark style="background: #FFB86CA6;">repräsentiert Farben unabhängig von spezifischen Geräten</mark> oder Systemen.
2. **Eigenschaften des XYZ-Farbraums**:
   - Die Color Matching Functions für den XYZ-Farbraum, dargestellt als $\overline{x}(\lambda)$, $\overline{y}(\lambda)$ und $\overline{z}(\lambda)$, sind positiv und <mark style="background: #FFB86CA6;">wurden so festgelegt, dass sie eine lineare Kombination der ursprünglichen RGB Color Matching Functions bilden, jedoch ohne negative Werte</mark>. Das erleichtert Berechnungen.
   - Die <mark style="background: #FFB86CA6;">Y-Komponente wurde so gewählt, dass sie die Helligkeit (Luminanz) repräsentiert</mark> und somit der Luminanzkurve $\overline{y}(\lambda)$ entspricht.
   - Die <mark style="background: #FFB86CA6;">Z-Komponente entspricht in etwa der Empfindlichkeit des S-Zapfens für Blau</mark>, während <mark style="background: #FFB86CA6;">X die Rot- und Grünbereiche abdeckt</mark>.
![[Pasted image 20241104083429.png|300]]
3. **Berechnung der XYZ-Tristimuluswerte**:
   - Die XYZ-Werte sind die **Tristimuluswerte** und werden wie folgt berechnet:
     $$
     X = \int \overline{x}(\lambda) P(\lambda) \, d\lambda, \quad Y = \int \overline{y}(\lambda) P(\lambda) \, d\lambda, \quad Z = \int \overline{z}(\lambda) P(\lambda) \, d\lambda
     $$
   - Hierbei gibt $P(\lambda)$ das Spektrum des Lichts an, das wir messen möchten. Die Integrale ergeben die Anteile der XYZ-Primärfarben, die für die Darstellung einer bestimmten Farbe benötigt werden.
4. **Übertragbarkeit auf RGB**:
   - <mark style="background: #FFB86CA6;">Der XYZ-Farbraum kann durch eine lineare Transformation in den RGB-Farbraum umgewandelt werden</mark>, was eine einfache Konvertierung ermöglicht. Diese Transformation wird <mark style="background: #FFB86CA6;">durch eine Matrix dargestellt</mark>, die das XYZ-System in RGB-Werte übersetzt. Die Umkehrung dieser Matrix (für die Umrechnung zurück) kann jedoch negative Werte enthalten, was bedeutet, dass manche XYZ-Werte auf nicht darstellbare RGB-Werte abgebildet werden (da sie physisch nicht möglich sind).
5. **Imaginary Primaries**:
   - Die Festlegung der <mark style="background: #FFB86CA6;">XYZ Color Matching Functions führt zu "imaginären Primärfarben" </mark>(übersättigte Farben), die physisch nicht realisierbar sind. Das <mark style="background: #FFB86CA6;">bedeutet, dass der XYZ-Farbraum Farben beschreibt, die jenseits des für das Auge sichtbaren Spektrums liegen, um den gesamten wahrnehmbaren Farbbereich mathematisch abzudecken.</mark>
### Chromatizität: Vom CIE XYZ zum CIE xyY Farbraum
Der <mark style="background: #FFB86CA6;">CIE xyY-Farbraum ist eine Transformation des XYZ-Farbraums, die es ermöglicht, Farben unabhängig von ihrer Helligkeit zu beschreiben</mark>. Das Ziel dieser Transformation ist es, nur den <mark style="background: #FFB86CA6;">Farbton und die Sättigung darzustellen, ohne die Helligkeit zu berücksichtigen.</mark>
#### Beobachtungen im XYZ-Farbraum
- Im XYZ-Farbraum repräsentieren alle Werte $kX, kY, kZ$ (wobei $k > 0$ ein Skalierungsfaktor ist) <mark style="background: #FFB86CA6;">dieselbe Farbe mit unterschiedlicher Intensität</mark>. Das bedeutet, dass <mark style="background: #FFB86CA6;">diese Werte lediglich in ihrer Helligkeit variieren, aber denselben Farbton und dieselbe Sättigung haben.</mark>
#### Normalisierung und Projektion
1. **Normalisierung**: <mark style="background: #FFB86CA6;">Um nur die Farbanteile darzustellen, wird eine Normalisierung durchgeführt</mark>, bei der $X + Y + Z = 1$ gesetzt wird. <mark style="background: #FFB86CA6;">Dadurch erhält man einen Farbpunkt, der von der Helligkeit unabhängig ist und nur Farbton und Sättigung repräsentiert</mark>.
2. **Projektion auf die XY-Ebene**: Nach der Normalisierung wird der Punkt auf die XY-Ebene projiziert (<mark style="background: #FFB86CA6;">Z wird weggelassen</mark>). Somit erhalten wir die sogenannten **x**- und **y**-Werte, die die <mark style="background: #FFB86CA6;">Chromatizität (Farbton und Sättigung) beschreiben</mark>.
Die Formeln für diese Transformation sind:
$$
x = \frac{X}{X + Y + Z}, \quad y = \frac{Y}{X + Y + Z}, \quad z = \frac{Z}{X + Y + Z} = 1 - x - y
$$
Da $z$ durch $x$ und $y$ bestimmt wird, wird es oft weggelassen. Das **xy-Diagramm** enthält dann alle Farbtöne und Sättigungen, die im XYZ-Farbraum dargestellt werden können.
#### Chromatizitätsdiagramm
Das resultierende **xy-Diagramm** oder <mark style="background: #FFB86CA6;">Chromatizitätsdiagramm zeigt alle wahrnehmbaren Farben als eine zweidimensionale Fläche</mark>:
- Die <mark style="background: #FFB86CA6;">äußere Linie des Diagramms zeigt die Spektralfarben, also die Farben mit maximaler Sättigung</mark> bei verschiedenen Wellenlängen.
- **Innenliegende Punkte** repräsentieren weniger gesättigte Farben, die durch Mischungen verschiedener Wellenlängen entstehen.
![[Pasted image 20241104083806.png|500]]
- Der <mark style="background: #FFB86CA6;">Weißpunkt befindet sich etwa in der Mitte des Diagramms und stellt eine Mischung aller Wellenlängen in ausgewogenen</mark> Anteilen dar. Er entspricht einem neutralen Weißlicht (ähnlich dem Sonnenlicht).
- Die Koordinaten des Weißpunkts sind ungefähr $x = y = z = \frac{1}{3}$​, was auf eine gleiche Gewichtung der drei Grundfarben hinweist.
##### Additive Farbmischung
- <mark style="background: #FFB86CA6;">Alle Farben innerhalb einer Linie oder eines Dreiecks können durch additive Farbmischung der Endpunkte dieser Linie oder Dreiecksfläche erzeugt werden</mark>.
![[Pasted image 20241104084526.png|200]]
- <mark style="background: #FFB86CA6;">Durch eine Linie, die vom Weißpunkt durch einen Punkt verläuft, erhält man die reine Farbe dieses Punktes</mark>.
- Eine Farbe und ihre Komplementärfarbe liegen entlang einer Linie, die durch den Weißpunkt verläuft. Diese Farben können sich bei Mischung zu Weiß ergänzen.
![[Pasted image 20241104084536.png|200]]
##### Spektralfarben im Chromatizitätsdiagramm
- Die **Spektralfarben** befinden sich entlang der Randkurve des Chromatizitätsdiagramms und repräsentieren Farben mit maximaler Sättigung für jede Wellenlänge des sichtbaren Spektrums.
    - <mark style="background: #FFB86CA6;">Diese Farben sind „rein“ und bestehen nur aus einer einzigen Wellenlänge</mark>. Sie werden auch als <mark style="background: #FFB86CA6;">monochromatische Farben</mark> bezeichnet.
1. **Kurze Wellenlängen (ca. 380–480 nm, Blau-Violett)**:
   - Bei kurzen Wellenlängen ist der **Z-Wert** sehr groß, da der blaue Rezeptor sehr empfindlich ist.
   - Da $Z$ dominiert, sind die **x- und y-Werte klein**.
   - Im Chromatizitätsdiagramm liegt der Punkt für diese bläulichen Farben nahe dem unteren Bereich, wo sowohl $x$ als auch $y$ geringe Werte haben.
![[Pasted image 20241104084019.png|300]]
2. **Mittelkurze Wellenlängen (ca. 480 nm, Cyan)**:
   - Bei Wellenlängen um 480 nm wird der **Y-Wert** größer, da der grüne Rezeptor mehr anspricht.
   - Dies führt zu einer Erhöhung des $y$-Wertes, während $x$ relativ klein bleibt.
   - Im Chromatizitätsdiagramm wandert der Punkt für diese Farbe in Richtung der Cyan-Töne.
![[Pasted image 20241104084028.png|300]]
3. **Mittlere Wellenlängen (ca. 500–540 nm, Grün)**:
   - In diesem Bereich ist der **Y-Wert am größten** und dominiert die Wahrnehmung.
   - Da $Y$ zunimmt, nimmt auch der $y$-Wert zu, und der Punkt bewegt sich im Diagramm weiter in Richtung Grün.
   - Diese Farben liegen im Chromatizitätsdiagramm in der Nähe des oberen Bereichs, da der $y$-Wert maximal ist.
![[Pasted image 20241104084035.png|300]]
4. **Längere Wellenlängen (ca. 580 nm, Gelb)**:
   - Mit zunehmender Wellenlänge wird der **Y-Wert kleiner**, und der **X-Wert** beginnt zu dominieren.
   - Der $x$-Wert steigt, während $y$ allmählich abnimmt.
   - Der Punkt bewegt sich im Chromatizitätsdiagramm in Richtung der gelben und orangefarbenen Bereiche.
![[Pasted image 20241104084042.png|300]]
5. **Sehr lange Wellenlängen (ca. 700 nm, Rot)**:
   - Bei diesen Wellenlängen ist der **X-Wert** am größten, und der **Y-Wert** wird weiter kleiner.
   - Der $x$-Wert ist hoch, während $y$ klein bleibt, was die rote Position im Diagramm erklärt.
   - Im Chromatizitätsdiagramm findet sich der Punkt für diese roten Wellenlängen ganz rechts.
![[Pasted image 20241104084052.png|300]]
#### Rückgewinnung der XYZ-Werte
<mark style="background: #FFB86CA6;">Falls die Helligkeit (Y) gegeben ist</mark> und die Farbe durch die Werte $x$ und $y$ spezifiziert ist, <mark style="background: #FFB86CA6;">können die ursprünglichen XYZ-Werte wie folgt berechnet werden</mark>:
$$
X = \frac{Y}{y} \cdot x, \quad Z = \frac{Y}{y} \cdot (1 - x - y)
$$
Diese Formeln erlauben es, eine Farbe im vollständigen XYZ-Raum wiederherzustellen, wenn die Chromatizität (x und y) und die Helligkeit (Y) bekannt sind.
## Gamut-Mapping
- <mark style="background: #FFB86CA6;">Der darstellbare Farbbereich eines Geräts (Gamut</mark>) ist durch ein Dreieck im Diagramm definiert, das <mark style="background: #FFB86CA6;">von den Primärfarben des Geräts aufgespannt</mark> wird.
![[Pasted image 20241104084652.png|300]]
- In der Praxis hat z.B. ein RGB-Monitor Primärfarben, die ein bestimmtes Dreieck (Gamut) aufspannen. Dieses Gamut ist oft kleiner als das gesamte sichtbare Spektrum.
- <mark style="background: #FFB86CA6;">Da verschiedene Geräte unterschiedliche Gamuts haben, wird das Gamut-Mapping verwendet, um Farben konsistent zwischen verschiedenen Geräten darzustellen</mark>. Dadurch wird vermieden, dass Farben außerhalb des Gamuts auf einem Gerät anders dargestellt werden.
## L\*a\*b\* Farbraum
Das **CIE L\*a\*b\* Modell** ist ein <mark style="background: #FFB86CA6;">psychophysisches Farbmodell</mark>, das entwickelt wurde, um Farben in einer Weise darzustellen, die der <mark style="background: #FFB86CA6;">menschlichen Farbwahrnehmung näher kommt</mark>. Es wurde speziell entwickelt, um Farbunterschiede wahrnehmungsgerecht abzubilden. Hier sind die Hauptmerkmale:
1. **Farbraum-Komponenten**:
   - **L\*** steht für die **Luminanz** oder Helligkeit der Farbe (0 = Schwarz, 100 = Weiß).
   - **a\*** repräsentiert den Farbaspekt auf der Achse von Grün bis Rot. Negative Werte bedeuten eher grünliche Töne, positive eher rötliche.
   - **b\*** repräsentiert den Farbaspekt auf der Achse von Blau bis Gelb. Negative Werte zeigen bläuliche Töne, positive eher gelbliche.
2. **Wahrnehmungsorientierte Struktur**:
   - Das Modell orientiert sich an der menschlichen Wahrnehmung, sodass euklidische Abstände im L\*a\*b\* Farbraum den wahrgenommenen Farbunterschieden entsprechen. Das bedeutet, dass Farben, die im L\*a\*b\*-Raum nahe beieinander liegen, für das menschliche Auge ähnlich aussehen.
3. **Anwendungen**:
   - L\*a\*b\* wird oft verwendet, um Farbunterschiede quantitativ zu messen, z.B. in der Qualitätskontrolle von Druckerzeugnissen oder bei der Farbabstimmung in verschiedenen Industrien.
4. **Vorteile gegenüber anderen Modellen**:
   - Im Gegensatz zu RGB oder HSV, die oft zur Farbdarstellung in Bildschirmen oder zur Farbauswahl genutzt werden, ist L\*a\*b\* darauf ausgelegt, die Wahrnehmung von Farben so zu modellieren, wie der Mensch sie tatsächlich sieht. 
