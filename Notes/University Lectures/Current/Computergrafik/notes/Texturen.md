![[Pasted image 20241127141611.png|300]]
- **Grundidee**: 
  - Eine <mark style="background: #FFB86CA6;">realistische Darstellung von Oberflächen wird durch Texturierung</mark> erreicht, indem <mark style="background: #FFB86CA6;">pro Pixel Feinstrukturen</mark> hinzugefügt werden.
  - <mark style="background: #FFB86CA6;">Kombination von Geometrie (3D-Modell) mit Bildern (Texturen).</mark>
  - Vorteil: <mark style="background: #FFB86CA6;">Hohe Detailgenauigkeit ohne erhöhten geometrischen Aufwand</mark> (nützlich für große Netze).
### **Texturierungstechniken**
1. **Klassisches Texture Mapping**:
   - <mark style="background: #FFB86CA6;">Verändert Reflexionseigenschaften</mark> wie <mark style="background: #FFB86CA6;">Farbe, Transparenz oder Reflexionskoeffizienten</mark> einer Oberfläche.
   - Beispiel: Holzmaserung oder Metallglanz.

2. **Bump Mapping / Normal Mapping**:
   - <mark style="background: #FFB86CA6;">Ändert die Oberflächennormalen, ohne die eigentliche Geometrie zu beeinflussen</mark>.
   - Simulation von Unebenheiten (z. B. für Ziegel oder Haut).

3. **Environment Mapping**:
   - <mark style="background: #FFB86CA6;">Darstellung von Spiegelungen basierend auf der Umgebung</mark>.
   - Beispiele: Reflexionen auf Wasser oder einer Metallkugel.

4. **Displacement Mapping**:
   - <mark style="background: #FFB86CA6;">Modifiziert die Geometrie einer Oberfläche physisch</mark> (z. B. für Wellen auf Wasser oder strukturierte Oberflächen wie eine Orangenhaut).

### **2D Texture Mapping**
![[Pasted image 20241127141658.png|300]]
- **Wie wird ein Punkt einer Textur zugeordnet?**:
  - Die Position eines Punktes auf der Oberfläche wird durch Texturkoordinaten ($s, t$) definiert.
  - Diese Koordinaten bestimmen, welches <mark style="background: #FFB86CA6;">Pixel der Textur (Texel)</mark> auf den Punkt der Oberfläche projiziert wird.

- **Beispielanwendung**:
  - Beleuchtungsmodelle, die Lichtstreuung simulieren, z. B. auf Haut (Subsurface Scattering).
### **Textur-Quellen**
- Texturen stammen aus:
  - Fotos, Simulationen, Videos.
  - Sie bestehen aus **Texeln** (Textur-Elementen, ähnlich wie Pixel).
- Arten:
  - 1D-Texturen: Farbtabellen.
  - 2D-Texturen: Fotografien oder synthetisch erzeugte Bilder.
  - 3D-Texturen: Erzeugt durch Simulationen, z. B. für Flüssigkeiten.
- **Texturkompression**: 
  - Reduziert Speicherbedarf durch Block-Kompression (z. B. 4:1 oder 8:1).
### **Mapping von 2D-Texturen**
- **Planare Projektion**:
  - Eine <mark style="background: #FFB86CA6;">Ebene</mark> wird durch einen <mark style="background: #FFB86CA6;">Punkt $p$ und zwei Richtungsvektoren $s, t$ definiert</mark>.
  - Texturkoordinaten $s, t$ werden berechnet, <mark style="background: #FFB86CA6;">indem der Oberflächenpunkt x auf diese Ebene projiziert</mark> wird:
    $$
    s = (x - p) \cdot s,\quad t = (x - p) \cdot t
    $$
  - Bereich [0, 1] repräsentiert die ganze Textur (unabhängig von der tatsächlichen Auflösung).
### **Mapping von 1D-Texturen**
- **Definition**:
  - Parameter (z. B. Höhe, Temperatur) entlang einer Linie werden durch 1D-Texturen dargestellt.
  - Texturkoordinaten werden durch den Skalar $s = (x - p) \cdot s$ bestimmt.
- **Beispiele**:
  - Links: Texturen entlang der Achsen ($s = x, s = y, s = z$).
  - Rechts: 1D-Textur entlang der vertikalen Achse (z. B. Farbverlauf).
## Parametrisierung
### Kugel-Parametrisierung
- **Konzept:** <mark style="background: #FFB86CA6;">Punkte auf der Oberfläche einer Kugel im Ursprung werden in Polarkoordinaten dargestellt</mark>. Dabei sind $r, \phi$ und $\theta$ die Parameter.
  - $r$: <mark style="background: #FFB86CA6;">Radius</mark>
  - $\phi$: <mark style="background: #FFB86CA6;">Azimutwinkel</mark> (horizontal)
  - $\theta$: <mark style="background: #FFB86CA6;">Polarwinkel</mark> (vertikal)
- **Texturkoordinaten ($s, t$):** 
  - $s = \phi / 2\pi$: Normierter Azimutwinkel.
  - $t = \theta / \pi$: Normierter Polarwinkel.
- **Visualisierung:** <mark style="background: #FFB86CA6;">Die Textur wird wie eine Karte über die Kugel gelegt</mark>. Probleme wie <mark style="background: #FF5582A6;">Verzerrungen</mark> können auftreten, insbesondere an den Polen der Kugel.
### Zylindrische Parametrisierung
- **Konzept:** <mark style="background: #FFB86CA6;">Punkte auf der Oberfläche eines Zylinders werden in Zylinderkoordinaten</mark> ($r, \phi, y$) dargestellt.
  - $r$: <mark style="background: #FFB86CA6;">Radius</mark> (oft konstant).
  - $\phi$: <mark style="background: #FFB86CA6;">Winkel entlang der Zylinderachse</mark>.
  - $y$: <mark style="background: #FFB86CA6;">Position entlang der Höhe</mark>.
- **Texturkoordinaten ($s, t$):**
  - $s = \phi / 2\pi$: Normierter Winkel.
  - $t = y / h$: Normierte Höhe ($h$ ist die Zylinderhöhe).
- **Beachte:** Ähnlich wie bei der Kugel treten auch hier Verzerrungen auf, insbesondere an den Übergängen der Textur.
### Würfel-Parametrisierung
- **Ansätze:**
  - **Projektion auf die sechs Würfelseiten:** <mark style="background: #FFB86CA6;">Jeder Punkt wird auf eine der sechs Flächen des Würfels projiziert, basierend auf der Oberflächennormalen</mark>.
  - **Strahl-basierte Methode:** <mark style="background: #FFB86CA6;">Ein Strahl vom Würfelmittelpunkt wird durch den Punkt auf der Würfeloberfläche verlängert und trifft eine der Flächen.</mark>
  - Diese Methoden führen zu einer direkten Zuordnung der Textur auf die Flächen des Würfels.
### Verwendung von Standardkörpern
- **Idee:** Zur Vereinfachung von Texturierungskomplexität wird die Oberfläche eines Objekts zuerst auf einen Standardkörper (z. B. Ebene, Würfel, Kugel, Zylinder) gemappt.
  - **Mapping-Schritte:**
    - **Shape Mapping:** Textur ($s, t$) wird auf die Oberfläche des Standardkörpers gemappt.
    - **Object Mapping:** Standardkörper wird dann auf die Oberfläche des eigentlichen Objekts übertragen.
- **Methoden:**
  - Basierend auf Normalenvektoren der Objektoberfläche.
  - Linien, die vom Mittelpunkt des Objekts durch die Oberfläche gehen.
### Parametrische Flächen als Texturkoordinaten
- **Konzept:** Parametrische Flächen wie Bézier- oder Spline-Patches werden durch $(u, v)$-Parameter dargestellt.
  - $u$: Parameter entlang einer Richtung.
  - $v$: Parameter entlang der anderen Richtung.
- **Texturzugriff:** Die Parameter $(u, v)$ werden direkt als Texturkoordinaten $(s, t)$ interpretiert.
- **Anwendung:** Besonders bei komplexen gekrümmten Flächen wie Bézier-Patches, bei denen der gesamte Flächenbereich durch Textur abgedeckt wird.
## Textur-Mapping
![[Pasted image 20241127142052.png|300]]
### **Texturkoordinaten für Dreiecksnetze**
- **Was sind Texturkoordinaten?**
  - <mark style="background: #FFB86CA6;">Texturkoordinaten sind spezielle Werte (s, t), die jeden Punkt auf der Oberfläche eines 3D-Objekts mit einem Punkt in der Textur verbinden</mark> (normalerweise auf einer 2D-Ebene).
- **Speichern von Texturkoordinaten:**
  - <mark style="background: #FFB86CA6;">In Dreiecksnetzen wird die Parametrisierung explizit gespeichert</mark>.
  - <mark style="background: #FFB86CA6;">Jedem Eckpunkt $\mathbf{v_i} = (x_i, y_i, z_i)$ eines Dreiecks wird eine Texturkoordinate $(s_i, t_i)$ zugeordnet</mark>.
  - Damit wird die Textur flexibel an die 3D-Form angepasst.

- <mark style="background: #FFB86CA6;">Interpolation innerhalb eines Dreiecks</mark>:
  - Innerhalb eines Dreiecks wird die Textur interpoliert. Dies geschieht mit Hilfe von baryzentrischen Koordinaten:
    $$
    v = v_0 + \lambda_1 (v_1 - v_0) + \lambda_2 (v_2 - v_0)
    $$
    Analog gilt dies für die Texturkoordinaten:
    $$
    t = t_0 + \lambda_1 (t_1 - t_0) + \lambda_2 (t_2 - t_0)
    $$
### **Was passiert bei Texturkoordinaten $< 0.0$ oder $> 1.0$?**
- **Problem:**
  - Texturkoordinaten $< 0.0$ oder $> 1.0$ liegen außerhalb der definierten Texturgrenzen.
  - Lösung: **Adressierungsmodi** werden verwendet, um zu definieren, wie diese Werte interpretiert werden.
- **Adressierungsmodi:**
  - <mark style="background: #FFB86CA6;">Clamp: Die Koordinaten werden auf den Bereich $[0, 1]$ begrenzt</mark>.
  - <mark style="background: #FFB86CA6;">Repeat: Die Textur wird wiederholt ("gekachelt").</mark>
  - <mark style="background: #FFB86CA6;">Mirror: Die Textur wird gespiegelt wiederholt</mark>.
### **Texture Wrapping**
- Hier wird gezeigt, wie die Textur sich außerhalb der Koordinatengrenzen verhält:
  - **Clamp/Clamp:** Die Textur ist auf die Koordinaten $[0, 1]$ beschränkt.
  - **Repeat/Clamp:** Die Textur wiederholt sich nur in einer Richtung.
  - **Repeat/Repeat:** Die Textur wiederholt sich in beiden Richtungen.
  - Zusätzliche Modi: z. B. **Mirror**, um die Kantenübergänge natürlicher wirken zu lassen.
### **Texture Mapping und Aliasing**
![[Pasted image 20241127142126.png#invert|300]]
- **Was ist Aliasing?**
  - Aliasing tritt auf, wenn eine kontinuierliche Funktion (z. B. Textur) diskret abgetastet wird.
  - In der Grafik entspricht dies der Abbildung einer hochaufgelösten Textur auf eine niedrigaufgelöste Oberfläche, was zu visuellen Artefakten führt.
- **Abtastprobleme:**
  - Das Abtastproblem betrifft die Zuordnung zwischen dem 2D-Texturraum und dem 3D-Objektraum (Parametrisierung).
  - Auch die Projektion vom 3D-Objektraum in den 2D-Bildraum kann Aliasing verursachen.
- **Lösung:**
  - Verwendung von **Anti-Aliasing-Techniken** und <mark style="background: #FFB86CA6;">Mipmapping</mark>, um die Diskretisierungseffekte zu reduzieren.

## Textur-Filterung
### Vergrößerung/Magnification
#### Nearest Neighbor
- **Methode:** <mark style="background: #FFB86CA6;">Wählt den Texel (Texture Pixel), der am nächsten am Pixelzentrum liegt</mark>.
- **Ergebnis:** <mark style="background: #FFB86CA6;">Es wird exakt die Farbe dieses Texels für den Pixel übernommen</mark>.
- **Vorteil:** <mark style="background: #FFB86CA6;">Einfach und schnell</mark>.
- **Nachteil:** <mark style="background: #FFB86CA6;">Kann zu "harten" Kanten</mark> oder grob wirkenden Texturen führen, da keine Glättung erfolgt.
#### Bilineare Interpolation
- **Methode:** Nutzt die <mark style="background: #FFB86CA6;">vier nächstgelegenen Texel und interpoliert ihre Farben basierend auf den relativen Abständen</mark>.
  - Die Farbe eines Pixels wird durch die gewichtete Summe der Texelfarben berechnet.
  - Dies erfolgt in <mark style="background: #FFB86CA6;">zwei Schritten</mark>:
    1. **Horizontale Interpolation:** <mark style="background: #FFB86CA6;">Zwischen zwei Texeln entlang einer Achse</mark>.
    2. **Vertikale Interpolation:** <mark style="background: #FFB86CA6;">Aus den Ergebnissen der horizontalen Interpolation</mark>.
- **Formel:**
  $$
  t = (1-b)t_{12} + b \cdot t_{34}
  $$
  - $t_{12} = t_1(1-a) + t_2a$ (interpoliert horizontal oben)
  - $t_{34} = t_3(1-a) + t_4a$ (interpoliert horizontal unten)
  - Endresultat kombiniert vertikal.
- **Vorteil:** <mark style="background: #FFB86CA6;">Glättet die Textur sichtbar</mark>, besonders bei Vergrößerungen.
- **Nachteil:** <mark style="background: #FFB86CA6;">Aufwändiger als Nearest Neighbor</mark>, da mehrere Texel und Berechnungen benötigt werden.
#### Alternative Interpretation
- Die Gewichte der Texel können als Flächenverhältnisse interpretiert werden (baryzentrisch):
  - Der Flächeninhalt eines Quadrats oder Dreiecks bestimmt den relativen Einfluss eines Texels auf den Zielpixel.

#### **Textur-Filtrierung höherer Ordnung**
- **Bikubische Interpolation**:
    - <mark style="background: #FFB86CA6;">Interpoliert Farben basierend auf 4 × 4 Texel</mark> (16 Texel für 2D-Texturen).
    - Nutzt komplexere mathematische Funktionen für glattere Ergebnisse (z. B. Splines).
    - **Nachteil**: <mark style="background: #FFB86CA6;">Rechenaufwändig</mark> und wird nicht direkt von GPU-Hardware unterstützt.
#### **Textur-Filtrierung (Verkleinerung/Minification)**
- **Problem**: <mark style="background: #FFB86CA6;">Mehrere Texel werden auf einen einzigen Pixel projiziert</mark>.
    - Bei nur einem Texel können **Aliasing-Artefakte** auftreten, da hochfrequente Informationen verloren gehen.
    - Beispiel: In der Abbildung "Aliasing" sind störende Muster sichtbar.
- **Lösungen**:
    - <mark style="background: #FFB86CA6;">Vorfilterung</mark>: Entfernt hochfrequente Inhalte vor der Abtastung.
    - <mark style="background: #FFB86CA6;">Supersampling</mark>: Nutzt eine höhere Abtastauflösung als nötig, um Aliasing zu reduzieren (teuer).
#### **Aliasing und Sampling**

- **Abtasttheorie (Nyquist-Frequenz)**:
    - Signal muss mindestens mit der doppelten Maximalfrequenz abgetastet werden.
    - **Aliasbildung** entsteht, wenn Frequenzanteile überlappen.
    - **Lösung**:
        1. **Vorfilterung** reduziert Frequenzen vor dem Sampling.
        2. **Sinc-Filter** rekonstruiert das Signal ideal (praktisch aber schwer realisierbar).
### Mip-Mapping
![[Pasted image 20241127152548.png|300]]
Mip-Mapping (lateinisch „multum in parvo“ = „viel in wenig“) ist eine <mark style="background: #FFB86CA6;">Technik der Texturvorfilterung</mark>, die in der Computergrafik verwendet wird, <mark style="background: #FFB86CA6;">um Speicher und Rendering-Zeit zu sparen sowie Aliasing-Effekte zu reduzieren</mark>. 
- **Funktionsweise:**
  - **Vorfilterung:** <mark style="background: #FFB86CA6;">Es werden kleinere Kopien (Mipmap-Stufen) einer Originaltextur erstellt</mark>. Jede Stufe hat <mark style="background: #FFB86CA6;">1/4 der Fläche der vorherigen</mark>, indem beide Dimensionen (Breite und Höhe) halbiert werden.
  - Das führt zu einer Pyramidenstruktur der Texturen (z. B. 64², 32², 16², ...).
  - **Speicherbedarf:** Diese <mark style="background: #FFB86CA6;">zusätzlichen Texturen benötigen nur etwa 33 % mehr Speicherplatz</mark>.
- **Hauptvorteile:**
  - <mark style="background: #FFB86CA6;">Reduktion von Aliasing: Verhindert „flimmernde“ Texturen bei weit entfernten Objekten</mark>.
  - <mark style="background: #FFB86CA6;">Schnellere Berechnungen: Die GPU entscheidet basierend auf der Distanz, welche Mipmap-Stufe verwendet wird</mark>.
- **Nachteil:** <mark style="background: #FFB86CA6;">Leichte Unschärfe, da Details in niedrigeren Stufen verloren gehen</mark>.
### **Tiefpass und Hochpass:**
- **Tiefpass:** Diese Filtertechnik bewirkt eine <mark style="background: #FFB86CA6;">Glättung der Textur, indem hochfrequente Details reduziert werden</mark>. 
  - Beispiel: Die linke Darstellung zeigt eine geglättete Version, bei der Details verschwimmen.
- **Hochpass:** Dieser Filter <mark style="background: #FFB86CA6;">betont die Kanten und Details in einem Bild, indem die Tiefpass-Version von der Originalversion subtrahiert</mark> wird.
  - Beispiel: Die rechte Darstellung hebt die Struktur der Türrahmen und der Wände hervor.
### **Anisotrope Texturfilterung:**
Anisotrope Filterung ist <mark style="background: #FFB86CA6;">eine Weiterentwicklung des Mip-Mappings</mark>, die <mark style="background: #FFB86CA6;">Verzerrungen reduziert, wenn Texturen unter einem schrägen Winkel betrachtet werden.</mark>
- **Problem:** 
  - <mark style="background: #FFB86CA6;">Mip-Mapping ist isotrop (gleichmäßig in allen Richtungen), während der "Footprint" eines Pixels oft länglich ist</mark>.
  - Resultat: <mark style="background: #FFB86CA6;">Verschwommene Details in schrägen Ansichten</mark>.
- **Lösung:** 
  - <mark style="background: #FFB86CA6;">Anpassung der Vorfilterung an die tatsächliche Form des Pixel-Footprints</mark>.
  - <mark style="background: #FFB86CA6;">RIP-Maps (Rectangular Mip-Maps)</mark> werden verwendet, um längliche Formen besser zu behandeln, sind aber <mark style="background: #FFB86CA6;">speicherintensiv</mark>.
### **Summed Area Tables (SAT):**
![[Pasted image 20241127152634.png#invert|400]]
![[Pasted image 20241127152647.png#invert|200]]
SATs sind eine Technik zur effizienten Berechnung von Summen über rechteckige Bereiche in einer Textur.
- **Anwendung:** 
  - <mark style="background: #FFB86CA6;">Berechnung von Box-Filtern oder weichen Schattierungen in Echtzeit</mark>.
  - Beispiel: Die Summe der markierten Bereiche wird mit der Formel berechnet: $D - B - C + A$.

## Bestimmen des Footprints
#### 1. Einfache Möglichkeit, den Footprint zu bestimmen
<mark style="background: #FFB86CA6;">Ein Footprint beschreibt den Bereich im Texturraum, der einem Pixel auf dem Bildschirm entspricht</mark>. Das Ziel ist, Texturen möglichst genau abzubilden, ohne Verzerrungen oder Alias-Effekte.
- **Grundidee:**
  - <mark style="background: #FFB86CA6;">Jeder Pixel auf dem Bildschirm projiziert sich in den Texturraum</mark>.
  - <mark style="background: #FFB86CA6;">Die Ecken des Pixels werden verfolgt, indem ein Primärstrahl (Ray) durch sie geschickt wird</mark>.
  - Diese Ecken definieren <mark style="background: #FFB86CA6;">einen 2×2-Pixelblock im Texturraum</mark>.
- **Bestimmung der Größe und Form:**
  - <mark style="background: #FFB86CA6;">Die Differenzen der Texturkoordinaten zwischen den Pixeln liefern die Größe und Form des Footprints</mark>.
  - Für verschiedene Verfahren:
    - **Mip-Mapping:** Es <mark style="background: #FFB86CA6;">wird der größte Wert zwischen Breite und Höhe bestimmt, und daraus wird die Mip-Map-Stufe $n$ gewählt</mark>.
    - **RIP-Mapping oder SAT (Summed Area Tables):** <mark style="background: #FFB86CA6;">Breite und Höhe werden separat behandelt</mark>.
- **Filterung:**
  - Nach Berechnung der Stelle im Texturraum wird an dieser Stelle mit der geeigneten Methode gefiltert (z. B. bilineare Interpolation).
#### 2. Ray Differentials zur Bestimmung der Footprints
Dieses Verfahren <mark style="background: #FFB86CA6;">nutzt die Ableitung der Texturkoordinaten im Bildschirmraum, um den Footprint genauer zu bestimmen.</mark>
- **Prinzip:****
  - Der <mark style="background: #FFB86CA6;">Primärstrahl trifft auf ein Objekt</mark>. Daraus ergeben sich:
    - Die <mark style="background: #FFB86CA6;">Schnittposition</mark> $x$,
    - Die <mark style="background: #FFB86CA6;">Normale</mark> $n$, die die <mark style="background: #FFB86CA6;">Tangentialebene definiert</mark>.
  - Die Tangentialebene <mark style="background: #FFB86CA6;">bildet den Übergang zwischen</mark>:
    - <mark style="background: #FFB86CA6;">Texturkoordinaten</mark> (z. B. $(s, t)$) und
    - der <mark style="background: #FFB86CA6;">Position im Raum</mark> (z. B. $(x, y, z)$).
- **Differenzen in den Texturkoordinaten:**
  - Die <mark style="background: #FFB86CA6;">Ableitung der Texturkoordinaten gibt an, wie sich diese Koordinaten verändern</mark>. Dies <mark style="background: #FFB86CA6;">erlaubt Rückschlüsse auf die Größe und Form des Footprints</mark>.
- **Allgemeiner Nutzen:**
  - Diese Methode lässt sich verallgemeinern, z. B. für Effekte wie Spiegelungen oder Lichttransmission.
#### Anwendung:
1. In Raytracing-Szenen ermöglicht das Verfahren, <mark style="background: #FFB86CA6;">schärfere und genauere Texturdetails</mark> darzustellen, indem es die Texturdetails besser an die Pixelauflösung anpasst.
2. Für dynamische Szenen wird häufig die zweite Methode verwendet, da sie präzisere Ergebnisse bei unterschiedlichen Blickwinkeln liefert.
## Texturierungstechniken
### **Diffuse Textur**
![[Pasted image 20241206114657.png|400]]
- **Definition**: Bezieht sich auf die Eigenfarbe eines Materials, also die Farbe, die unabhängig von Spiegelungseffekten bleibt.
- **Beispiel**: Im Phong-Beleuchtungsmodell wird die diffuse Reflexion durch den Koeffizienten $k_d$ bestimmt, welcher hier aus einer Textur gelesen wird.
- **Formel**:
  $$
  I = k_a \cdot I_L + k_d \cdot I_L \cdot (N \cdot L) + k_s \cdot I_L \cdot (R \cdot V)^n
  $$
  - $k_a$: Ambient-Beleuchtung (Umgebungslicht).
  - $k_d$: Diffuse Reflexion.
  - $k_s$: Spekularer Reflexionsanteil.
  - $I_L$: Intensität der Lichtquelle.
  - $N$: Normale.
  - $L$: Richtung zur Lichtquelle.
  - $R$: Reflektionsvektor.
  - $V$: Richtung zum Betrachter.
  - $n$: Glanzexponent (Kontrolle der Streuung der Reflexion).
### **Bump Mapping / Normal Mapping**
- **Ziel**: <mark style="background: #FFB86CA6;">Erzeugung von Oberflächendetails durch Manipulation der Normalen ohne geometrische Änderung der Oberfläche</mark>.
![[Pasted image 20241206114727.png#invert|300]]
- **Unterschiede**:
  - **Bump Mapping**: <mark style="background: #FFB86CA6;">Verändert die Helligkeitswerte basierend auf einer Höhe</mark> (Graustufen-Textur).
  - **Normal Mapping**: <mark style="background: #FFB86CA6;">Verwendet eine Farbkodierung (RGB) zur Definition der Normalen</mark>.
- **Vorgehen**:
  - Die Normale $N$ wird durch Texturen modifiziert, was <mark style="background: #FFB86CA6;">zu Änderungen im Beleuchtungsmodell führt</mark>.
  - Die berechneten Werte für $(N \cdot L)$ und $(R \cdot V)^n$ ändern sich, was die Oberflächendetails beeinflusst.
### **Displacement Mapping**
![[Pasted image 20241206114809.png|300]]
- **Unterschied zu Bump/Normal Mapping**:
  - **Displacement Mapping** <mark style="background: #FFB86CA6;">verändert die tatsächliche Geometrie der Oberfläche</mark> (z. B. durch GPU-Tessellation).
  - Im Gegensatz zu Bump Mapping wird die Oberfläche physisch verschoben.
- **Beispiel**:
  - Eine flache Oberfläche wird durch eine Höhenkarte (Heightmap) zu einer welligen oder strukturierten Oberfläche transformiert.
- **Vorteil**:
  - Realistische Silhouetten, da die Geometrie tatsächlich angepasst wird.
#### **Inverse Displacement Mapping**
- **Prinzip**: Eine Approximation von Displacement Mapping</mark>.
- **Merkmale**:
  - Verwendet Parallax Occlusion Mapping, um Details in einer Textur zu simulieren, ohne die Geometrie zu ändern.
  - Schnelle Berechnung, aber problematisch bei Silhouetten.
### **Gloss-Map / Gloss-Textur**
- **Definition**: Steuert die Stärke und Streuung der spekularen Reflexion.
- **Beispiel**: Im Phong-Modell wird $k_s$ aus der Textur gelesen.
- **Effekt**:
  - Oberflächen erscheinen glänzender oder matter abhängig vom Texturwert.
  - Der Glanzexponent $n$ wird ebenfalls aus einer Textur ausgelesen.
### **Ambient Occlusion**
- **Definition**: Simuliert den Effekt, wie viel Umgebungslicht (diffuses Licht) einen Punkt auf einer Oberfläche erreicht.
- **Eigenschaften**:
  - Bestimmt durch Raycasting, wobei geprüft wird, welche Strahlen keine Geometrie in der Umgebung treffen.
  - Vorberechnung ist oft notwendig und wird pro Vertex oder pro Pixel gespeichert.
- **Vorteil**:
  - Fügt Tiefe und Schatten hinzu, ohne teure Echtzeitberechnungen durchzuführen.
## Textur-Atlas
![[Pasted image 20241206115340.png#invert|400]]
- **Definition**: Ein <mark style="background: #FFB86CA6;">Textur-Atlas ist eine spezielle (bijektive) Parametrisierung, bei der jeder Oberflächenpunkt eines 3D-Modells genau einer Stelle in der Textur zugeordnet wird</mark>.
- **Besonderheiten**:
  - Jeder Punkt in der Textur wird nur einmal genutzt, außer bei symmetrischen Objekten.
  - Die Erstellung erfordert das Zerschneiden und „Ausrollen“ des 3D-Modells (Netz), was zu Verzerrungen führen kann, abhängig von der Anzahl und Lage der Schnitte.
- **Anwendungen**:
  - Effizientes Texturieren von komplexen Modellen.
  - Speichern von Oberflächendaten (z. B. Ambient Occlusion, Normal Maps).
  - "Painting": Direktes Erstellen einer Textur auf dem 3D-Modell.
## Transparenz und Alpha-Test
- **Transparenz**:
  - Rasterbilder und Texturen speichern häufig Transparenz in einem **Alpha-Kanal** (RGBA-Format).
  - Der Alpha-Wert ($\alpha$) bestimmt die Opazität (0 = transparent, 1 = undurchsichtig).
- **Alpha-Test**:
  - <mark style="background: #FFB86CA6;">Verwirft Fragmente basierend auf einem Schwellenwert</mark> ($\alpha < \text{threshold}$).
  - Nützlich für semitransparente Objekte oder Maskierungen.
- **Verwendung**:
  - Raytracing: Alpha-Kanal (z. B. $k_t$) zur Bestimmung der Transparenz eines Oberflächenpunktes.
  - Rasterisierung: Erzeugung von Aussparungen oder Maskeneffekten durch Alpha-Tests.
## Impostors
![[Pasted image 20241206115414.png|300]]
- **Definition**: <mark style="background: #FFB86CA6;">Impostors sind texturierte Polygone, die stets senkrecht zur Kamera ausgerichtet sind</mark>.
- **Vorteile**:
  - <mark style="background: #FFB86CA6;">Sehr effizient für weit entfernte Objekte, die keine detaillierte Geometrie benötigen</mark> (z. B. Wolken, Rauch, Funken).
  - Reduziert die Renderkosten erheblich, da nur ein einfaches, flaches Polygon verwendet wird.
- **Beispiele**:
  - Darstellung von Partikelsystemen in Spielen oder Animationen.
  - Wolken, die sich nicht dynamisch verändern, aber realistisch aussehen sollen.
## 3D-Texturen für Oberflächen
- **2D-Texturen**:
  - Probleme wie der „<mark style="background: #FFB86CA6;">Tapeten-Effekt“ entstehen, wenn Texturen wiederholt werden</mark>.
  - <mark style="background: #FFB86CA6;">Schwierige Parametrisierung bei komplexen Objekten</mark>.
- **3D-Texturen**:
  - <mark style="background: #FFB86CA6;">Volumendaten</mark>, bei denen jeder Punkt im Volumen eigene Texturkoordinaten hat.
  - Vorteil: <mark style="background: #FFB86CA6;">Keine Probleme mit Parametrisierung</mark>, da die Textur direkt im Volumen gespeichert wird.
  - Nachteil: <mark style="background: #FFB86CA6;">Hoher Speicherbedarf</mark> (z. B. $512^3$ RGB = 384 MB).
- **Verwendung**:
  - „Herausschneiden“ einer Skulptur durch Texturkoordinaten-Zuweisung.
  - Volumen-Rendering von inneren Strukturen wie Organen oder Materialdichte.
### 3D-Texturen und Volumenvisualisierung
- **Definition**: Darstellung von Simulations- oder Messdaten in Form von 3D-Texturen.
- **Beispiele**:
  - **Medizin**: Röntgenbilder, CT- oder MRT-Scans.
  - **Geowissenschaften**: Dichteverteilung im Erdinneren.
  - **Wettermodelle**: Feuchtigkeitsverteilung in der Atmosphäre.
- **Voxels**:
  - Speicherung in Voxeln (3D-Pixeln), oft in regelmäßigen Gitterstrukturen.
  - Geeignet für zeitabhängige Daten (4D-Daten mit Zeitkomponente).
## Environment Mapping
**Motivation**  
- Environment Mapping ermöglicht die Darstellung reflektierender Objekte mit Spiegelungen einer Umgebung, ohne dass diese Umgebung geometrisch repräsentiert werden muss.  
- Es ist eine schnelle Approximation für Reflexionen und <mark style="background: #FFB86CA6;">ersetzt komplexe Raytracing-Berechnungen, indem das Bild der Umgebung in einer Textur gespeichert wird.</mark>

**Grundprinzip**  
1. **Reflexionsrichtung berechnen**:  
   - Die Richtung $r$ eines Reflexionsstrahls wird basierend auf der Normale $n$ und der Sichtvektorrichtung $v$ am Objekt berechnet.  
   $$
   r = 2 (n \cdot v) n - v
   $$
2. **Zugriff auf Environment Map**:  
   - Die Richtung $r$ wird in Texturkoordinaten umgewandelt, um die passende Farbe aus der Umgebungstextur (Environment Map) zu lesen.
**Annahmen und Einschränkungen**  
- <mark style="background: #FFB86CA6;">Nur die Richtung $r$ wird verwendet, ohne Berücksichtigung der Entfernung</mark>.  
  - Diese Vereinfachung <mark style="background: #FFB86CA6;">ist akzeptabel, wenn die Umgebung weit entfernt</mark> ist (z. B. Himmel oder weite Landschaften).
- Die Umgebung wird auf der inneren Oberfläche einer virtuellen Kugel abgebildet.
### **Parametrisierungen**  
#### **Latitude/Longitude-Maps**:  
  - Die <mark style="background: #FFB86CA6;">Kugel wird durch Polarkoordinaten parametriert</mark>:
    - $\theta$: Polarwinkel (0 bis $\pi$).
    - $\phi$: Azimuthwinkel (0 bis $2\pi$).  
  - Nachteile:
    - <mark style="background: #FFB86CA6;">Ungleichmäßige Verteilung der Texturdaten, besonders an den Polen</mark>.
    - <mark style="background: #FFB86CA6;">Höherer Rechenaufwand</mark> bei der Umrechnung von $r$ in Texturkoordinaten.
#### **Sphere Mapping**:  
  - <mark style="background: #FFB86CA6;">Darstellung der Umgebung auf einer virtuellen Spiegelkugel</mark>.  
  - <mark style="background: #FFB86CA6;">Bild der Kugel wird als Texture verwendet</mark> (z. B. „chrome ball“ Methode).
  - Vorteil: <mark style="background: #FFB86CA6;">Sehr einfach umzusetzen</mark>.
  - Nachteil: <mark style="background: #FFB86CA6;">Verzerrungen bei ungleichmäßigen Oberflächen</mark>.
#### Cube Environment Maps
- die Umgebung auf die sechs Flächen eines Würfels projiziert wird.
### Sphere Mapping
**Definition**  
Sphere Mapping ist eine Technik des Environment Mapping, bei der eine Umgebung auf die Oberfläche einer virtuellen Spiegelkugel projiziert wird, um Reflexionen auf Objekten darzustellen.
**Berechnung der Texturkoordinaten**  
1. **Gegebene Daten**:  
   - **Betrachtungsrichtung** $v$: Richtung zur Kamera.  
   - **Oberflächennormale** $n_x$: Normale des Objekts an einem Punkt.  
   - **Reflexionsrichtung** $r$: Richtung, in die das Licht reflektiert wird.  
   - $v_0$: Richtung, aus der die Umgebung in die Sphere Map aufgenommen wurde.  
2. **Ziel**:  
   - Den Punkt auf der Kugeloberfläche finden, dessen Normale $n$ mittig zwischen $r$ und $v_0$ liegt.  
   - Dieser Punkt gibt die Texturkoordinaten $s, t$ für die Sphere Map.  

3. **Berechnungsdetails**:  
   - Reflexionsrichtung:
     $$
     r = 2 (v \cdot n) n - v
     $$
   - „Half-Way Vector“:
     $$
     h = \frac{r + v_0}{|r + v_0|}
     $$
     $h$ liegt genau zwischen $r$ und $v_0$ und liefert die Normale $n$ auf der Kugeloberfläche.
4. **Texturkoordinaten**:  
   Die Kugeloberfläche wird auf ein Rechteck projiziert:
   $$
   s = \frac{h_x + 1}{2}, \quad t = \frac{h_y + 1}{2}
   $$
   Hierbei werden nur die $x$- und $y$-Komponenten verwendet.
**Probleme und Einschränkungen**  
- **Ungleichmäßige Abtastung**:  
  - Starke Verzerrung am Rand der Kugelprojektion.  
  - Besonders ausgeprägt bei Betrachtungsrichtungen, die stark von $v_0$ abweichen.  
- **Singularität**:  
  - Am Rand treten Singularitäten auf, die zu Artefakten in der Textur führen können.
- **Einschränkungen**:  
  - Sphere Mapping eignet sich nur gut für Betrachtungsrichtungen, die $v_0$ ähneln.
### Cube Environment Maps
**Definition**  
Eine andere Parametrisierung des Environment Mapping, bei der die Umgebung auf die sechs Flächen eines Würfels projiziert wird.
**Berechnung der Texturkoordinaten**  
1. **Gegebene Daten**:  
   - Reflexionsrichtung $r = (r_x, r_y, r_z)$.  
   - Würfelflächen repräsentieren verschiedene Bereiche der Umgebung.  

2. **Zuweisung der Flächen**:  
   - Die betraglich größte Komponente von $r$ bestimmt die Würfelfläche:
     - $|r_x| > |r_y|$ und $|r_x| > |r_z|$: „right“ oder „left“.  
     - $|r_y| > |r_x|$ und $|r_y| > |r_z|$: „top“ oder „bottom“.  
     - $|r_z| > |r_x|$ und $|r_z| > |r_y|$: „front“ oder „back“.  

3. **Texturkoordinaten innerhalb der Fläche**:  
   - Für die Fläche „right“:
     $$
     s = \frac{r_y}{2 r_x} + \frac{1}{2}, \quad t = \frac{r_z}{2 r_x} + \frac{1}{2}
     $$
**Vorteile von Cube Maps**  
- Gleichmäßige Abtastung über die gesamte Umgebung.  
- Bessere Ergebnisse für Betrachtungsrichtungen, die stark von $v_0$ abweichen.
### Vorfilterung von Environment Maps
**Ziel der Vorfilterung**  
Die Vorfilterung von Environment Maps <mark style="background: #FFB86CA6;">dient dazu, die Reflexion von Licht auf Oberflächen realistisch darzustellen</mark>, <mark style="background: #FFB86CA6;">indem der Einfluss von Licht aus verschiedenen Richtungen zusammengefasst wird</mark>. Dies wird für unterschiedliche Zwecke wie spiegelnde Objekte, imperfekte Spiegelungen oder diffuse Reflexionen verwendet.

**Konzepte der Vorfilterung**  
1. **Reflexionsrichtung ($r$)**:  
   - Für <mark style="background: #FFB86CA6;">spiegelnde Objekte wird die genaue Reflexionsrichtung verwendet</mark>, um den Lichtwert aus der Umgebung zu bestimmen.  
   - Beispiel: Ein glänzender Chromball spiegelt exakt das Umgebungsbild wider.
2. **Imperfekte Spiegelung**:  
   - Für <mark style="background: #FFB86CA6;">Oberflächen mit leicht unscharfen Reflexionen wird Licht aus mehreren Richtungen gewichtet</mark>.  
   - Dies <mark style="background: #FFB86CA6;">orientiert sich am Phong-Modell</mark> mit:
     $$
     (r \cdot d)^n
     $$
     wobei $d$ die Richtung des einfallenden Lichts ist und $n$ den Glanzexponenten bestimmt.
3. **Diffuse Reflexion**:  
   - Für matte Oberflächen wird die <mark style="background: #FFB86CA6;">Menge an Licht berechnet, die von allen möglichen Richtungen kommt</mark> und von der Normale $n$ beeinflusst wird.
4. **Kombination von Texturen**:  
   - Oft werden mehrere vorgefilterte Texturen kombiniert, um Reflexionen für unterschiedliche Oberflächenmaterialien darzustellen.
**Techniken der Vorfilterung**  
- <mark style="background: #FFB86CA6;">Summed Area Tables</mark>:  
  - Ermöglichen eine effiziente Approximation der Vorfilterung, indem die Texturwerte über einen Winkelbereich zusammengefasst werden.  
  - Vorteil: Kann on-the-fly berechnet werden.
### (Selbst-)Verschattung
**Problemstellung**  
- Eine korrekte Berechnung von Schatten ist nicht mit vorgefilterten Environment Maps kompatibel, da diese keine geometrischen Informationen enthalten.  
- Effiziente Ansätze:  
  - Schattenberechnung durch das Objekt selbst oder andere Objekte.  
  - Beschränkung auf wenige Schattenstrahlen (z. B. um die Reflexionsrichtung $r$).  
  - Verwendung von **Ambient Occlusion** als Annäherung.

**Mathematische Annäherung**  
- Vereinfachte Formel zur Berechnung der Verschattung:
  $$
  \frac{2\pi}{N} \sum \left( L_i(\omega_i) V_x(\omega_i) \right) \approx \frac{2\pi}{N} \sum L_i(\omega_i) \cdot \frac{2\pi}{N} \sum V_x(\omega_i)
  $$
  - $L_i$: Lichtintensität aus Richtung $\omega_i$.  
  - $V_x(\omega_i)$: Sichtbarkeitsfunktion (1: Lichtstrahl trifft Geometrie, 0: sonst).
### Vorfilterung für diffuse Beleuchtung
**Ziel**  
- Die Funktion $L(r)$, die das einfallende Licht in der Environment Map beschreibt, wird so vorgefiltert, dass sie für Flächen mit bestimmten Normalen $n$ direkt angewendet werden kann.

**Vorgehen**  
1. **Eingabe**:  
   - Eine Environment Map mit Lichtwerten basierend auf der Reflexionsrichtung $r$.
2. **Ausgabe**:  
   - Eine Beleuchtungstextur, die für jede Normale $n$ das diffus reflektierte Licht enthält.
3. **Berechnung**:  
   - Abbildung von Texturkoordinaten $(s, t)$ auf die Richtung $r$.  
   - Reflexionsrichtung $r$ wird gespiegelt an der Normale $n$:
     $$
     r = 2 (v_0 \cdot n) n - v_0
     $$
     wobei $v_0$ die Richtung zur Kamera ist.
### Fazit: Texture Mapping
- **Bedeutung**:  
  - Texture Mapping ist eine grundlegende Technik in der Computergrafik, die es ermöglicht, Details und Oberflächenmerkmale effizient darzustellen.
- **Einsatzgebiete**:  
  - Farb-, Gloss- und Ambient Occlusion-Texturen.  
  - Normal- und Displacement-Mapping.  
  - Environment Mapping und Image-Based Lighting.  
  - Shadow Mapping und Visualisierungen.  
- **Effizienzsteigerung**:  
  - Verwendung von Lookup-Tabellen und vorgefilterten Environment Maps.  


