![[Pasted image 20241127141611.png|300]]
- **Grundidee**: 
  - Eine realistische Darstellung von Oberflächen wird durch Texturierung erreicht, indem pro Pixel Feinstrukturen hinzugefügt werden.
  - Kombination von Geometrie (3D-Modell) mit Bildern (Texturen).
  - Vorteil: Hohe Detailgenauigkeit ohne erhöhten geometrischen Aufwand (nützlich für große Netze).
### **Texturierungstechniken**
1. **Klassisches Texture Mapping**:
   - Verändert Reflexionseigenschaften wie Farbe, Transparenz oder Reflexionskoeffizienten einer Oberfläche.
   - Beispiel: Holzmaserung oder Metallglanz.

2. **Bump Mapping / Normal Mapping**:
   - Ändert die Oberflächennormalen, ohne die eigentliche Geometrie zu beeinflussen.
   - Simulation von Unebenheiten (z. B. für Ziegel oder Haut).

3. **Environment Mapping**:
   - Darstellung von Spiegelungen basierend auf der Umgebung.
   - Beispiele: Reflexionen auf Wasser oder einer Metallkugel.

4. **Displacement Mapping**:
   - Modifiziert die Geometrie einer Oberfläche physisch (z. B. für Wellen auf Wasser oder strukturierte Oberflächen wie eine Orangenhaut).

### **2D Texture Mapping**
![[Pasted image 20241127141658.png|300]]
- **Wie wird ein Punkt einer Textur zugeordnet?**:
  - Die Position eines Punktes auf der Oberfläche wird durch Texturkoordinaten ($s, t$) definiert.
  - Diese Koordinaten bestimmen, welches Pixel der Textur (Texel) auf den Punkt der Oberfläche projiziert wird.

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
  - Eine Ebene wird durch einen Punkt $p$ und zwei Richtungsvektoren $s, t$ definiert.
  - Texturkoordinaten $s, t$ werden berechnet, indem der Oberflächenpunkt $x$ auf diese Ebene projiziert wird:
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
- **Konzept:** Punkte auf der Oberfläche einer Kugel im Ursprung werden in Polarkoordinaten dargestellt. Dabei sind $r, \phi$ und $\theta$ die Parameter.
  - $r$: Radius
  - $\phi$: Azimutwinkel (horizontal)
  - $\theta$: Polarwinkel (vertikal)
- **Texturkoordinaten ($s, t$):** 
  - $s = \phi / 2\pi$: Normierter Azimutwinkel.
  - $t = \theta / \pi$: Normierter Polarwinkel.
- **Visualisierung:** Die Textur wird wie eine Karte über die Kugel gelegt. Probleme wie Verzerrungen können auftreten, insbesondere an den Polen der Kugel.
### Zylindrische Parametrisierung
- **Konzept:** Punkte auf der Oberfläche eines Zylinders werden in Zylinderkoordinaten ($r, \phi, y$) dargestellt.
  - $r$: Radius (oft konstant).
  - $\phi$: Winkel entlang der Zylinderachse.
  - $y$: Position entlang der Höhe.
- **Texturkoordinaten ($s, t$):**
  - $s = \phi / 2\pi$: Normierter Winkel.
  - $t = y / h$: Normierte Höhe ($h$ ist die Zylinderhöhe).
- **Beachte:** Ähnlich wie bei der Kugel treten auch hier Verzerrungen auf, insbesondere an den Übergängen der Textur.
### Würfel-Parametrisierung
- **Ansätze:**
  - **Projektion auf die sechs Würfelseiten:** Jeder Punkt wird auf eine der sechs Flächen des Würfels projiziert, basierend auf der Oberflächennormalen.
  - **Strahl-basierte Methode:** Ein Strahl vom Würfelmittelpunkt wird durch den Punkt auf der Würfeloberfläche verlängert und trifft eine der Flächen.
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
  - Texturkoordinaten sind spezielle Werte $(s, t)$, die jeden Punkt auf der Oberfläche eines 3D-Objekts mit einem Punkt in der Textur verbinden (normalerweise auf einer 2D-Ebene).
- **Speichern von Texturkoordinaten:**
  - In Dreiecksnetzen wird die Parametrisierung explizit gespeichert.
  - Jedem Eckpunkt $\mathbf{v_i} = (x_i, y_i, z_i)$ eines Dreiecks wird eine Texturkoordinate $(s_i, t_i)$ zugeordnet.
  - Damit wird die Textur flexibel an die 3D-Form angepasst.

- **Interpolation innerhalb eines Dreiecks:**
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
  - **Clamp:** Die Koordinaten werden auf den Bereich $[0, 1]$ begrenzt.
  - **Repeat:** Die Textur wird wiederholt ("gekachelt").
  - **Mirror:** Die Textur wird gespiegelt wiederholt.
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
  - Verwendung von **Anti-Aliasing-Techniken** und **Mipmapping**, um die Diskretisierungseffekte zu reduzieren.

## Textur-Filterung
### Vergrößerung/Magnification
#### Nearest Neighbor
- **Methode:** Wählt den Texel (Texture Pixel), der am nächsten am Pixelzentrum liegt.
- **Ergebnis:** Es wird exakt die Farbe dieses Texels für den Pixel übernommen.
- **Vorteil:** Einfach und schnell.
- **Nachteil:** Kann zu "harten" Kanten oder grob wirkenden Texturen führen, da keine Glättung erfolgt.
#### Bilineare Interpolation
- **Methode:** Nutzt die vier nächstgelegenen Texel und interpoliert ihre Farben basierend auf den relativen Abständen.
  - Die Farbe eines Pixels wird durch die gewichtete Summe der Texelfarben berechnet.
  - Dies erfolgt in zwei Schritten:
    1. **Horizontale Interpolation:** Zwischen zwei Texeln entlang einer Achse.
    2. **Vertikale Interpolation:** Aus den Ergebnissen der horizontalen Interpolation.
- **Formel:**
  $$
  t = (1-b)t_{12} + b \cdot t_{34}
  $$
  - $t_{12} = t_1(1-a) + t_2a$ (interpoliert horizontal oben)
  - $t_{34} = t_3(1-a) + t_4a$ (interpoliert horizontal unten)
  - Endresultat kombiniert vertikal.
- **Vorteil:** Glättet die Textur sichtbar, besonders bei Vergrößerungen.
- **Nachteil:** Aufwändiger als Nearest Neighbor, da mehrere Texel und Berechnungen benötigt werden.
#### Alternative Interpretation
- Die Gewichte der Texel können als Flächenverhältnisse interpretiert werden (baryzentrisch):
  - Der Flächeninhalt eines Quadrats oder Dreiecks bestimmt den relativen Einfluss eines Texels auf den Zielpixel.

Bitte gib mir kurz Zeit, um die Konzepte zu den hochgeladenen Folien zu erklären. Ich werde die Informationen strukturiert aufbereiten.

#### **Textur-Filtrierung höherer Ordnung**
- **Bikubische Interpolation**:
    - Interpoliert Farben basierend auf 4 × 4 Texel (16 Texel für 2D-Texturen).
    - Nutzt komplexere mathematische Funktionen für glattere Ergebnisse (z. B. Splines).
    - **Nachteil**: Rechenaufwändig und wird nicht direkt von GPU-Hardware unterstützt.
#### **Textur-Filtrierung (Verkleinerung/Minification)**

- **Problem**: Mehrere Texel werden auf einen einzigen Pixel projiziert.
    - Bei nur einem Texel können **Aliasing-Artefakte** auftreten, da hochfrequente Informationen verloren gehen.
    - Beispiel: In der Abbildung "Aliasing" sind störende Muster sichtbar.
- **Lösungen**:
    - **Vorfilterung**: Entfernt hochfrequente Inhalte vor der Abtastung.
    - **Supersampling**: Nutzt eine höhere Abtastauflösung als nötig, um Aliasing zu reduzieren (teuer).
#### **Aliasing und Sampling**

- **Abtasttheorie (Nyquist-Frequenz)**:
    - Signal muss mindestens mit der doppelten Maximalfrequenz abgetastet werden.
    - **Aliasbildung** entsteht, wenn Frequenzanteile überlappen.
    - **Lösung**:
        1. **Vorfilterung** reduziert Frequenzen vor dem Sampling.
        2. **Sinc-Filter** rekonstruiert das Signal ideal (praktisch aber schwer realisierbar).
### Mip-Mapping
![[Pasted image 20241127152548.png|300]]
Mip-Mapping (lateinisch „multum in parvo“ = „viel in wenig“) ist eine Technik der Texturvorfilterung, die in der Computergrafik verwendet wird, um Speicher und Rendering-Zeit zu sparen sowie Aliasing-Effekte zu reduzieren. 
- **Funktionsweise:**
  - **Vorfilterung:** Es werden kleinere Kopien (Mipmap-Stufen) einer Originaltextur erstellt. Jede Stufe hat 1/4 der Fläche der vorherigen, indem beide Dimensionen (Breite und Höhe) halbiert werden.
  - Das führt zu einer Pyramidenstruktur der Texturen (z. B. 64², 32², 16², ...).
  - **Speicherbedarf:** Diese zusätzlichen Texturen benötigen nur etwa 33 % mehr Speicherplatz.
- **Hauptvorteile:**
  - Reduktion von Aliasing: Verhindert „flimmernde“ Texturen bei weit entfernten Objekten.
  - Schnellere Berechnungen: Die GPU entscheidet basierend auf der Distanz, welche Mipmap-Stufe verwendet wird.
- **Nachteil:** Leichte Unschärfe, da Details in niedrigeren Stufen verloren gehen.
### **Tiefpass und Hochpass:**
- **Tiefpass:** Diese Filtertechnik bewirkt eine Glättung der Textur, indem hochfrequente Details reduziert werden. 
  - Beispiel: Die linke Darstellung zeigt eine geglättete Version, bei der Details verschwimmen.
- **Hochpass:** Dieser Filter betont die Kanten und Details in einem Bild, indem die Tiefpass-Version von der Originalversion subtrahiert wird.
  - Beispiel: Die rechte Darstellung hebt die Struktur der Türrahmen und der Wände hervor.
### **Anisotrope Texturfilterung:**
Anisotrope Filterung ist eine Weiterentwicklung des Mip-Mappings, die Verzerrungen reduziert, wenn Texturen unter einem schrägen Winkel betrachtet werden.
- **Problem:** 
  - Mip-Mapping ist isotrop (gleichmäßig in allen Richtungen), während der "Footprint" eines Pixels oft länglich ist.
  - Resultat: Verschwommene Details in schrägen Ansichten.

- **Lösung:** 
  - Anpassung der Vorfilterung an die tatsächliche Form des Pixel-Footprints.
  - RIP-Maps (Rectangular Mip-Maps) werden verwendet, um längliche Formen besser zu behandeln, sind aber speicherintensiv.
### **Summed Area Tables (SAT):**
![[Pasted image 20241127152634.png#invert|200]]
![[Pasted image 20241127152647.png#invert|200]]
SATs sind eine Technik zur effizienten Berechnung von Summen über rechteckige Bereiche in einer Textur.

- **Funktionsweise:**
  - Jedes Element der Tabelle speichert die Summe aller Elemente, die oberhalb und links davon liegen.
  - Mit diesen vorberechneten Summen kann die Summe eines beliebigen Rechtecks in konstanter Zeit berechnet werden.

- **Anwendung:** 
  - Berechnung von Box-Filtern oder weichen Schattierungen in Echtzeit.
  - Beispiel: Die Summe der markierten Bereiche wird mit der Formel berechnet: $D - B - C + A$.

## Bestimmen des Footprints
#### 1. Einfache Möglichkeit, den Footprint zu bestimmen
Ein **Footprint** beschreibt den Bereich im Texturraum, der einem Pixel auf dem Bildschirm entspricht. Das Ziel ist, Texturen möglichst genau abzubilden, ohne Verzerrungen oder Alias-Effekte.
- **Grundidee:**
  - Jeder Pixel auf dem Bildschirm projiziert sich in den Texturraum.
  - Die Ecken des Pixels werden verfolgt, indem ein **Primärstrahl** (Ray) durch sie geschickt wird.
  - Diese Ecken definieren einen **2×2-Pixelblock** im Texturraum.
- **Bestimmung der Größe und Form:**
  - Die **Differenzen der Texturkoordinaten** zwischen den Pixeln liefern die Größe und Form des Footprints.
  - Für verschiedene Verfahren:
    - **Mip-Mapping:** Es wird der größte Wert zwischen Breite und Höhe bestimmt, und daraus wird die Mip-Map-Stufe $n$ gewählt.
    - **RIP-Mapping oder SAT (Summed Area Tables):** Breite und Höhe werden separat behandelt.
- **Filterung:**
  - Nach Berechnung der Stelle im Texturraum wird an dieser Stelle mit der geeigneten Methode gefiltert (z. B. bilineare Interpolation).
#### 2. Ray Differentials zur Bestimmung der Footprints
Dieses Verfahren nutzt die **Ableitung der Texturkoordinaten** im Bildschirmraum, um den Footprint genauer zu bestimmen.
- **Prinzip:**
  - Der **Primärstrahl** trifft auf ein Objekt. Daraus ergeben sich:
    - Die Schnittposition $x$,
    - Die Normale $n$, die die **Tangentialebene** definiert.
  - Die Tangentialebene bildet den Übergang zwischen:
    - Texturkoordinaten (z. B. $(s, t)$) und
    - der Position im Raum (z. B. $(x, y, z)$).
- **Differenzen in den Texturkoordinaten:**
  - Die Ableitung der Texturkoordinaten (meist entlang der Bildschirmachsen) gibt an, wie sich diese Koordinaten verändern. Dies erlaubt Rückschlüsse auf die Größe und Form des Footprints.
- **Allgemeiner Nutzen:**
  - Diese Methode lässt sich verallgemeinern, z. B. für Effekte wie Spiegelungen oder Lichttransmission.
#### Anwendung:
1. In Raytracing-Szenen ermöglicht das Verfahren, **schärfere und genauere Texturdetails** darzustellen, indem es die Texturdetails besser an die Pixelauflösung anpasst.
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
- **Ziel**: Erzeugung von Oberflächendetails durch Manipulation der Normalen ohne geometrische Änderung der Oberfläche.
![[Pasted image 20241206114727.png#invert|300]]
- **Unterschiede**:
  - **Bump Mapping**: Verändert die Helligkeitswerte basierend auf einer Höhe (Graustufen-Textur).
  - **Normal Mapping**: Verwendet eine Farbkodierung (RGB) zur Definition der Normalen.
- **Vorgehen**:
  - Die Normale $N$ wird durch Texturen modifiziert, was zu Änderungen im Beleuchtungsmodell führt.
  - Die berechneten Werte für $(N \cdot L)$ und $(R \cdot V)^n$ ändern sich, was die Oberflächendetails beeinflusst.
### **Displacement Mapping**
![[Pasted image 20241206114809.png|300]]
- **Unterschied zu Bump/Normal Mapping**:
  - **Displacement Mapping** verändert die tatsächliche Geometrie der Oberfläche (z. B. durch GPU-Tessellation).
  - Im Gegensatz zu Bump Mapping wird die Oberfläche physisch verschoben.
- **Beispiel**:
  - Eine flache Oberfläche wird durch eine Höhenkarte (Heightmap) zu einer welligen oder strukturierten Oberfläche transformiert.
- **Vorteil**:
  - Realistische Silhouetten, da die Geometrie tatsächlich angepasst wird.
### **Inverse Displacement Mapping**
- **Prinzip**: Eine Approximation von Displacement Mapping.
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
- **Definition**: Ein Textur-Atlas ist eine spezielle (bijektive) Parametrisierung, bei der jeder Oberflächenpunkt eines 3D-Modells genau einer Stelle in der Textur zugeordnet wird.
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
  - Verwirft Fragmente basierend auf einem Schwellenwert ($\alpha < \text{threshold}$).
  - Nützlich für semitransparente Objekte oder Maskierungen.
- **Verwendung**:
  - Raytracing: Alpha-Kanal (z. B. $k_t$) zur Bestimmung der Transparenz eines Oberflächenpunktes.
  - Rasterisierung: Erzeugung von Aussparungen oder Maskeneffekten durch Alpha-Tests.
## Impostors
![[Pasted image 20241206115414.png|300]]
- **Definition**: Impostors sind texturierte Polygone, die stets senkrecht zur Kamera ausgerichtet sind.
- **Vorteile**:
  - Sehr effizient für weit entfernte Objekte, die keine detaillierte Geometrie benötigen (z. B. Wolken, Rauch, Funken).
  - Reduziert die Renderkosten erheblich, da nur ein einfaches, flaches Polygon verwendet wird.
- **Beispiele**:
  - Darstellung von Partikelsystemen in Spielen oder Animationen.
  - Wolken, die sich nicht dynamisch verändern, aber realistisch aussehen sollen.
## 3D-Texturen für Oberflächen
- **2D-Texturen**:
  - Probleme wie der „Tapeten-Effekt“ entstehen, wenn Texturen wiederholt werden.
  - Schwierige Parametrisierung bei komplexen Objekten.
- **3D-Texturen**:
  - Volumendaten, bei denen jeder Punkt im Volumen eigene Texturkoordinaten hat.
  - Vorteil: Keine Probleme mit Parametrisierung, da die Textur direkt im Volumen gespeichert wird.
  - Nachteil: Hoher Speicherbedarf (z. B. $512^3$ RGB = 384 MB).
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
- Es ist eine schnelle Approximation für Reflexionen und ersetzt komplexe Raytracing-Berechnungen, indem das Bild der Umgebung in einer Textur gespeichert wird.

**Grundprinzip**  
1. **Reflexionsrichtung berechnen**:  
   - Die Richtung $r$ eines Reflexionsstrahls wird basierend auf der Normale $n$ und der Sichtvektorrichtung $v$ am Objekt berechnet.  
   $$
   r = 2 (n \cdot v) n - v
   $$
2. **Zugriff auf Environment Map**:  
   - Die Richtung $r$ wird in Texturkoordinaten umgewandelt, um die passende Farbe aus der Umgebungstextur (Environment Map) zu lesen.
**Annahmen und Einschränkungen**  
- Nur die Richtung $r$ wird verwendet, ohne Berücksichtigung der Entfernung.  
  - Diese Vereinfachung ist akzeptabel, wenn die Umgebung weit entfernt ist (z. B. Himmel oder weite Landschaften).
- Die Umgebung wird auf der inneren Oberfläche einer virtuellen Kugel abgebildet.
**Parametrisierungen**  
- **Latitude/Longitude-Maps**:  
  - Die Kugel wird durch Polarkoordinaten parametriert:
    - $\theta$: Polarwinkel (0 bis $\pi$).
    - $\phi$: Azimuthwinkel (0 bis $2\pi$).  
  - Nachteile:
    - Ungleichmäßige Verteilung der Texturdaten, besonders an den Polen.
    - Höherer Rechenaufwand bei der Umrechnung von $r$ in Texturkoordinaten.
- **Sphere Mapping**:  
  - Darstellung der Umgebung auf einer virtuellen Spiegelkugel.  
  - Bild der Kugel wird als Texture verwendet (z. B. „chrome ball“ Methode).
  - Vorteil: Sehr einfach umzusetzen.
  - Nachteil: Verzerrungen bei ungleichmäßigen Oberflächen.
**Hardwareeinsatz**  
- Environment Mapping wird oft mit GPU-Hardware beschleunigt, besonders bei Echtzeit-Anwendungen wie Spielen.  
- Beim Raytracing wird das Licht aus der Environment Map abgerufen, wenn ein Reflexionsstrahl kein weiteres Objekt trifft.
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
Die Vorfilterung von Environment Maps dient dazu, die Reflexion von Licht auf Oberflächen realistisch darzustellen, indem der Einfluss von Licht aus verschiedenen Richtungen zusammengefasst wird. Dies wird für unterschiedliche Zwecke wie spiegelnde Objekte, imperfekte Spiegelungen oder diffuse Reflexionen verwendet.

**Konzepte der Vorfilterung**  
1. **Reflexionsrichtung ($r$)**:  
   - Für **spiegelnde Objekte** wird die genaue Reflexionsrichtung verwendet, um den Lichtwert aus der Umgebung zu bestimmen.  
   - Beispiel: Ein glänzender Chromball spiegelt exakt das Umgebungsbild wider.
2. **Imperfekte Spiegelung**:  
   - Für Oberflächen mit leicht unscharfen Reflexionen wird Licht aus mehreren Richtungen gewichtet.  
   - Dies orientiert sich am Phong-Modell mit:
     $$
     (r \cdot d)^n
     $$
     wobei $d$ die Richtung des einfallenden Lichts ist und $n$ den Glanzexponenten bestimmt.
3. **Diffuse Reflexion**:  
   - Für matte Oberflächen wird die Menge an Licht berechnet, die von allen möglichen Richtungen kommt und von der Normale $n$ beeinflusst wird.
4. **Kombination von Texturen**:  
   - Oft werden mehrere vorgefilterte Texturen kombiniert, um Reflexionen für unterschiedliche Oberflächenmaterialien darzustellen.

**Techniken der Vorfilterung**  
- **Summed Area Tables**:  
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


