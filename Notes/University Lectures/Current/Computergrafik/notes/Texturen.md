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


