
## Rausch-/Noise-Funktion
- kann <mark style="background: #FFB86CA6;">zur Erzeugung von Texturen</mark> und Mustern genutzt
![[Pasted image 20250128182411.png|600]]
### Eigenschaften
- <mark style="background: #FFB86CA6;">Reporduzierbar</mark>: n(x) liefert immer den gleichen Wert für gleiches x
- <mark style="background: #FFB86CA6;">Keine erkennbare Periodizität</mark>
- <mark style="background: #FFB86CA6;">Begrenzter Wertebereich</mark>
- <mark style="background: #FFB86CA6;">Definierte Frequenzverteilung</mark>
- <mark style="background: #FFB86CA6;">Stetigkeit</mark>: $n(x) \approx x(x+\epsilon)$
### Lattice Value Noise
1. (Pseudo-)Zufallszahlen <mark style="background: #FFB86CA6;">auf einem Gitter</mark>
   - Eine Funktion $Z(\mathbf{x})$ ordnet jedem Punkt $\mathbf{x}$ in $\mathbb{R}^3$ eine Zufallszahl $Z \in [0,1]$ zu.
   - Dabei wird $\mathbf{x} = (x,y,z)$ auf die nächstgelegenen Ganzzahlen $(\lfloor x \rfloor, \lfloor y \rfloor, \lfloor z \rfloor)$ abgebildet.
   - Das bedeutet, dass die Werte nur an den Gitterpunkten zufällig sind, dazwischen sind sie konstant → <mark style="background: #FFB86CA6;">stückweise konstante Werte mit unendlich hohen Frequenzen</mark>.
2. **Räumliche Korrelation und Bandbegrenzung durch Interpolation**
   - <mark style="background: #FFB86CA6;">Damit das Noise „glatter“ erscheint, interpoliert man zwischen den zufälligen Werten</mark>.
   - In 1D wird die Noise-Funktion durch lineare Interpolation berechnet:
     $$
     n(x) = Z(x) \cdot (1 - f_x) + Z(x+1) \cdot f_x
     $$
     wobei $f_x = x - \lfloor x \rfloor$ der Bruchteil von $x$ ist.
   - In 2D und 3D nutzt man bi-/trilineare Interpolation, besser ist **bikubische Interpolation**, da sie glattere Übergänge erzeugt.

3. **Frequenzveränderung durch Skalierung**
   - Wenn man $n(2 \cdot x)$ statt $n(x)$ verwendet, verdoppelt sich die Frequenz des Noise.
#### Interpolation
1. **Nearest Neighbor Interpolation**
   - Einfachste Methode: <mark style="background: #FFB86CA6;">Jeder Punkt erhält den Wert des nächstgelegenen Gitterpunkts</mark>.
   - Ergebnis: <mark style="background: #FFB86CA6;">Sehr kantige, unnatürlich wirkende Oberflächen</mark>.
2. **Trilineare Interpolation**
   - <mark style="background: #FFB86CA6;">Interpoliert zwischen den Werten</mark> der Gitterpunkte, <mark style="background: #FFB86CA6;">erzeugt weichere Übergänge</mark>.
   - Problem: <mark style="background: #FFB86CA6;">Banding-Effekte durch Unstetigkeiten in der ersten Ableitung</mark>.
3. **Trikubische Interpolation**
   - <mark style="background: #FFB86CA6;">Zweimal stetig differenzierbar, also keine sichtbaren Unstetigkeiten</mark>.
   - Erzeugt **glatte Übergänge**, ideal für organische Strukturen.
![[Pasted image 20250128182625.png|600]]

#### **Zwei Methoden zur Implementierung**
1. **Naiver Ansatz**
   - <mark style="background: #FFB86CA6;">Man speichert ein großes 2D/3D-Gitter mit Zufallszahlen</mark>.
   - Problem: <mark style="background: #FF5582A6;">Hoher Speicherverbrauch und Periodizität</mark>, daher unpraktisch.

2. **Eleganter Ansatz mit Hash-Funktion**
   - <mark style="background: #FFB86CA6;">Man erstellt ein 1D-Array</mark> `rtab[256]` mit Zufallswerten.
   - <mark style="background: #FFB86CA6;">Zugriff erfolgt über eine Hash-Funktion</mark>:
     $$
     \text{Index}(\lfloor x \rfloor, \lfloor y \rfloor, \lfloor z \rfloor) = P\left(\lfloor x \rfloor + P\left(\lfloor y \rfloor + P(\lfloor z \rfloor)\right)\right)
     $$
   - $P(\cdot)$ ist eine Permutation, die zyklische Muster verhindert.
   - Dadurch kann Noise <mark style="background: #BBFABBA6;">speichereffizient</mark> erzeugt werden.

### **Spektrale Synthese durch Kombination von Frequenzen**
<mark style="background: #FFB86CA6;">Noise kann durch Überlagerung von Frequenzbereichen komplexer gemacht werden</mark>.
### **Konzepte**
1. **Turbulenz-Funktion**
   - <mark style="background: #FFB86CA6;">Summe mehrerer Noise-Funktionen mit steigender Frequenz und abnehmender Amplitude</mark>:
     $$
     \text{turbulence}(\mathbf{x}) = \sum_k \left(\frac{1}{f}\right)^k n(f^k \cdot \mathbf{x})
     $$
   - <mark style="background: #FFB86CA6;">Höhere Oktaven erzeugen feinere Strukturen</mark>.

2. **Frequenzspektrum**
   - Typischerweise $1/f$ → mit $f=2$ entsteht **rosa Rauschen**.
   - Rosa Rauschen ist visuell ansprechend, da es **natürliche Strukturen imitiert**.
### **Implizite vs. Explizite Methoden**
1. **Implizite Methode**
   - <mark style="background: #FFB86CA6;">Berechnung direkt im Shader für jedes Pixel</mark>.
   - Vorteil: <mark style="background: #FFB86CA6;">Kein Speicherverbrauch für Texturen.</mark>
   - Nachteil: <mark style="background: #FFB86CA6;">Langsame Berechnung</mark>, besonders bei komplexen Shadern.

2. **Explizite Methode**
   - <mark style="background: #FFB86CA6;">Vorberechnung einer 2D/3D-Textur und Speicherung</mark>.
   - Vorteil: <mark style="background: #FFB86CA6;">Schnelle Darstellung mit normalem Texture-Mapping</mark>.
   - Nachteil: <mark style="background: #FFB86CA6;">Erhöhter Speicherverbrauch</mark>.

3. **Kombination beider Methoden**
   - <mark style="background: #FFB86CA6;">On-Demand-Synthese und Caching</mark> ermöglicht Effizienzsteigerung.

## Filterung bei Turbulenz
Die Turbulenz-Funktion wird als Summe von Noise-Funktionen mit unterschiedlichen Frequenzen dargestellt:

$$
\text{turbulence}(\mathbf{x}) = \sum_k \left(\frac{1}{f}\right)^k n(f^k \mathbf{x})
$$

### **Warum ist das Filtern von hohen Frequenzen wichtig?**
- <mark style="background: #FFB86CA6;">Hohe Frequenzanteile können zu Aliasing führen</mark>, insbesondere wenn das Noise für Texturen in Computergrafik verwendet wird.
### **Lösung: Höhere Oktaven weglassen**
- Da höhere Oktaven feine Strukturen hinzufügen, führt ihr Weglassen zu einem glatteren Ergebnis.
- Dies ist eine Art <mark style="background: #FFB86CA6;">Tiefpassfilter</mark>, der unerwünschte hochfrequente Details unterdrückt.
## Hypertextures (Perlin 1989)
### **Hauptkonzept: Erzeugung von komplexen 3D-Volumenmodellen**
<mark style="background: #FFB86CA6;">Hypertextures sind eine Methode zur Veränderung volumetrischer Strukturen wie Wolken, Rauch oder Lava</mark>. Sie basieren auf einer <mark style="background: #FFB86CA6;">Dichtefunktion</mark> (Density Function, DF):

$$
D(\mathbf{x})
$$

wobei:
- $D(\mathbf{x}) = 1$ für <mark style="background: #FFB86CA6;">opake Bereiche</mark> (volle Materie).
- $D(\mathbf{x}) = 0$ für <mark style="background: #FFB86CA6;">transparente Bereiche</mark> (Luft).
- **Zwischenwerte** definieren weiche Übergänge, die für **realistische volumetrische Effekte** sorgen.
### **Analogie zu Displacement Mapping**
- <mark style="background: #FFB86CA6;">Während Displacement Mapping nur die Oberfläche eines Objekts modifiziert, verändert Hypertexture die gesamte Volumenstruktur</mark>.
- Beispiel: **Ein animiertes Feuer kann mit Hypertextures realistisch modelliert werden.**
## Distanzfelder, Distanzfunktionen
### **Hauptkonzept: Implizite Darstellung von Objekten**
<mark style="background: #FFB86CA6;">Anstatt Objekte durch Polygone zu beschreiben, kann man sie durch eine Distanzfunktion definieren</mark>:

$$
f(\mathbf{x})
$$

Diese Funktion <mark style="background: #FFB86CA6;">gibt die kürzeste Entfernung zur Oberfläche eines Objekts</mark> zurück.
### **Eigenschaften der Distanzfunktion**
- **Isosurface $f(\mathbf{x}) = 0$**: Die <mark style="background: #FFB86CA6;">eigentliche Oberfläche</mark> des Objekts.
- **Positive Werte $f(\mathbf{x}) > 0$**: Punkte außerhalb des Objekts.
- **Negative Werte $f(\mathbf{x}) < 0$**: Punkte innerhalb des Objekts.
### **Beispiel: Kugel**
Die Distanzfunktion einer Kugel mit Mittelpunkt $\mathbf{m}$ und Radius $r$:

$$
f(\mathbf{x}) = |\mathbf{x} - \mathbf{m}| - r
$$

Diese Methode ist besonders nützlich für **raymarching-basierte Renderverfahren**.
## Sphere Tracing – Schnittpunktberechnung
### **Hauptkonzept: Effiziente Raymarching-Methode mit Distanzfeldern**
**Sphere Tracing** ist eine Technik, <mark style="background: #FFB86CA6;">um den Schnittpunkt eines Strahls mit einem Objekt in einem Distanzfeld zu berechnen</mark>.

### **Wie funktioniert Sphere Tracing?**
1. **Start an einem Punkt $\mathbf{x}$ entlang des Strahls.**
2. **Berechne die Distanz zur Oberfläche $f(\mathbf{x})$.**
3. **Bewege dich um genau diese Distanz weiter.**
4. **Wiederhole, bis der Wert nahe 0 ist** → Schnittpunkt gefunden.
Vorteil:
- **Schneller als traditionelles Raymarching**, da es große Schritte nimmt.
- **Kein Risiko, einen Schnittpunkt zu verpassen**, solange die Distanzfunktion korrekt definiert ist.
## Verknüpfung von Distanzfunktionen (CSG)
### **Hauptkonzept: Constructive Solid Geometry (CSG)**
Mit Distanzfunktionen kann man <mark style="background: #FFB86CA6;">boolesche Operationen auf Objekten durchführen</mark>.
### **Operationen**
1. **Vereinigung**:

   $$
   f_{A \cup B}(\mathbf{x}) = \min(f_A(\mathbf{x}), f_B(\mathbf{x}))
   $$

   - <mark style="background: #FFB86CA6;">min()</mark>
   - Das resultierende Objekt ist das Minimum der beiden Distanzwerte.
   - Beispiel: Zwei Kugeln verschmelzen zu einer.

2. **Schnittmenge**:

   $$
   f_{A \cap B}(\mathbf{x}) = \max(f_A(\mathbf{x}), f_B(\mathbf{x}))
   $$

   - <mark style="background: #FFB86CA6;">max()</mark>
   - Nur der gemeinsame Bereich bleibt erhalten.
   - Beispiel: Ein Quader wird in eine Kugel geschnitten.

3. **Differenz**:

   $$
   f_{A \setminus B}(\mathbf{x}) = \max(f_A(\mathbf{x}), -f_B(\mathbf{x}))
   $$

   - <mark style="background: #FFB86CA6;">max(.,-)</mark>
   - Das eine Objekt wird vom anderen subtrahiert.
   - Beispiel: Ein <mark style="background: #FFB86CA6;">Loch in einen Würfel schneiden</mark>.
## Textursynthese
- <mark style="background: #FFB86CA6;">Oft liegt eine kleine Beispieltextur (Exemplar) vor</mark>, die aus einer Fotografie extrahiert wird.
- Ziel ist es, <mark style="background: #FFB86CA6;">daraus eine größere, realistisch aussehende Textur zu erzeugen</mark>.
### **Problem: Tiling (Kachelung)**
- Wenn eine kleine Textur einfach wiederholt wird, <mark style="background: #FFB86CA6;">entstehen sichtbare Wiederholungsmuster</mark> (linkes Bild).
### **Hauptziel**
- <mark style="background: #FFB86CA6;">Eine größere Textur erzeugen, die gleich aussieht, aber nicht einfach eine Kopie des Exemplars ist</mark>.
- „Gleich aussehen“ bedeutet hier, dass die **statistischen Eigenschaften** übereinstimmen.
### **Wichtige Aspekte**
- <mark style="background: #FFB86CA6;">Variation erzeugen</mark>: Die generierte Textur sollte <mark style="background: #FFB86CA6;">nicht exakt gleich sein, sondern organisch wirken</mark>.
- <mark style="background: #FFB86CA6;">Nur für stochastische Muster geeignet: Semantische Strukturen (z. B. Blätter oder Gesichter) lassen sich nicht gut</mark> durch reine Textursynthese erzeugen.
### Pixelbasierte Textursynthese
#### **Prinzip**
- Die <mark style="background: #FFB86CA6;">Ausgabetextur wird Pixel für Pixel erzeugt</mark>.
- Beispiel: **Zeilenweise von links nach rechts und von oben nach unten**.
#### **Annahme**
- Ein Teil der Textur wurde bereits erzeugt oder kopiert.
- Das bedeutet, dass <mark style="background: #FFB86CA6;">neue Pixel basierend auf den vorhandenen Pixeln berechnet werden</mark>.
### **Wie funktioniert die Synthese?**
- Beim Generieren eines neuen Pixels wird <mark style="background: #FFB86CA6;">die Nachbarschaft betrachtet</mark>.
- Typischerweise werden <mark style="background: #FFB86CA6;">30–100 Pixel in der Umgebung</mark> analysiert.
- Diese Nachbarschaft <mark style="background: #FFB86CA6;">muss groß genug sein, um Strukturen in der Textur zu erfassen</mark>.
![[Pasted image 20250128183340.png|500]]
### **Wie wird entschieden, welcher Pixel generiert wird?**
- Die Methode <mark style="background: #FFB86CA6;">sucht in der Eingabetextur nach ähnlichen Nachbarschaften</mark>.
- **Ähnlich** bedeutet, dass <mark style="background: #FFB86CA6;">die Summe der Farbabweichungen klein ist</mark>.
![[Pasted image 20250128183357.png|500]]
### **Wie wird der nächste Pixel bestimmt?**
- <mark style="background: #FFB86CA6;">Einer der ähnlichsten Bereiche wird zufällig gewählt</mark>.
- **Zufällige Auswahl** <mark style="background: #FFB86CA6;">sorgt für stochastische Variation, sodass keine Wiederholungen entstehen</mark>.
### **Ablauf**
1. Suche ähnliche Nachbarschaften in der Eingabetextur.
2. **Kopiere einen Pixel** aus der ähnlichsten Region.
3. Wiederhole diesen Vorgang, bis alle Pixel der neuen Textur berechnet wurden.

## Lindenmayer-Systeme (L-Systeme)
- Modellierung biologischer Wachstumsprozesse.
- Basiert auf **formalen Grammatiken**
### **Grundprinzip**
- <mark style="background: #FFB86CA6;">Sukzessives Ersetzen von Zeichen nach bestimmten Regeln</mark> → Erzeugung komplexer Strukturen aus einfachen Elementen.
![[Pasted image 20250128183717.png#invert|500]]
### Eigenschaften von Fraktalen
- <mark style="background: #FFB86CA6;">Selbstähnlichkeit: Kleine Teile ähneln dem gesamten Objekt</mark>.
- <mark style="background: #FFB86CA6;">Nicht-ganzzahlige Dimension: Fraktale haben oft eine fraktale Dimension zwischen zwei ganzzahligen Werten</mark>.
- Beispiele in der Natur: **Berge, Flüsse, Blitze, Galaxien**.
Formale Definition:
$$D = \frac{\log(N)}{\log(S)}$$
wobei:
- $D$ die fraktale Dimension ist,
- $N$ die <mark style="background: #FFB86CA6;">Anzahl der Kopien in kleinerem Maßstab</mark>,
- $S$ der <mark style="background: #FFB86CA6;">Skalierungsfaktor</mark>.
### L-Systeme als Ersetzungssysteme
Ein **L-System** besteht aus einem **Quadruppel**:
$$
G = (V, \Sigma, S, P)
$$
- **$V$**: Alphabet (Zeichenmenge).
- **$\Sigma \subset V$**: Terminalsymbole.
- **$S \in V^*$**: Startwort.
- **$P \subset (V^* \backslash \Sigma^*) \times V^*$**: Produktionsregeln.
### **Unterschied zu normalen Grammatiken**
- **Regeln werden parallel angewendet** → realistische Modellierung biologischer Prozesse.
### Turtle-Grafik für L-Systeme
- „Bildbeschreibungssprache“ für geometrische Konstruktionen.
- Schildkröte bewegt sich mit Position $(x,y)$ und Richtung $\alpha$.
#### **Typische Kommandos**
- **$F$**: Vorwärtsbewegung mit Länge $d$.
- **$f$**: Vorwärtsbewegung ohne Zeichnen.
- **$+$**: Drehung um Winkel $\delta$.
- **$-$**: Drehung um Winkel $-\delta$.

Neue Position:
$$
x' = x + d \cos \alpha, \quad y' = y + d \sin \alpha
$$
### **Definition eines L-Systems mit Turtle-Kommandos**
$$
G = (V, \Sigma, S, P)
$$
- **Alphabet**: $V = \{F, +, -\}$
- **Startwort**: $S = F + F + F + F$
- **Regeln**: $P = \{ F \mapsto F + F - F - FF + F + F - F \}$
- **Winkel**: $\alpha = 90^\circ$

→ Erzeugung **fraktaler Muster** durch rekursive Anwendung der Regeln.
![[Pasted image 20250128183937.png#invert|600]]