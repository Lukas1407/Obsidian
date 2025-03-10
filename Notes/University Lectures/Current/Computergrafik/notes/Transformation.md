- **Definition**: Eine Transformation ist eine mathematische Abbildung, die <mark style="background: #FFB86CA6;">einen Punkt</mark> $x$ in <mark style="background: #FFB86CA6;">einem Raum auf einen anderen Punkt</mark> $x'$ abbildet.
- <mark style="background: #FFB86CA6;">Lineare Transformationen</mark>:
  - Werden durch Matrizen dargestellt: $x' = T(x) = A \cdot x$, wobei $A$ die <mark style="background: #FFB86CA6;">Transformationsmatrix</mark> ist.
  - Beispiele: <mark style="background: #FFB86CA6;">Verschiebungen, Rotationen, Skalierungen</mark>.

**Anwendungen in der Computergrafik**:
- Platzierung von Objekten in einer Szene.
- Animationen und Deformationen.
- Kamerabewegungen und Projektionen (z. B. 3D in 2D umwandeln).
- Texturierung, Spiegelungen, Schatteneffekte.
## **Koordinatensysteme in der Computergrafik**
![[Pasted image 20241127125454.png#invert|300]]
In der Computergrafik werden verschiedene Koordinatensysteme verwendet, um Objekte zu beschreiben und zu manipulieren:
- **Objekt- oder Modell-Koordinatensystem**:
  - <mark style="background: #FFB86CA6;">Definiert die Geometrie eines Objekts unabhängig von der Szene</mark>.
  - Transformation: Modelltransformation (Verschieben, Skalieren, Rotieren).
- **Weltkoordinatensystem**:
  - <mark style="background: #FFB86CA6;">Beschreibt die Position der Objekte relativ zur gesamten Szene</mark>.
  - Transformation: Kameratransformation (Blickwinkel und Perspektive der Kamera).
- **Kamerakoordinatensystem**:
  - <mark style="background: #FFB86CA6;">Objekte werden relativ zur Kamera dargestellt</mark>.
  - Anwendungen: Raytracing, Projektionen in den Bildschirmraum.
### **Transformationsgruppen**
![[Pasted image 20241127125514.png#invert|400]]
- <mark style="background: #FFB86CA6;">Euklidische Transformationen</mark>:
  - <mark style="background: #FFB86CA6;">Erhalten Abstände und Winkel</mark> (z. B. <mark style="background: #ADCCFFA6;">Rotation, Translation</mark>).
- <mark style="background: #FFB86CA6;">Lineare Abbildungen</mark>:
  - <mark style="background: #ADCCFFA6;">Skalierung</mark>, Spiegelung, Scherung.
  - Eigenschaften: additive und homogene Operationen (lineare Algebra).
- **Affin**:
  - <mark style="background: #FFB86CA6;">Parallelität bleibt erhalten</mark>.
  - Enthält Translationen, Skalierungen, Scherungen.
- **Projektiv**:
  - Linien bleiben Linien, aber Perspektive wird hinzugefügt.
  - Beispiel: Perspektivische Abbildungen für 3D-Szenen.
### **Affine und Projektive Abbildungen**
- **Affine Abbildungen**:
  - Beispiele: Skalierungen, Scherungen, Spiegelungen.
  - Anwendungen: Manipulationen wie Verzerrungen von Objekten.
- **Projektive Abbildungen**:
  - Perspektivische Transformationen: Objekte erscheinen kleiner, je weiter sie entfernt sind.
  - Anwendung: 3D-Projektionen auf eine 2D-Fläche (Bildschirm).

## 2D Transformationen
#### **Grundlegende Transformationen**
![[Pasted image 20241127125753.png#invert|400]]
1. **Lineare 2D-Transformationen**:
   - Werden durch **Vektor-Matrix-Multiplikation** dargestellt:
     $$
     \begin{pmatrix} x' \\ y' \end{pmatrix} = \begin{pmatrix} a_{11} & a_{12} \\ a_{21} & a_{22} \end{pmatrix} \begin{pmatrix} x \\ y \end{pmatrix}
     $$
   - Beispiele:
     - **Skalierung** (Größenänderung).
     - **Scherung** (Verzerrung).
     - **Spiegelung** (Reflektion).
     - **Rotation** (Drehung).
#### **Skalierung**
- Skalierung verändert die Größe eines Objekts:
  - **Uniform**: Gleiche Skalierung in $x$- und $y$-Richtung ($s_x = s_y$).
  - **Nicht-uniform**: Unterschiedliche Skalierung ($s_x \neq s_y$).
  - Matrixform:
    $$
    \text{Scale}(s_x, s_y) = \begin{pmatrix} s_x & 0 \\ 0 & s_y \end{pmatrix}
    $$
  - Beispiel: Ein Rechteck wird in $y$-Richtung um den Faktor 1.5 gestreckt.
#### **Spiegelung**
- **Definition**: Negative Skalierung, die an einer Achse spiegelt:
  - An der $y$-Achse: $x' = -x, y' = y$.
  - Matrix:
    $$
    \begin{pmatrix} -1 & 0 \\ 0 & 1 \end{pmatrix}
    $$
  - An der $x$-Achse: $x' = x, y' = -y$.
#### **Scherung**
- **Definition**: Verschiebung proportional zum Abstand von einer Achse.
  - **Horizontal** (entlang der $x$-Achse):
    $$
    \text{Shear}_x(s) = \begin{pmatrix} 1 & s \\ 0 & 1 \end{pmatrix}
    $$
  - **Vertikal** (entlang der $y$-Achse):
    $$
    \text{Shear}_y(s) = \begin{pmatrix} 1 & 0 \\ s & 1 \end{pmatrix}
    $$
#### **Rotation**
- **Definition**: Drehung eines Punktes oder Objekts um einen Winkel $\phi$:
  - Matrixform:
    $$
    R(\phi) = \begin{pmatrix} \cos \phi & -\sin \phi \\ \sin \phi & \cos \phi \end{pmatrix}
    $$

#### **Zusammengesetzte Transformationen**
- Mehrere Transformationen werden durch Matrix-Multiplikation kombiniert:
  - Reihenfolge ist entscheidend ($RS \neq SR$).
  - Beispiel: Skalierung gefolgt von Rotation:
    $$
    \mathbf{v}_3 = R(S\mathbf{v}_1)
    $$
## **3D Transformationen**
#### **Rotationen in 3D**
- Rotationen erfolgen um eine der drei Achsen:
  - **Rotation um die $x$-Achse**:
    $$
    R_x(\phi) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos \phi & -\sin \phi \\ 0 & \sin \phi & \cos \phi \end{pmatrix}
    $$
  - **Rotation um die $y$-Achse**:
    $$
    R_y(\phi) = \begin{pmatrix} \cos \phi & 0 & \sin \phi \\ 0 & 1 & 0 \\ -\sin \phi & 0 & \cos \phi \end{pmatrix}
    $$
  - **Rotation um die $z$-Achse**:
    $$
    R_z(\phi) = \begin{pmatrix} \cos \phi & -\sin \phi & 0 \\ \sin \phi & \cos \phi & 0 \\ 0 & 0 & 1 \end{pmatrix}
    $$
#### **Zusammengesetzte Rotationen**
- Durch Matrix-Multiplikation kombiniert.
- Beispiele:
  - Euler-Winkel: Kombination von Rotationen um $x$-, $y$- und $z$-Achse.
#### **Repräsentation von Rotationen**
- <mark style="background: #FFB86CA6;">Matrixdarstellungen</mark>.
- <mark style="background: #FFB86CA6;">Quaternions: Kompakte Darstellung für Rotationen in 3D</mark>.
- <mark style="background: #FFB86CA6;">Euler-Winkel: Sequentielle Rotationen</mark>.
**Anwendungen in der Computergrafik:**
- Platzierung von Objekten in 3D-Szenen.
- Kamerabewegungen.
- Animationen und Deformationen.
- Perspektivische Projektionen auf 2D-Bildschirme.

### **3D Transformationen: Rotation**
#### **Orthogonale Matrizen für Rotationen**
- Rotationen werden durch **orthogonale Matrizen** dargestellt.
- **Eigenschaften einer orthogonalen Matrix $M$:**
  - $M^\top \cdot M = M \cdot M^\top = I$ (Einheitsmatrix).
  - $|\det(M)| = 1$: Matrizen, die Orientierung erhalten.
  - Die Inverse ist gleich der Transponierten: $M^{-1} = M^\top$.
  - Solche Matrizen beschreiben sowohl algebraisch als auch geometrisch gültige Rotationen.
#### **Rotation zwischen Koordinatensystemen**
- Gegeben: Drei Basisvektoren $\mathbf{u}, \mathbf{v}, \mathbf{w}$, die ein orthonormales System bilden.
- Rotationsmatrix:
  $$
  R_{\mathbf{uvw}} = \begin{pmatrix}
  u_x & u_y & u_z \\
  v_x & v_y & v_z \\
  w_x & w_y & w_z
  \end{pmatrix}
  $$
  - Diese Matrix transformiert Vektoren in das Koordinatensystem $\mathbf{u}, \mathbf{v}, \mathbf{w}$.
  - Anwendung: $R_{\mathbf{uvw}} \cdot \mathbf{p}$ berechnet die Projektion eines Punktes $\mathbf{p}$ auf die Achsen von $\mathbf{u}, \mathbf{v}, \mathbf{w}$.
#### **Transformation aus dem lokalen in das globale Koordinatensystem**
- Die Transponierte $R_{\mathbf{uvw}}^\top$ transformiert Punkte aus dem lokalen in das globale Koordinatensystem.
- Dies geschieht durch Multiplikation:
  $$
  R_{\mathbf{uvw}}^\top \cdot \mathbf{p}
  $$
  - Dies stellt die Umkehrung der Transformation dar.
### **Rotation um eine beliebige Achse**
- Rotationen können um jede beliebige Achse $\mathbf{d}$ durchgeführt werden.
- Schritte:
  1. Wähle ein orthogonales Koordinatensystem:
     - $\mathbf{d}$: Die Rotationsachse.
     - $\mathbf{e}, \mathbf{f}$: Zusätzliche orthogonale Basisvektoren.
  2. Erstelle eine Matrix $M$, die das lokale Koordinatensystem beschreibt:
     $$
     M = \begin{pmatrix}
     \mathbf{d}^\top \\
     \mathbf{e}^\top \\
     \mathbf{f}^\top
     \end{pmatrix}
     $$
  3. Berechne die Rotation um die $x$-Achse ($\mathbf{d}$-Achse):
     $$
     R_{d, \phi} = M^{-1} R_x(\phi) M
     $$


![[Euler Winkel]]

## Inverse Transformationen
1. **Matrixinversion und Transformationen**
   - Eine **inverse Matrix** repräsentiert die inverse geometrische Transformation.
   - Berechnung der Inversen:
     - Standardmethoden wie Cramersche Regel, LU-Zerlegung oder Gauss-Elimination sind rechenaufwändig und anfällig für numerische Fehler.
   - Effizienter Ansatz: Nutze die Eigenschaften spezieller Matrizen.
     - **Skalierung**: Inverse ist die Umkehrung der Skalierungsfaktoren:
       $$
       S^{-1}(x, y, z) = S\left(\frac{1}{x}, \frac{1}{y}, \frac{1}{z}\right)
       $$
     - **Rotation**: Transponierte Matrix ist die Inverse, <mark style="background: #FFB86CA6;">da Rotationsmatrizen orthogonal</mark> sind:
       $$
       R^{-1}(\phi) = R^\top(\phi)
       $$
     - **Translation**: Inverse ist eine Verschiebung in die entgegengesetzte Richtung:
       $$
       T^{-1}(x, y, z) = T(-x, -y, -z)
       $$
   - Für zusammengesetzte Transformationen gilt:
     $$
     (AB)^{-1} = B^{-1}A^{-1}
     $$

2. **Nicht-invertierbare Transformationen**
   - Projektionen sind oft nicht invertierbar, da sie Informationen verlieren (z. B. Skalierung auf $S(0,0,0)$).

### **Beispiele für Transformationen**
1. **2D Translation**
   - Transformation eines Ortsvektors:
     $$
     \begin{pmatrix}
     x' \\ y' \\ 1
     \end{pmatrix} = 
     \begin{pmatrix}
     1 & 0 & \Delta x \\
     0 & 1 & \Delta y \\
     0 & 0 & 1
     \end{pmatrix}
     \begin{pmatrix}
     x \\ y \\ 1
     \end{pmatrix}
     $$

2. **Rotation um einen Punkt**
   - Schritte:
     1. Verschiebe Ursprung zum Rotationszentrum.
     2. Führe Rotation durch.
     3. Verschiebe Ursprung zurück.
### **Grundlegende 3D-Transformationen**
1. **Skalierung und Scherung**
   - Skalierung entlang der Achsen:
     $$
     \text{scale}(s_x, s_y, s_z) =
     \begin{pmatrix}
     s_x & 0 & 0 & 0 \\
     0 & s_y & 0 & 0 \\
     0 & 0 & s_z & 0 \\
     0 & 0 & 0 & 1
     \end{pmatrix}
     $$

2. **Rotationen**
   - Beispiel: Rotation um die $z$-Achse:
     $$
     R_z(\phi) =
     \begin{pmatrix}
     \cos \phi & -\sin \phi & 0 & 0 \\
     \sin \phi & \cos \phi & 0 & 0 \\
     0 & 0 & 1 & 0 \\
     0 & 0 & 0 & 1
     \end{pmatrix}
     $$
### **Transformation von Normalen**

- **Normale**:
  - Ein Normalvektor ist ein Vektor, der senkrecht auf einer Oberfläche steht.
  - Normalen sind wichtig für Schattierung und Beleuchtung.
  
- **Transformation**:
  - Normalen können nicht direkt wie Objekte transformiert werden, da sie durch affine Transformationen (z. B. Scherungen oder Skalierungen) ihre Richtung oder Länge verlieren könnten.
  - Lösung: Die Normale wird durch die **invers-transponierte Matrix** transformiert:  
    $n' = (M^{-1})^T n$.

- **Wichtig**:
  - Nur die Richtung der Normale bleibt bei Skalierung und Scherung korrekt erhalten, die Länge jedoch nicht.
#### **Beispiele**
![[Pasted image 20241127130908.png#invert|300]]
- **Falsche Transformation**:
  - Wird eine Normale wie ein Objekt transformiert (z. B. einfach mit $M$), kann sie schief stehen und nicht mehr senkrecht zur Oberfläche sein.
- **Korrekte Transformation**:
  - Mit der invers-transponierten Matrix bleibt die Normale korrekt, d. h. sie steht weiterhin senkrecht zur Oberfläche.

### Erklärung der Konzepte auf den Folien zu Transformationen und Schnitttests

#### Schnittpunktberechnung in Modell- oder Weltkoordinaten?
- **Modelltransformationen**: Objektkoordinaten werden in Weltkoordinaten transformiert. 
  - Beispiele:
    - **Dreieck**: Die Eckpunkte des Dreiecks werden direkt transformiert.
    - **Kugel**: Transformiert den Mittelpunkt und den Radius (bei isotroper Skalierung). Bei Scherung ist die Kugel jedoch nicht mehr perfekt kugelförmig.
  - Herausforderung: Scherung und nicht isotrope Skalierung machen die mathematische Behandlung schwieriger.
- **Empfohlene Lösung**: Schnittpunktberechnung in **Modellkoordinaten**, weil:
  - Transformationen sind einfacher.
  - Rechenoperationen werden auf den Strahl angewendet, nicht auf das Objekt.

---

#### Transformationen und Schnitttests
- **Zwei Ansätze**:
  1. **Schnittpunktberechnung in Weltkoordinaten**:
     - Transformationen werden für jedes Primitive (z. B. Dreiecke, Kugeln) separat gehandhabt.
     - Nachteil: Komplexer und ineffizient, vor allem bei vielen ähnlichen Objekten.
     - Vorteil: Nützlich bei statischen Modellen.
  2. **Schnittpunktberechnung in Modellkoordinaten**:
     - Transformiere den Strahl in Modellkoordinaten und berechne den Schnitt.
     - Vorteil: Effizienter für instanziierte Objekte, die eine gemeinsame Transformation nutzen.

---

#### Transformation des Strahls
- Der Strahl wird von Welt- in Objektkoordinaten transformiert, da die Schnittberechnung einfacher ist.
- **Transformationen**:
  - Ein Punkt im Weltkoordinatensystem ($p_{ws}$) wird durch die Matrix $M$ aus Objektkoordinaten ($p_{os}$) berechnet:
    - $p_{ws} = M \cdot p_{os}$
  - Umgekehrt:
    - $p_{os} = M^{-1} \cdot p_{ws}$

---

#### Strahlursprung und Strahlrichtung transformieren
- **Strahlursprung**: $e_{os} = M^{-1} \cdot e_{ws}$
- **Strahlrichtung**: $d_{os} = M^{-1} \cdot d_{ws}$
  - Hier wird die Richtung als Differenz zweier Punkte behandelt, weshalb die Matrix-Inversion direkt genutzt werden kann.

---

#### Normierung der Strahlrichtung
- $d_{os}$ ist oft nicht normiert (Länge ≠ 1), vor allem bei Skalierung durch $M$.
- **Optionen**:
  1. **Normieren**: Verwende $d_{os} / |d_{os}|$.
  2. **Nicht normieren**: Behandle $d_{os}$ direkt, passe jedoch die Berechnung der Schnittpunkte an.
- Unterschiedliche Fälle:
  - Bei Normierung muss der Skalierungsfaktor später korrigiert werden ($t_{ws} \neq t_{os}$).
  - Ohne Normierung: Schnittberechnung ist direkt.

---

#### Fazit: Transformationen und Schnitttests
- Vorteil der Strahltransformation:
  - Effizienter, insbesondere für instanziierte Modelle (z. B. Dreiecksnetze).
  - Nur eine Transformation pro Strahl notwendig, nicht pro Objekt.
- **Wichtige Überlegungen**:
  - Transformationen sollten nur angewandt werden, wenn notwendig, da Matrixinversion und Transformation kostenintensiv sind.
  - Wiederverwendung von Transformationen durch Modellhierarchien und Szenengraphen optimiert die Berechnung.

Falls du ein bestimmtes Detail vertieft haben möchtest, lass es mich wissen!

