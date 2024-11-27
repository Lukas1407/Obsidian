![[Pasted image 20241127130519.png#invert|400]]
1. **Objekt-, Welt- und Kamerakoordinaten**
   - **Objekt- oder Modell-Koordinatensystem**:
     - Jedes Objekt hat sein eigenes Koordinatensystem, das für die Modellerstellung genutzt wird.
     - Transformationen (z. B. Platzierung im Raum) erfolgen relativ zu diesem System.
   - **Weltkoordinatensystem**:
     - Globale Referenz für die Szene. Hier werden Objekte durch Translation, Rotation oder Skalierung platziert.
   - **Kamerakoordinatensystem**:
     - Perspektive der Kamera. Transformationen von Welt- zu Kamerakoordinaten sind essenziell für das Rendern.
   - Beim **Raytracing** wird außerdem die Transformation von **Sichtstrahlen** in Weltkoordinaten benötigt.

2. **Transformationen zwischen den Systemen**
   - **Modelltransformation**:
     - Wandelt Objektkoordinaten in Weltkoordinaten um.
   - **Kameratransformation**:
     - Wandelt Weltkoordinaten in Kamerakoordinaten um.
### **Wechsel zwischen Koordinatensystemen**

1. **Globale und lokale Koordinatensysteme**
   - **Globales Koordinatensystem (Welt)**:
     - Ursprung $\mathbf{0}$ und Basisvektoren $\mathbf{x}, \mathbf{y}$ definieren die Referenz.
   - **Lokales Koordinatensystem (Objekt)**:
     - Ursprung $\mathbf{e}$ und Basisvektoren $\mathbf{u}, \mathbf{v}$ definieren das Koordinatensystem eines Objekts.

2. **Transformation eines Punktes $P$**
   - Der Punkt $P$ hat Koordinaten $(p_x, p_y)$ im globalen und $(p_u, p_v)$ im lokalen System.
   - **Umrechnung zwischen den Systemen**:
     - Transformation von global nach lokal:
       $$
       \begin{pmatrix}
       p_u \\ p_v \\ 1
       \end{pmatrix}
       = 
       \begin{pmatrix}
       u_x & u_y & -e_x \\ 
       v_x & v_y & -e_y \\
       0 & 0 & 1
       \end{pmatrix}
       \begin{pmatrix}
       p_x \\ p_y \\ 1
       \end{pmatrix}
       $$
     - Transformation von lokal nach global:
       $$
       \begin{pmatrix}
       p_x \\ p_y \\ 1
       \end{pmatrix}
       = 
       \begin{pmatrix}
       u_x & u_y & e_x \\ 
       v_x & v_y & e_y \\
       0 & 0 & 1
       \end{pmatrix}
       \begin{pmatrix}
       p_u \\ p_v \\ 1
       \end{pmatrix}
       $$
### **Projektive Abbildungen**

1. **Definition**
   - Projektive Abbildungen modellieren die Projektion auf eine Bildebene, wie sie in der Kamerarendering-Pipeline verwendet wird.
   - **Ziel**: Punkte im 3D-Raum auf eine 2D-Bildebene projizieren.

2. **Beispiel: Projektion auf $z = 1$**
   - Die Projektion erfolgt durch eine Matrixmultiplikation:
     $$
     \begin{pmatrix}
     x' \\ y' \\ z'
     \end{pmatrix}
     =
     \begin{pmatrix}
     1 & 0 & 0 & 0 \\
     0 & 1 & 0 & 0 \\
     0 & 0 & 1 & 0 \\
     0 & 0 & 1 & 0
     \end{pmatrix}
     \begin{pmatrix}
     x \\ y \\ z \\ 1
     \end{pmatrix}
     $$
   - **Dehomogenisierung**:
     - Nach der Projektion teilt man $x$ und $y$ durch $z$, um die finalen 2D-Koordinaten zu erhalten:
       $$
       (x, y, z) \mapsto \left(\frac{x}{z}, \frac{y}{z}\right)
       $$

3. **Anwendung**
   - Diese Technik wird in der Rasterisierung genutzt, um 3D-Objekte auf einer 2D-Anzeige darzustellen.

