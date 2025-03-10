## Grafik Pipeline
- Beschreibt den grundsätzlichen Ablauf, <mark style="background: #FFB86CA6;">wie ein 3D-Objekt</mark> (z.B. bestehend aus Dreiecken) <mark style="background: #FFB86CA6;">in ein fertiges 2D-Bild auf dem Bildschirm</mark> überführt wird.
![[Pasted image 20250126175454.png#invert|700]]
- **Applikation**  
	- Hier läuft dein Programmcode, der <mark style="background: #FFB86CA6;">Szenen aufbaut</mark>, Daten lädt, Transformationen berechnet usw.
- **Geometrieverarbeitung**  
    - In dieser Stufe werden u.a. die <mark style="background: #FFB86CA6;">Vertex-Shader</mark> (modernes OpenGL) ausgeführt. Außerdem finden <mark style="background: #FFB86CA6;">Koordinatentransformationen</mark> (Modell‐, Kamera‐, Projektions‐Transformation), <mark style="background: #FFB86CA6;">Beleuchtungsmodelle (im klassischen OpenGL per Fixed Function)</mark> und ggf. <mark style="background: #FFB86CA6;">Tesselierung</mark> statt.
- **Rasterisierung**  
    - <mark style="background: #FFB86CA6;">Aus deinen 3D-Polygonen werden Pixel- oder „Fragment“-Daten erzeugt</mark>, d.h. es wird festgelegt, <mark style="background: #FFB86CA6;">welche Pixel auf dem Bildschirm überhaupt zu den jeweiligen Dreiecken gehören</mark>.
- **Fragment-Operationen**  
	- In diesem Schritt laufen die <mark style="background: #FFB86CA6;">Fragment-Shader</mark> (modernes OpenGL) oder die <mark style="background: #FFB86CA6;">Fixed-Function-Per-Pixel-Berechnungen</mark> (klassisches OpenGL). Außerdem werden hier <mark style="background: #FFB86CA6;">Z-Tests</mark> (Tiefenpuffer) ausgeführt, <mark style="background: #FFB86CA6;">ggf. Blending (Transparenz), Stencil-Tests und andere Operationen</mark>, <mark style="background: #FFB86CA6;">bevor der final berechnete Farbwert in den Framebuffer geschrieben wird</mark>.
### Vorteile
- Viele Schritte können <mark style="background: #FFB86CA6;">parallelisiert</mark> werden
- <mark style="background: #FFB86CA6;">-> Effizientes Rendering</mark>

## OpenGL
- **OpenGL** ist eine plattformunabhängige, hardwareunabhängige und (weitgehend) sprachunabhängige **3D Rendering API**.
- OpenGL kümmert sich ausschließlich um das Rendering selbst, also die Grafik-Pipeline (Zeichnen von Primitiven, Texturierung, Shaderausführung usw.).
- Es **enthält jedoch kein eigenes Fenstermanagement** (Fenster erstellen, Eingabe‐Handling etc.). Dafür braucht man zusätzliche Bibliotheken wie [[GLUT]]/freeGLUT, GLFW, SDL, Qt, o.Ä.
### Immediate Mode vs. Retained Mode
- <mark style="background: #FFB86CA6;">Klassisches (älteres) OpenGL arbeitete viel im Immediate Mode</mark>:  
    <mark style="background: #FFB86CA6;">Jede Zeichenanweisung</mark> (`glBegin(GL_TRIANGLES)`, `glVertex3f(x, y, z)`, `glEnd()`) <mark style="background: #FFB86CA6;">wurde direkt in die Pipeline geschickt und sofort gerendert</mark>.
- Moderne APIs (bzw. andere Ansätze wie <mark style="background: #FFB86CA6;">Retained Mode</mark>) <mark style="background: #FFB86CA6;">verwalten Objekte in einer Szenengraph-Struktur, die man einmal aufbaut und dann komplett rendern lässt</mark>.
- In <mark style="background: #FFB86CA6;">modernem OpenGL benutzt man allerdings meist Vertex Buffer Objects (VBOs) und Vertex Array Objects (VAOs</mark>), also man legt Geometriedaten in der GPU ab und <mark style="background: #FFB86CA6;">ruft sie gesammelt auf</mark>. Trotzdem liegt die „Philosophie“ in OpenGL noch nahe am Immediate Mode, vergleicht man es z.B. mit einem echten Szenengraphen.
### Merkmale
- **OpenGL als Zustandsmaschine**:  
    <mark style="background: #FFB86CA6;">Fast alles in OpenGL wird als „State“ angesehen, den man „aktiviert“ oder „konfiguriert“</mark>. Beispielsweise Beleuchtung an/aus, Texturen an/aus, Shaderprogramme, usw.
- **Klassisches OpenGL (Fixed Function)**  
    – Hat eine <mark style="background: #FFB86CA6;">festverdrahtete Pipeline, in der man nur bestimmte Beleuchtungsmodelle (z.B. Blinn-Phong) und Interpolationen (z.B. Gouraud Shading) konfigurieren kann.</mark>
- **Modernes OpenGL (Programmable Pipeline)**  
    – Bietet <mark style="background: #FFB86CA6;">frei programmierbare Shader-Stufen</mark> (z.B. Vertex-, Tessellation-, Geometrie- und Fragment-Shader via GLSL). Damit bekommt man <mark style="background: #FFB86CA6;">deutlich mehr Flexibilität</mark> und kann beliebige Beleuchtungs- oder Shadingeffekte realisieren.
- **Client-Server-Konzept**  
    – Ursprünglich konnte man den „Server“ (Grafikkarte) auch über Netzwerk ansteuern (z.B. in professionellen Workstation-Umgebungen). Heute laufen <mark style="background: #FFB86CA6;">Client (Applikation) und Server (Grafiktreiber/GPU)</mark> meist auf demselben Rechner, aber intern bleibt der grundsätzliche Architekturgedanke bestehen.
- **Extensions**  
    – Hersteller (AMD, NVIDIA, Intel) können für ihre GPUs spezifische Funktionen über „Extensions“ bereitstellen, die über das normale OpenGL hinausgehen.
### Double-Buffering
- In OpenGL arbeiten wir oft mit **Double-Buffering**, <mark style="background: #FFB86CA6;">um Flackern zu vermeiden</mark>:
    - <mark style="background: #FFB86CA6;">Front Buffer: der aktuell sichtbare Puffer auf dem Bildschirm</mark>.
    - <mark style="background: #FFB86CA6;">Back Buffer: ein unsichtbarer Puffer, in den das nächste Bild gezeichnet wird</mark>.
- Wenn das Rendering fertig ist, tauscht (`glutSwapBuffers()`) <mark style="background: #FFB86CA6;">man beide Buffer (Front ↔ Back).</mark> <mark style="background: #FFB86CA6;">Dadurch erscheint das fertig gerenderte Bild sofort sichtbar, ohne dass wir Zwischenschritte sehen.</mark>
- Mit `GLUT_DEPTH` aktiviert man zusätzlich einen Tiefenpuffer (Z-Buffer), damit OpenGL die korrekte Ordnung der Objekte im Raum bestimmt (was vorne/hinten liegt).

## Koordinatensysteme
- **Objektkoordinaten**
    - Lokale Koordinaten eines <mark style="background: #FFB86CA6;">Objekts, relativ zu seinem Ursprung</mark>.
    - Transformation: **Modeltransformation** (z. B. `glTranslate`, `glRotate`, `glScale`), um das Objekt in die Welt zu platzieren.
- **Weltkoordinaten**
    - <mark style="background: #FFB86CA6;">Alle Objekte werden in einem gemeinsamen Weltkoordinatensystem positioniert</mark>.
    - Transformation: **Kameratransformation**, um eine virtuelle Kamera zu positionieren (ändert das Koordinatensystem, sodass die Kamera immer im Ursprung ist).
- **Kamerakoordinaten**
    - <mark style="background: #FFB86CA6;">Koordinaten aus Sicht der Kamera</mark>.
    - Transformation: **Projektionstransformation** (z. B. Perspektiv- oder Orthogonalprojektion), um die Szene in 2D zu bringen.
- **Clip-Koordinaten**
    - <mark style="background: #FFB86CA6;">Die Szene wird in den sichtbaren Bereich (Frustum) geclippt</mark>.
    - Perspektivische Division (Division durch www): Normiert die Koordinaten in den Bereich [-1, 1].
- **Normalized Device Coordinates (NDC)**
    - <mark style="background: #FFB86CA6;">Koordinaten in einem standardisierten Würfel</mark>.
    - Transformation: **Viewport Transformation**, um die Koordinaten in Bildschirmkoordinaten zu konvertieren.
- **Fensterkoordinaten**
    - <mark style="background: #FFB86CA6;">Endgültige Pixelkoordinaten für die Darstellung im Fenster</mark>.
![[Pasted image 20250126180438.png#invert|300]]
### Transformationen
OpenGL verwendet <mark style="background: #FFB86CA6;">Matrix-Stacks, um Transformationen zu speichern</mark>:
- **Modelview-Matrix**: Kombination aus Modell- und Kameratransformationen.
- **Projektion-Matrix**: Projektionstransformation.
- **Textur-Matrix**: (Seltener genutzt) für Texturkoordinatentransformationen.
## **Geometrische Primitive**
OpenGL arbeitet mit geometrischen Grundformen, sogenannten **Primitiven**:
- **GL_POINTS**: Einzelne Punkte.
- **GL_LINES**: Einzelne Linien zwischen zwei Punkten.
- **GL_TRIANGLES**: Einzelne Dreiecke (effizient für Rasterisierung).
- **GL_QUADS**: <mark style="background: #FFB86CA6;">Vierecke</mark> (veraltet, oft in Dreiecke zerlegt).
- **GL_TRIANGLE_STRIP / GL_TRIANGLE_FAN**: <mark style="background: #FFB86CA6;">Optimierte Dreiecksketten für schnellere Verarbeitung</mark>.
![[Pasted image 20250126180601.png#invert|500]]
### Beispiel
```OpenGL
glBegin(GL_TRIANGLES);
glVertex3f(0.0f, 0.0f, 0.0f);
glVertex3f(1.0f, 0.0f, 0.0f);
glVertex3f(0.0f, 1.0f, 0.0f);
glEnd();

```
### **Dreiecke, Polygone und typische Probleme**
#### **T-Vertices**
- Entstehen, <mark style="background: #FFB86CA6;">wenn sich Dreiecke an einer gemeinsamen Kante treffen, aber nicht sauber verbunden sind</mark>.
- Vermeidung:
    - <mark style="background: #FFB86CA6;">Kanten hinzufügen, um Lücken zu schließen</mark>.
    - <mark style="background: #FFB86CA6;">Geometrie so modellieren, dass keine „freien“ Punkte übrig bleiben</mark>.
#### **Z-Fighting**
- <mark style="background: #FFB86CA6;">Flackern, wenn sich Polygone auf derselben Ebene überlagern</mark>.
- Ursachen:
    - <mark style="background: #FFB86CA6;">Rundungsfehler bei der Tiefenberechnung</mark>.
- Lösung:
    - `glPolygonOffset(factor, units)` verwenden, um eine <mark style="background: #FFB86CA6;">leichte Verschiebung</mark> zu erzeugen.
    - Beispiel: Texturen oder Overlays auf andere Geometrie legen.
## **Backface Culling**
- **Definition**: Backface Culling ist ein Optimierungsprozess, bei dem <mark style="background: #FFB86CA6;">Flächen, die von der Kamera wegzeigen, verworfen werden</mark>.
- **Vorder- und Rückseiten von Dreiecken**:
    - Standard: Vorderseite = Vertices gegen den Uhrzeigersinn (**GL_CCW**).
    - Rückseite = Vertices im Uhrzeigersinn (**GL_CW**).
    - Einstellung mit `glFrontFace(GL_CCW)` oder `glFrontFace(GL_CW)`.
- **Aktivierung von Backface Culling**:
    - `glEnable(GL_CULL_FACE)` aktiviert den Mechanismus.
    - Mit `glCullFace(GL_BACK)` wird die Rückseite ignoriert.
- **Vorteil**:
    - <mark style="background: #FFB86CA6;">Reduktion des Rasterisierungsaufwands bei geschlossenen Objekten, da Rückseiten ohnehin nicht sichtbar sind</mark>.
## Beleuchtung und Schattierung
- **Farbdefinition**:
    - Direkt pro Vertex als Attribut: `glColor*()` (z. B. `glColor3f(r, g, b)`).
    - Oder durch Materialeigenschaften und OpenGL-Beleuchtungsmodell: `glMaterial*()`.
- **Aktivierung der Beleuchtung**:
    - `glEnable(GL_LIGHTING)` aktiviert die Berechnung.
- **Schattierung**:
    - **Flat Shading**: Ein einheitlicher Farbwert pro Dreieck.
    - **Smooth Shading (Gouraud)**: Interpolierte Farben zwischen Vertices.
- **Kombination mit Texturen**:
    - Texturen und Beleuchtung kombinieren mit `glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)`.
### **Beleuchtungsberechnung**
- **Ort der Berechnung**:
    - <mark style="background: #FFB86CA6;">Beleuchtung wird in den Kamerakoordinaten berechnet, nach der Modelview-Transformation</mark>.
- **Shading-Typen**:
    - OpenGL unterstützt pro Vertex (<mark style="background: #FFB86CA6;">Gouraud</mark>) oder pro Dreieck (Flat).
    - <mark style="background: #FFB86CA6;">Keine Berechnung pro Pixel (z. B. Phong Shading).</mark>
- **Lichtquellenmodell**:
    - <mark style="background: #FFB86CA6;">Diffuse, spekulare und Umgebungsbeleuchtung</mark>.
    - Materialfarben + globale Beleuchtung werden kombiniert.
    - Transformationen (z. B. Normale, Vertexposition) durch die Modelview-Matrix.
### **4. Blinn-Phong-Beleuchtungsmodell**
- <mark style="background: #FFB86CA6;">Erweiterung des klassischen Phong-Modells</mark>:
  - Phong: $I_s = k_s \cdot I_L \cdot (\mathbf{R_L} \cdot \mathbf{V})^n$.
  - Blinn-Phong <mark style="background: #FFB86CA6;">ersetzt den Reflexionsvektor</mark> $\mathbf{R_L}$ durch den <mark style="background: #FFB86CA6;">Halfway-Vektor</mark> $\mathbf{H} = \frac{\mathbf{V} + \mathbf{L}}{||\mathbf{V} + \mathbf{L}||}$.
- **Vorteil von Blinn-Phong**:
  - <mark style="background: #FFB86CA6;">Effizienter, da H$ konstant bleibt, wenn Licht oder Kamera sehr weit entfernt ist</mark>.
  - Spekularer Anteil: $I_s = k_s \cdot I_L \cdot (\mathbf{N} \cdot \mathbf{H})^{n'}$, mit $n' \approx 4n$.
## Modernes OpenGL
- **Hardware-Fortschritte**:
    - GPUs mit sehr hoher Rechenleistung (mehrere TFLOPS).
    - Dedizierter, schneller GPU-Speicher (>900 GB/s).
- **Probleme des klassischen OpenGL**:
    - Zu <mark style="background: #FFB86CA6;">hoher Overhead durch Immediate Mode</mark> (`glBegin`, `glVertex`, `glEnd`).
    - Daten müssen effizienter in Puffer geschrieben werden (z. B. VBOs).
- **Ziele des modernen OpenGL**:
    - <mark style="background: #FFB86CA6;">Keine Fixed-Function-Verarbeitung</mark> mehr.
    - <mark style="background: #FFB86CA6;">Nutzung von programmierbaren Shadern</mark>.
    - **Kompatibilitätsprofil**: Fixed-Function-API bleibt nutzbar.
    - **Core-Profil**: Nur moderne Funktionen ohne Fixed Function.
### Funktionalität
- **Beliebige Beleuchtung und Materialien**:
    - <mark style="background: #FFB86CA6;">Shader ermöglichen flexiblere Modelle</mark>.
    - <mark style="background: #FFB86CA6;">Nicht mehr gebunden an</mark> `glMaterial` oder `glLight`.
- **Prozedurale Texturen**:
    - <mark style="background: #FFB86CA6;">Texturen werden dynamisch direkt auf der GPU berechnet</mark>.
- **Komplexe Datenstrukturen**:
    - Z. B. Octrees für Texture Mapping.
- **Geometrieverarbeitung und Animation**:
    - Dynamische Veränderungen von Geometrie (z. B. Skinning für Animationen).
### Moderne Grafik-Pipeline
![[Pasted image 20250127121422.png#invert|700]]
![[Pasted image 20250126181252.png#invert|700]]
1. **Vertex Shader**:
    - <mark style="background: #FFB86CA6;">Transformiert einzelne Vertices in den Clip-Raum</mark>.
    - Kann Attributdaten interpolieren (z. B. Farben, Normalen).
    - **Eingabe**: <mark style="background: #FF5582A6;">Ein Vertex</mark> und dessen Attribute + Transformationsmatritzen
    - **Ausgabe**: Transformierte Position und weitergeleitete Attribute.
    - <mark style="background: #FFB86CA6;">Beispiel: Berechnung von Beleuchtung pro Vertex (Gouraud Shading).</mark>
2. **Tessellation** (optional):
    - <mark style="background: #FFB86CA6;">Unterteilt Primitives (z. B. Dreiecke) in kleinere Einheiten, um detailliertere Geometrie zu erstellen</mark>.
    - **Tessellation Control Shader**: Definiert die Unterteilung.
    - **Tessellation Evaluation Shader**: Berechnet die neuen Vertices.
    - Eingabe-Typ: `GL_PATCH`.
3. **Geometry Shader** (optional):
    - <mark style="background: #FFB86CA6;">Verarbeitet</mark> <mark style="background: #FF5582A6;">ein</mark> <mark style="background: #FFB86CA6;">vollständiges Primitive</mark> (z. B. ein Dreieck).
    - <mark style="background: #FFB86CA6;">Kann neue Primitives erzeugen, bestehende entfernen oder umwandeln</mark>.
    - Beispiel: <mark style="background: #FFB86CA6;">Erzeugen von Schattenvolumen</mark> oder Instanzierung.
4. **Primitive Assembly & Vertex Post-Processing**:
    - <mark style="background: #ADCCFFA6;">nicht-programmierbare Stufe („fixed-function“)</mark>
    - <mark style="background: #FFB86CA6;">Setzt Vertices zu Primitives zusammen</mark> (z. B. Dreiecke).
    - **Clipping**: <mark style="background: #FFB86CA6;">Entfernt Teile, die außerhalb des sichtbaren Bereichs liegen</mark>.
    - <mark style="background: #FFB86CA6;">Backface Culling</mark>
    - Perspektivische Division: <mark style="background: #FFB86CA6;">Überführt Clip-Koordinaten in Normalized Device Coordinates (NDC).</mark>
5. **Rasterization**:
    - <mark style="background: #ADCCFFA6;">nicht-programmierbare Stufe („fixed-function“)</mark>
    - <mark style="background: #FFB86CA6;">Wandelt Primitives in Fragmente (Pixel) um.</mark>
    - Interpoliert Attribute (z. B. Farben, Texturkoordinaten).
    - **Eingabe**: Primitives in NDC
    - **Ausgabe**: Fragmente mit 2D-Koordinaten.
6. **Fragment Shader**:
    - <mark style="background: #FFB86CA6;">Berechnet die endgültige Farbe jedes Fragments</mark>.
    - Kann <mark style="background: #FFB86CA6;">Texturen</mark> verwenden oder <mark style="background: #FFB86CA6;">Beleuchtungsberechnungen</mark> durchführen.
    - **Eingabe**: Fragmente mit interpolierten Attributen.
    - **Ausgabe**: Farbe, Opazität und Tiefe.
7. **Fragment-Operationen**:
    - <mark style="background: #ADCCFFA6;">nicht-programmierbare Stufe („fixed-function“)</mark>
    - <mark style="background: #FFB86CA6;">Führt Tests durch (z. B. Z-Buffer, Stencil-Test).</mark>
    - <mark style="background: #FFB86CA6;">Kombiniert Fragments mit vorhandenen Daten im Framebuffer</mark>.
    - <mark style="background: #FFB86CA6;">Ergebnis wird in den Framebuffer geschrieben</mark>.
### **Transform Feedback**
- Ermöglicht es, die Ausgabe transformierter Vertices direkt in einen Buffer zu schreiben.
- Wird genutzt, um Geometrie-Daten zwischen Shader-Stufen weiterzuverarbeiten.
### **Framebuffer Objects (FBOs)**
- <mark style="background: #FFB86CA6;">Speichern Zwischenergebnisse</mark> (z. B. die Ausgabe des Fragment Shaders).
- Verwendung:
    - **Post-Processing-Effekte**: Filter, Farbkorrektur.
    - **Deferred Shading**: Beleuchtungsberechnungen in mehreren Durchgängen.
    - **Order-Independent Transparency**: Transparente Objekte korrekt darstellen.
- <mark style="background: #FFB86CA6;">Shader können lesend und schreibend auf FBOs zugreifen</mark>.
### **Vorteile moderner Grafik-Pipelines**
- **Massive Parallelität**:
    - GPUs sind spezialisiert auf <mark style="background: #FFB86CA6;">parallele Verarbeitung</mark>, wodurch Millionen von Vertices und Pixeln gleichzeitig verarbeitet werden.
- **Flexibilität**:
    - <mark style="background: #FFB86CA6;">Programmierbare Shader ersetzen fixe Funktionen</mark>, was kreative Effekte ermöglicht.
- **Effizienz**:
    - <mark style="background: #FFB86CA6;">Pipeline-Verarbeitung sorgt für einen hohen Durchsatz</mark>.
## Datenstrukturen für Dreiecks- und Polygonnetze
#### **Shared Vertex-Datenstruktur**
![[Pasted image 20250126182550.png#invert|200]]
- **Beschreibung**:
    - Enthält eine <mark style="background: #FFB86CA6;">Liste von Vertices (Punkte im Raum, z. B. A, B, C, D, E, F).</mark>
    - <mark style="background: #FFB86CA6;">Zusätzliche Indizes definieren die Zuordnung von Vertices zu Dreiecken</mark> oder Polygonen (z. B. {A, B, C}, {C, E, D}).
- **Vorteile**:
    - <mark style="background: #FFB86CA6;">Effiziente Speicherung</mark>, da jeder Vertex nur einmal gespeichert wird.
    - <mark style="background: #FFB86CA6;">Einfach zu implementieren</mark>.
- **Nachteile**:
    - <mark style="background: #FFB86CA6;">Abfragen, wie „Welche Dreiecke grenzen an ein bestimmtes Dreieck?“</mark> sind aufwendig, da die Struktur keine direkten Verweise enthält.
#### **Half-Edge-Datenstruktur**
![[Pasted image 20250126182607.png#invert|200]]
- **Beschreibung**:
    - <mark style="background: #FFB86CA6;">Speichert für jede Kante (Edge) eines Polygons eine sogenannte Half-Edge</mark>.
    - Jede Half-Edge enthält:
        - <mark style="background: #FFB86CA6;">Geschwisterkante: Die gegenüberliegende Half-Edge</mark> (gleiche Kante, andere Richtung).
        - <mark style="background: #FFB86CA6;">End-Vertex: Der Vertex, zu dem die Kante zeigt</mark>.
        - <mark style="background: #FFB86CA6;">Anliegendes Polygon</mark>: Die Fläche, die die Kante begrenzt.
        - <mark style="background: #FFB86CA6;">Nächste Kante entlang des Kantenzugs</mark>: Führt zur nächsten Kante des gleichen Polygons.
    - Pro Fläche wird ein Verweis auf eine ihrer Kanten gespeichert.
- **Vorteile**:
    - <mark style="background: #FFB86CA6;">Effiziente Navigation</mark> zwischen benachbarten Kanten, Vertices und Flächen.
    - Optimiert für Algorithmen, die topologische Informationen benötigen.
- **Anwendungsbeispiele**:
    - Überprüfung benachbarter Dreiecke.
    - Selbstüberschneidung und Überlappung von Polygonen.
## Shader in OpenGL
#### **Allgemeines**
- **Shader-Typen**:
    - **Vertex Shader**: Transformation und Beleuchtung von Vertices.
    - **Tessellation Shader**: Unterteilung von Geometrie in kleinere Segmente.
    - **Geometry Shader**: Verarbeitung kompletter Primitives, z. B. Erzeugen zusätzlicher Geometrie.
    - **Fragment Shader**: Berechnung der Pixel-Farbe.
    - **Compute Shader**: Allgemeine Berechnungen, unabhängig von der Pipeline.
- **GLSL (OpenGL Shading Language)**:
    - Definiert Operationen und Funktionen.
    - Unterstützt unterschiedliche Eingabe-/Ausgabeformate je nach Shader-Typ.
#### **Effiziente Nutzung**
- Aufgabenverteilung:
    - Shader sind für spezielle Aufgaben gedacht, aber einige Aufgaben können auf mehreren Stufen durchgeführt werden (z. B. Beleuchtung).
- **Performance-Optimierungen**:
    - Verlagerung von Aufgaben zwischen Stufen, um Flaschenhälse zu vermeiden.
    - Beispiel: Teilweise Beleuchtung im Vertex Shader, um die Last des Fragment Shaders zu reduzieren.
- **Kombination von Shadern**:
    - Komplexe Animationen können über Transform Feedback zwischen Shadern oder durch Compute Shader direkt realisiert werden.
## Rasterisierung und Antialiasing
- **Rasterisierung**: Der Prozess, bei dem <mark style="background: #FFB86CA6;">Primitive (wie Dreiecke) in Pixel oder Fragmente umgewandelt werden</mark>, die auf dem Bildschirm dargestellt werden können.
- **Supersampling (FSAA)**: Mehrere <mark style="background: #FFB86CA6;">Samples pro Pixel</mark> werden berechnet, um Kanten weicher darzustellen. Beispiele:
    - <mark style="background: #FFB86CA6;">Einfaches Supersampling: Samples sind gleichmäßig verteilt</mark>.
    - Rotated Grid Supersampling: Gitter ist gedreht für bessere Qualität.
    - Quincunx (NVIDIA): Kombiniert mehrere Samples mit unterschiedlichen Gewichten.
- **Multisample Antialiasing (MSAA)**: <mark style="background: #FFB86CA6;">Reduziert Rechenaufwand, indem nur Tiefeninformationen supersampled werden, nicht Farben</mark>.
- **Coverage Sampling AA (CSAA)**: <mark style="background: #FFB86CA6;">Optimierte Version von MSAA, bei der mehr Sichtbarkeits-Samples pro Pixel gespeichert werden</mark>.
## Fragment-Verarbeitung
- **Pipeline-Stufen**: <mark style="background: #FFB86CA6;">Fragmente durchlaufen verschiedene Tests und Operationen, bevor sie in den Framebuffer geschrieben werden</mark>:
    - <mark style="background: #FFB86CA6;">Ownership Test: Prüft, ob das Fragment zur aktuellen Oberfläche gehört</mark>.
    - <mark style="background: #FFB86CA6;">Scissor Test: Schneidet Fragmente außerhalb eines Rechtecks aus.</mark>
    - <mark style="background: #FFB86CA6;">Stencil Test</mark>: Verwendet eine Maske, um <mark style="background: #FFB86CA6;">Fragmente basierend auf vorher festgelegten Regeln zu akzeptieren oder abzulehnen</mark>.
    - <mark style="background: #FFB86CA6;">Depth Buffer Test: Sorgt dafür, dass Fragmente (Pixel) nur dann gezeichnet werden, wenn sie näher an der Kamera liegen als die bereits im Tiefenpuffer gespeicherten Werte</mark>
    - <mark style="background: #FFB86CA6;">Blending: Kombiniert Fragmentfarben mit bestehenden Farben im Framebuffer</mark>.
- Zusätzliche Operationen wie **sRGB-Konvertierung**, **Dithering**, und **Logische Operationen** können folgen.
## **Alpha-Test**
- In <mark style="background: #FFB86CA6;">klassischem OpenGL werden Fragmente basierend auf ihrem Alpha-Wert (Transparenz) verworfen</mark>.
- Beispiel: Nur Fragmente mit $\alpha = 1.0$ werden gezeichnet.
- **Modernes OpenGL**: <mark style="background: #FFB86CA6;">Alpha-Test existiert nicht mehr direkt; stattdessen wird im Fragment-Shader</mark> ein `discard` verwendet.
### **OpenGL Blending**
- Kombiniert Farbwerte des Fragments ("Source") mit den Farben bereits existierender Fragmente ("Destination") im Framebuffer.
- Formel: $\text{Output} = (\text{Source Factor} \cdot \text{Source Color}) \, \text{Op} \, (\text{Dest Factor} \cdot \text{Dest Color})$
![[Pasted image 20250126183035.png|500]]
- Beispiel für Transparenz: Kugel (Source) wird mit Hintergrund (Destination) gemischt.
- **Semitransparenz**: Erfordert Tiefensortierung, da die Reihenfolge der Mischung entscheidend ist.
### **Stencil Buffer**
- Ein zusätzlicher Speicherbereich, der <mark style="background: #FFB86CA6;">Werte für jedes Pixel speichert</mark>. 
	- <mark style="background: #FFB86CA6;">Basierend darauf werden dann die Masken bestimmt zur Auswahl oder nicht-Auswahl der Fragmente</mark>
	- Kann durch Stencil-Operationen verändert werden:
		- Keep, Zero, Replace, Increment/Decrement, Invert
- Anwendungen:
    - **Maskierung**: Teile eines Bildes ausschneiden oder hervorheben.
    - **Schattenvolumen**: Projektionen für realistische Schatten berechnen.
    - **Stencil-Routing**: Steuert, welche Fragmente gezeichnet werden.
![[Pasted image 20250126183206.png#invert|700]]
1. **Schattenvolumen (Crow, SIGGRAPH '77)**:
    - Schattenvolumen sind eine <mark style="background: #FFB86CA6;">Methode zur Berechnung von Schatten</mark> in der Computergrafik.
    - Der **Stencil Buffer** wird verwendet, um die Schnitte der Schattenvolumen zu zählen:
        1. **Zeichnen der Objekte:** Zuerst werden die sichtbaren Flächen im Z-Buffer gespeichert.
        2. **Deaktivierung von Farb- und Tiefenpuffern:** Damit beeinflussen die Schattenvolumen diese Puffer nicht.
        3. **Zeichnen der Schattenvolumen:**
            - **Vorderseiten** des Volumens erhöhen den Stencil-Wert.
            - **Rückseiten** verringern den Stencil-Wert.
        4. **Erkennung von Schatten:** Überall dort, wo der Stencil-Wert > 0 ist, befindet sich Schatten.
    - **Backface Culling:** Dient dazu, nur Vorder- oder Rückseiten der Volumen zu betrachten.
**Schattenvolumen eines Dreiecksnetzes**:
- Der Prozess:
    1. Bestimme die Schattenvolumen für jedes Objekt und jede Lichtquelle. Dies kann mithilfe eines Geometry Shaders und `GL_TRIANGLES_ADJACENCY` erfolgen.
    2. Identifiziere Oberflächenpunkte, die in den Schattenvolumen liegen.
- **Visualisierung:** Gelbe Kanten zeigen die Silhouetten bezüglich der Lichtquellen.
1. **Fragment-Verarbeitung: Scissor-Test**:
    - **Funktion:** Entfernt Fragmente außerhalb eines rechteckigen Bereichs.
    - **Zweck:** Wird verwendet, um kleinere Fensterbereiche zu aktualisieren (z. B. beim Löschen).
    - **Vergleich:** Ähnlich wie eine rechteckige Stencil-Maske.
2. **Fragment-Verarbeitung: Dithering (historisch)**:
    - **Funktion:** Dithering war notwendig, um Farbfehler zu minimieren, als Hardware noch keine TrueColor-Unterstützung hatte.
    - Beispiel: Pixel werden "gestreut", um eine glattere Darstellung zu erreichen.
5. **Fragment-Verarbeitung: Logical Ops**:
    - **Funktion:** Führt bitweise Operationen auf den Farbwerten des Fragments und des Framebuffers durch.
    - Beispiele:
        - `glEnable(GL_LOGIC_OP);`
        - `glLogicOp(GL_XOR);`
    - **Wichtig:** Logical Ops verhindern das Blending, auch wenn dieses aktiviert ist.
3. **Fragment-Verarbeitung: Occlusion Query**:
    - **Funktion:** Zählt Fragmente, die sowohl den Stencil- als auch den Tiefentest bestehen. Dies verändert den Framebuffer nicht direkt.
    - **Anwendung:** **Occlusion Culling**:
        - Überprüft, ob ein Objekt vollständig verdeckt ist.
        - Falls ja, wird es nicht gezeichnet.

### **Framebuffer Objects (FBOs)**

- **Bisheriger Ansatz**: Beim Rendering werden Fragmente direkt in OpenGL-Framebuffer (z. B. Farbpuffer, Tiefenpuffer) geschrieben.
- **FBOs**:
    - Erlauben die Ausgabe in einen „Off-Screen Buffer“, sodass die Szene nicht direkt auf dem Bildschirm gerendert wird.
    - **Color Buffers**: Eine oder mehrere Texturen, um Farbinformationen zu speichern.
    - **Render Buffers**: Speichern Tiefen- und Stencil-Informationen.
- **Vorteile**: FBOs werden für moderne Rendering-Techniken verwendet, wie z. B. Blooming (Lichteffekte) oder Simulation von OpenGL-Accumulation Buffers für Nachbeleuchtung.

### **Shadow Mapping (Williams, SIGGRAPH '78)**

- **Grundidee**:
    - Eine **Shadow Map** ist eine Textur, die pro Lichtquelle erzeugt wird und die Tiefeninformationen der Szene aus Sicht der Lichtquelle speichert.
    - Ziel ist es, die Positionen der Objekte zu analysieren und zu entscheiden, ob ein Punkt im Schatten liegt.
- **Schritte**:
    
    1. **Tiefenpuffer aus Sicht der Lichtquelle speichern**.
    2. Beim Rendering der Szene wird geprüft, ob ein Punkt weiter entfernt ist, als es in der Shadow Map gespeichert wurde (→ Schatten).
    - ![[Pasted image 20250126183721.png|500]]
    - ![[Pasted image 20250126183733.png|500]]**Blauer Punkt**: Nicht im Schatten, da die tatsächliche Entfernung zur Lichtquelle der in der Shadow Map entspricht.
    - **Roter Punkt**: Im Schatten, da die tatsächliche Entfernung größer ist.

### **Raytracing API**
- **Überblick**:
    - GPUs unterstützen Raytracing, indem sie Strahlen durch die Szene verfolgen, um Schnitte mit Objekten zu berechnen.
    - Dies ist effizienter als das klassische Rasterizing.
- **Beschleunigungsstrukturen**:
    - **Bottom-Level AS**: Enthält Primitive (z. B. Dreiecke) und wird von der GPU genutzt.
    - **Top-Level AS**: Verknüpft Transformationen, Bottom-Level-AS und Shader-Informationen.
    - Vorteil: Daten können inkrementell aktualisiert werden.
- **Shader**:
    - **Ray Generation Shader**: Startet die Berechnung eines Strahls.
    - **Intersection Shader**: Bestimmt, ob ein Strahl eine Primitive schneidet.
    - **Any Hit Shader**: Kann frühe Abbrüche auslösen (z. B. bei transparenten Oberflächen).
    - **Closest Hit Shader**: Wird aufgerufen, wenn der nächste Schnittpunkt gefunden ist.
    - **Miss Shader**: Wird verwendet, wenn kein Schnitt gefunden wird.