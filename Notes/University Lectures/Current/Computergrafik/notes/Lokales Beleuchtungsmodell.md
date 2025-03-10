## BRDF (Bidirectional Reflectance Distribution Function)
Die BRDF <mark style="background: #FFB86CA6;">beschreibt, wie Licht von einer Oberfläche reflektiert wird</mark>. Sie gibt das <mark style="background: #FFB86CA6;">Verhältnis zwischen der Lichtmenge, die in eine bestimmte Richtung auf eine Oberfläche trifft, und der Lichtmenge, die in eine andere Richtung von dieser Oberfläche reflektiert</mark> wird.
- **Einfallsrichtung (`l`)**: Die <mark style="background: #FFB86CA6;">Richtung, aus der das Licht auf die Oberfläche trifft</mark>.
- **Betrachterrichtung (`v`)**: Die Richtung, in der das reflektierte Licht zum Betrachter gelangt.
- **Normale (`n`)**: Der Vektor, der senkrecht zur Oberfläche steht und die Orientierung der Oberfläche angibt.
![[Pasted image 20241108095704.png#invert|400]]

### Funktionsweise
- Die <mark style="background: #FFB86CA6;">BRDF ist eine Funktion</mark> $f_r(\mathbf{l}, \mathbf{x}, \mathbf{v})$, die <mark style="background: #FFB86CA6;">beschreibt, wie Licht in verschiedene Richtungen reflektiert wird</mark>.
- Sie verwendet ein **Referenzkoordinatensystem** mit den Vektoren **Tangente (`t`)**, **Bitangente (`b`)**, und **Normale (`n`)**. Diese Vektoren spannen ein Koordinatensystem auf, um die Orientierung der Oberfläche festzulegen.
- Die BRDF hängt von Winkeln ab:
  - **Polarwinkel** $\theta$: Der Winkel zwischen Einfallsrichtung und Normale sowie zwischen Beobachtungsrichtung und Normale.
  - **Azimutalwinkel** $\phi$: Der Winkel in der horizontalen Ebene zwischen den Einfalls- und Beobachtungsrichtungen.
![[Pasted image 20241108095815.png#invert|400]]
### Radiometrische Größen
Die BRDF basiert auf radiometrischen Größen:
- **Radiance (Strahldichte)**: <mark style="background: #FFB86CA6;">Die Menge an Licht, die in Richtung des Betrachters</mark> (`v`) <mark style="background: #FFB86CA6;">reflektiert wird</mark>, gemessen in $\text{W}/\text{m}^2\text{sr}$.
- **Irradiance (Flussdichte)**: <mark style="background: #FFB86CA6;">Die Menge an Licht, die auf die Oberfläche in Richtung</mark> `l` <mark style="background: #FFB86CA6;">trifft</mark>, gemessen in $\text{W}/\text{m}^2$.
- Die BRDF kann als das <mark style="background: #FFB86CA6;">Verhältnis der Strahldichte zur einfallenden Flussdichte betrachtet</mark> werden: Sie beschreibt, wie viel Licht in eine bestimmte Richtung reflektiert wird.
### Anwendung: Anisotrope und isotrope BRDF
- **Anisotrope BRDF**: Die Reflexion hängt von der Orientierung der Oberfläche ab. Dies tritt auf, <mark style="background: #FFB86CA6;">wenn die Oberfläche eine Mikrostruktur aufweist</mark>, wie z. B. gebürstetes Metall, das das <mark style="background: #FFB86CA6;">Licht in bestimmten Richtungen stärker reflektiert</mark>.
- **Isotrope BRDF**: Die Reflexion ist rotationsinvariant um die Normale; die <mark style="background: #FFB86CA6;">Reflexionseigenschaften ändern sich nicht bei Drehung</mark> der Oberfläche. Dies ist <mark style="background: #FFB86CA6;">typisch für glatte, gleichmäßig strukturierte Oberflächen</mark>.
### Verallgemeinerung: BRDF und BTDF
- **BTDF (Bidirectional Transmission Distribution Function)** <mark style="background: #FFB86CA6;">beschreibt die Lichtübertragung durch das Material</mark> (Transparenz).
- Die **BSDF (Bidirectional Scattering Distribution Function)** <mark style="background: #FFB86CA6;">kombiniert BRDF und BTDF und beschreibt sowohl die Reflexion als auch die Transmission von Licht</mark>.
### Messung von BRDFs
- Ein **Gonioreflektometer** ist ein Messinstrument, das die Reflektionseigenschaften realer Materialproben misst. Dabei wird das Reflexionsverhalten bei verschiedenen Einfalls- und Blickwinkeln analysiert.
- Die **Messung realer Materialproben** ist jedoch sehr aufwändig und zeitintensiv. Zudem können die Daten durch Rauschen oder andere Unregelmäßigkeiten verfälscht sein.
## Phong-Beleuchtungsmodell
Das Phong-Beleuchtungsmodell ist ein phänomenologisches Modell, das die <mark style="background: #FFB86CA6;">Reflexion von Licht auf einer Oberfläche mit drei Hauptkomponenten beschreibt</mark>:
- <mark style="background: #ADCCFFA6;">Ambient</mark> (Umgebungslicht): <mark style="background: #FFB86CA6;">Modelliert die indirekte Beleuchtung durch Licht, das von anderen Oberflächen reflektiert wird</mark>. Diese Komponente sorgt dafür, dass die gesamte Szene eine Grundhelligkeit hat.
- <mark style="background: #ADCCFFA6;">Diffuse Reflexion</mark>: Nach dem Lambert’schen Gesetz reflektiert eine Oberfläche <mark style="background: #FFB86CA6;">Licht gleichmäßig in alle Richtungen</mark>, unabhängig vom Betrachtungswinkel. Diese Reflexion ist abhängig vom Winkel zwischen der Lichtrichtung und der Normalen der Oberfläche.
- <mark style="background: #ADCCFFA6;">Spekulare Reflexion</mark>: Modelliert Glanzlichter, die entstehen, wenn <mark style="background: #FFB86CA6;">Licht in einer bevorzugten Richtung reflektiert wird</mark>, wie bei einer Spiegelung. Diese Reflexion erzeugt Lichtpunkte und Glanz auf der Oberfläche.
![[Pasted image 20241108100130.png|400]]
### Diffuse Reflexion (Lambert’sche Reflexion)
- Die <mark style="background: #FFB86CA6;">Lambert’sche Reflexion tritt auf, wenn Licht gleichmäßig in alle Richtungen reflektiert wird</mark>.
- Die Intensität der diffusen Reflexion wird berechnet als $k_d \cdot I_L \cdot \cos(\theta)$, wobei:
  - $k_d$ der Materialkoeffizient ist (beschreibt die Farbe und Reflexionsstärke des Materials).
  - $I_L$ die <mark style="background: #FFB86CA6;">Lichtintensität</mark>.
  - $\cos(\theta)$ das <mark style="background: #FFB86CA6;">Verhältnis zwischen der Lichtrichtung</mark> `l` <mark style="background: #FFB86CA6;">und der Normalen</mark> `n` der Oberfläche, berechnet als $\cos(\theta) = \mathbf{n} \cdot \mathbf{l}$ (Skalarprodukt).
![[Pasted image 20241108100156.png#invert|400]]
### Spekulare Reflexion (Glanzlichter)
- Bei der **spekularen Reflexion** wird Licht <mark style="background: #FFB86CA6;">bevorzugt in einer bestimmten Richtung reflektiert</mark>, die von der **Reflexionsrichtung** $\mathbf{r}_1$ abhängt.
- **Perfekte Spiegelung**: Der <mark style="background: #FFB86CA6;">Reflexionsvektor</mark> $\mathbf{r}_1$ wird durch die Formel berechnet:
  $$
  \mathbf{r}_1 = 2(\mathbf{l} \cdot \mathbf{n}) \mathbf{n} - \mathbf{l}
  $$
  Dabei ist `l` die Lichtrichtung, `n` die Normale der Oberfläche. Dieser Vektor beschreibt die Richtung, in die das Licht bei einer idealen Spiegelung reflektiert wird.
![[Pasted image 20241108100217.png#invert|400]]
### Phong-Exponent und Abnahme der spekularen Reflexion
- **Imperfekte Spiegelung**: <mark style="background: #FFB86CA6;">Nicht alle Oberflächen sind perfekte Spiegel; sie reflektieren Licht in einem Bereich um </mark>$\mathbf{r}_1$ herum.
- <mark style="background: #FFB86CA6;">Der Phong-Exponent n steuert, wie stark das Glanzlicht fokussiert ist:</mark>
  - **Große Werte von $n$** führen zu <mark style="background: #FFB86CA6;">kleinen, scharf begrenzten Glanzlichtern</mark> (z. B. bei glänzenden Oberflächen).
  - **Kleine Werte von $n$** führen zu <mark style="background: #FFB86CA6;">breiteren, weniger konzentrierten Glanzlichtern</mark> (z. B. bei matten Oberflächen).
- Die spekulare Reflexion wird berechnet als:
  $$
  I_s = k_s \cdot I_L \cdot \cos^n(\alpha) = k_s \cdot I_L \cdot (\mathbf{r}_1 \cdot \mathbf{v})^n
  $$
  - Hier ist $k_s$ der spekulare Reflexionskoeffizient, $I_L$ die Lichtintensität, und $\alpha$ der Winkel zwischen der Reflexionsrichtung $\mathbf{r}_1$ und der Blickrichtung $\mathbf{v}$.
  - **Hinweis**: Der Ausdruck $(\mathbf{r}_1 \cdot \mathbf{v})$ kann als Maß dafür betrachtet werden, wie gut die Reflexion mit der Blickrichtung übereinstimmt. Ein hoher Wert bedeutet ein starkes Glanzlicht in Blickrichtung.

## Phong-Shading
- <mark style="background: #FFB86CA6;">Beleuchtungsberechnung mit interpolierten Normalen nennt man Phong Shading</mark>

## Gouraud Shading
- <mark style="background: #FFB86CA6;">OpenGL unterstützt</mark> nativ keine Beleuchtungsberechnung pro Pixel, also kein Phong Shading, <mark style="background: #FFB86CA6;">nur Gouraud Shading</mark>
- Die <mark style="background: #FFB86CA6;">Beleuchtung wird nur an den Eckpunkten (Vertex-Normalen) eines Dreiecks berechnet</mark> wird. 
- Die <mark style="background: #FFB86CA6;">resultierenden Farben werden anschließend über die Fläche des Dreiecks interpoliert</mark>.
### Ablauf
1. Für jeden Eckpunkt (Vertex) wird die Beleuchtung basierend auf der Normale an diesem Punkt berechnet (Phong-Beleuchtungsmodell).
2. Die <mark style="background: #FFB86CA6;">berechneten Farben an den Eckpunkten werden entlang der Kanten interpoliert</mark>.
3. Für <mark style="background: #FFB86CA6;">jeden Pixel innerhalb des Dreiecks wird die Farbe aus der Kanteninterpolation berechnet</mark>.
### Vorteile 
- <mark style="background: #FFB86CA6;">Rechenaufwand ist gering</mark>, da die Beleuchtung nur an wenigen Punkten berechnet wird (nur an den Eckpunkten).
	- z.B. ein Dreieck wird nur an den 3 Eckpunkten berechnet
- Effizient und für große Dreiecke mit wenig Detail geeignet.
