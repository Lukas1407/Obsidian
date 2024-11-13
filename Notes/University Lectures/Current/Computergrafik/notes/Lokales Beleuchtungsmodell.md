## BRDF (Bidirectional Reflectance Distribution Function)
Die BRDF beschreibt, wie Licht von einer Oberfläche reflektiert wird. Sie gibt das Verhältnis zwischen der Lichtmenge, die in eine bestimmte Richtung auf eine Oberfläche trifft, und der Lichtmenge, die in eine andere Richtung von dieser Oberfläche reflektiert wird.
- **Einfallsrichtung (`l`)**: Die Richtung, aus der das Licht auf die Oberfläche trifft.
- **Betrachterrichtung (`v`)**: Die Richtung, in der das reflektierte Licht zum Betrachter gelangt.
- **Normale (`n`)**: Der Vektor, der senkrecht zur Oberfläche steht und die Orientierung der Oberfläche angibt.
![[Pasted image 20241108095704.png#invert|400]]

### Funktionsweise
- Die BRDF ist eine Funktion $f_r(\mathbf{l}, \mathbf{x}, \mathbf{v})$, die beschreibt, wie Licht in verschiedene Richtungen reflektiert wird.
- Sie verwendet ein **Referenzkoordinatensystem** mit den Vektoren **Tangente (`t`)**, **Bitangente (`b`)**, und **Normale (`n`)**. Diese Vektoren spannen ein Koordinatensystem auf, um die Orientierung der Oberfläche festzulegen.
- Die BRDF hängt von Winkeln ab:
  - **Polarwinkel** $\theta$: Der Winkel zwischen Einfallsrichtung und Normale sowie zwischen Beobachtungsrichtung und Normale.
  - **Azimutalwinkel** $\phi$: Der Winkel in der horizontalen Ebene zwischen den Einfalls- und Beobachtungsrichtungen.
![[Pasted image 20241108095815.png#invert|400]]
### Radiometrische Größen
Die BRDF basiert auf radiometrischen Größen:
- **Radiance (Strahldichte)**: Die Menge an Licht, die in Richtung des Betrachters (`v`) reflektiert wird, gemessen in $\text{W}/\text{m}^2\text{sr}$.
- **Irradiance (Flussdichte)**: Die Menge an Licht, die auf die Oberfläche in Richtung `l` trifft, gemessen in $\text{W}/\text{m}^2$.
- Die BRDF kann als das Verhältnis der Strahldichte zur einfallenden Flussdichte betrachtet werden: Sie beschreibt, wie viel Licht in eine bestimmte Richtung reflektiert wird.
### Anwendung: Anisotrope und isotrope BRDF
- **Anisotrope BRDF**: Die Reflexion hängt von der Orientierung der Oberfläche ab. Dies tritt auf, wenn die Oberfläche eine **Mikrostruktur** aufweist, wie z. B. gebürstetes Metall, das das Licht in bestimmten Richtungen stärker reflektiert.
- **Isotrope BRDF**: Die Reflexion ist rotationsinvariant um die Normale; die Reflexionseigenschaften ändern sich nicht bei Drehung der Oberfläche. Dies ist typisch für glatte, gleichmäßig strukturierte Oberflächen.
### Verallgemeinerung: BRDF und BTDF
- **BTDF (Bidirectional Transmission Distribution Function)** beschreibt die Lichtübertragung durch das Material (Transparenz).
- Die **BSDF (Bidirectional Scattering Distribution Function)** kombiniert BRDF und BTDF und beschreibt sowohl die Reflexion als auch die Transmission von Licht.
### Messung von BRDFs
- Ein **Gonioreflektometer** ist ein Messinstrument, das die Reflektionseigenschaften realer Materialproben misst. Dabei wird das Reflexionsverhalten bei verschiedenen Einfalls- und Blickwinkeln analysiert.
- Die **Messung realer Materialproben** ist jedoch sehr aufwändig und zeitintensiv. Zudem können die Daten durch Rauschen oder andere Unregelmäßigkeiten verfälscht sein.
## Phong-Beleuchtungsmodell
Das Phong-Beleuchtungsmodell ist ein **phänomenologisches Modell**, das die Reflexion von Licht auf einer Oberfläche mit drei Hauptkomponenten beschreibt:
- **Ambient (Umgebungslicht)**: Modelliert die indirekte Beleuchtung durch Licht, das von anderen Oberflächen reflektiert wird. Diese Komponente sorgt dafür, dass die gesamte Szene eine Grundhelligkeit hat.
- **Diffuse Reflexion**: Nach dem Lambert’schen Gesetz reflektiert eine Oberfläche Licht gleichmäßig in alle Richtungen, unabhängig vom Betrachtungswinkel. Diese Reflexion ist abhängig vom Winkel zwischen der Lichtrichtung und der Normalen der Oberfläche.
- **Spekulare Reflexion**: Modelliert Glanzlichter, die entstehen, wenn Licht in einer bevorzugten Richtung reflektiert wird, wie bei einer Spiegelung. Diese Reflexion erzeugt Lichtpunkte und Glanz auf der Oberfläche.
![[Pasted image 20241108100130.png|400]]
### Diffuse Reflexion (Lambert’sche Reflexion)
- Die **Lambert’sche Reflexion** tritt auf, wenn Licht gleichmäßig in alle Richtungen reflektiert wird.
- Die Intensität der diffusen Reflexion wird berechnet als $k_d \cdot I_L \cdot \cos(\theta)$, wobei:
  - $k_d$ der Materialkoeffizient ist (beschreibt die Farbe und Reflexionsstärke des Materials).
  - $I_L$ die Lichtintensität.
  - $\cos(\theta)$ das Verhältnis zwischen der Lichtrichtung `l` und der Normalen `n` der Oberfläche, berechnet als $\cos(\theta) = \mathbf{n} \cdot \mathbf{l}$ (Skalarprodukt).
![[Pasted image 20241108100156.png#invert|400]]
### Spekulare Reflexion (Glanzlichter)
- Bei der **spekularen Reflexion** wird Licht bevorzugt in einer bestimmten Richtung reflektiert, die von der **Reflexionsrichtung** $\mathbf{r}_1$ abhängt.
- **Perfekte Spiegelung**: Der Reflexionsvektor $\mathbf{r}_1$ wird durch die Formel berechnet:
  $$
  \mathbf{r}_1 = 2(\mathbf{l} \cdot \mathbf{n}) \mathbf{n} - \mathbf{l}
  $$
  Dabei ist `l` die Lichtrichtung, `n` die Normale der Oberfläche. Dieser Vektor beschreibt die Richtung, in die das Licht bei einer idealen Spiegelung reflektiert wird.
![[Pasted image 20241108100217.png#invert|400]]
### Phong-Exponent und Abnahme der spekularen Reflexion
- **Imperfekte Spiegelung**: Nicht alle Oberflächen sind perfekte Spiegel; sie reflektieren Licht in einem Bereich um $\mathbf{r}_1$ herum.
- Der **Phong-Exponent** $n$ steuert, wie stark das Glanzlicht fokussiert ist:
  - **Große Werte von $n$** führen zu kleinen, scharf begrenzten Glanzlichtern (z. B. bei glänzenden Oberflächen).
  - **Kleine Werte von $n$** führen zu breiteren, weniger konzentrierten Glanzlichtern (z. B. bei matten Oberflächen).
- Die spekulare Reflexion wird berechnet als:
  $$
  I_s = k_s \cdot I_L \cdot \cos^n(\alpha) = k_s \cdot I_L \cdot (\mathbf{r}_1 \cdot \mathbf{v})^n
  $$
  - Hier ist $k_s$ der spekulare Reflexionskoeffizient, $I_L$ die Lichtintensität, und $\alpha$ der Winkel zwischen der Reflexionsrichtung $\mathbf{r}_1$ und der Blickrichtung $\mathbf{v}$.
  - **Hinweis**: Der Ausdruck $(\mathbf{r}_1 \cdot \mathbf{v})$ kann als Maß dafür betrachtet werden, wie gut die Reflexion mit der Blickrichtung übereinstimmt. Ein hoher Wert bedeutet ein starkes Glanzlicht in Blickrichtung.


## Phong-Shading
- Beleuchtungsberechnung mit interpolierten Normalen nennt man Phong Shading