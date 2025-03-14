Hier ist eine Erklärung des **Snell'schen Brechungsgesetzes** und der Konzepte, die in den Folien dargestellt sind:

### 1. Grundlagen des Snell'schen Brechungsgesetzes
Das Snell'sche Brechungsgesetz beschreibt, wie sich die <mark style="background: #FFB86CA6;">Richtung einer Lichtwelle ändert, wenn sie von einem Medium in ein anderes übergeht</mark>, das eine andere optische Dichte (Brechzahl) hat. Dies geschieht <mark style="background: #FFB86CA6;">aufgrund der unterschiedlichen Lichtgeschwindigkeiten in den beiden Medien</mark>.

- **Brechzahl** $\eta$: Ein Maß für die Geschwindigkeit des Lichts in einem Medium, definiert als:
  $$
  \eta = \frac{c_0}{c_\eta}
  $$
  wobei $c_0$ die <mark style="background: #FFB86CA6;">Lichtgeschwindigkeit im Vakuum</mark> und $c_\eta$ die <mark style="background: #FFB86CA6;">Lichtgeschwindigkeit im Medium</mark> ist.

- Wenn das Licht auf die Grenzfläche zwischen zwei Medien trifft, wird es entweder reflektiert oder gebrochen. Die **Ausbreitungsrichtung** des Lichts ist senkrecht zur Wellenfront.

### 2. Wellenlängenabhängigkeit der Brechzahl
- Die <mark style="background: #FFB86CA6;">Brechzahl ist oft wellenlängenabhängig</mark> ($\eta(\lambda)$), was als Dispersion bezeichnet wird. Dies führt zur Aufspaltung des Lichts in verschiedene Wellenlängen, wie man es bei einem Prisma sieht, wo Licht in seine Spektralfarben zerlegt wird.

### 3. Das Snell'sche Gesetz
Das Snell'sche Gesetz lautet:
$$
\eta_i \sin \theta_i = \eta_t \sin \theta_t
$$
wobei:
- $\theta_i$ der Einfallswinkel im Medium 1 (mit Brechzahl $\eta_i$) ist.
- $\theta_t$ der Brechungswinkel im Medium 2 (mit Brechzahl $\eta_t$) ist.

- **Übergang in optisch dichteres Medium** ($\eta_t > \eta_i$): Der Lichtstrahl wird zum Lot hin gebrochen.
- **Übergang in optisch dünneres Medium** ($\eta_t < \eta_i$): Der Lichtstrahl wird vom Lot weg gebrochen.

### 4. Fresnel-Effekt
Der <mark style="background: #FFB86CA6;">Fresnel-Effekt beschreibt</mark>, wie sich die Intensität des <mark style="background: #FFB86CA6;">reflektierten und gebrochenen Lichts in Abhängigkeit vom Einfallswinkel ändert</mark>. Bei einem flachen Einfallswinkel wird mehr Licht reflektiert, während bei steilem Einfallswinkel mehr Licht gebrochen wird.

### 5. Grenzwinkel der Totalreflexion
- Es gibt einen <mark style="background: #FFB86CA6;">kritischen Grenzwinkel</mark> $\theta_c$, bei dem <mark style="background: #FFB86CA6;">das Licht bei einem Übergang von einem optisch dichteren zu einem optisch dünneren Medium vollständig reflektiert wird, anstatt gebrochen zu werden</mark>.
- Der kritische Winkel ist definiert als:
  $$
  \sin \theta_c = \frac{\eta_t}{\eta_i}
  $$
  Wenn $\theta_i > \theta_c$, tritt **Totalreflexion** auf.

### 6. Transmissionsvektor (Brechungsrichtung berechnen)
Für die <mark style="background: #FFB86CA6;">Berechnung des Transmissionsvektors (d.h., der Richtung des gebrochenen Strahls) </mark>in der Computergrafik wird die folgende Formel verwendet:
$$
\mathbf{t} = -\frac{\sin \theta_t}{\sin \theta_i} (\mathbf{i} - \cos \theta_i \mathbf{n}) - \cos \theta_t \mathbf{n}
$$
oder eine weiter ausgeführte Version:
$$
\mathbf{t} = \frac{\eta_i}{\eta_t} \mathbf{i} + \left( \frac{\eta_i}{\eta_t} \cos \theta_i - \sqrt{1 - \left( \frac{\eta_i}{\eta_t} \right)^2 (1 - \cos^2 \theta_i)} \right) \mathbf{n}
$$
wobei:
- $\mathbf{i}$: Einfallsvektor,
- $\mathbf{n}$: Normalenvektor,
- $\mathbf{r}$: Reflexionsvektor,
- $\mathbf{t}$: Transmissions- oder Brechungsvektor.

Diese Gleichungen helfen, den Weg des Lichtstrahls zu bestimmen, wenn er von einem Medium ins andere übergeht, und sind entscheidend für die Simulation realistischer Reflektionen und Brechungen in der Computergrafik.

