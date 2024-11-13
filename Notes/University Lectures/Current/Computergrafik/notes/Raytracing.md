Raytracing ist eine Methode in der Computergrafik, die realistische Bilder erzeugt, indem sie das Verhalten von Lichtstrahlen (Rays) simuliert. Die Idee ist, Lichtstrahlen von der Kamera aus durch jedes Pixel eines Bildes zu senden und deren Weg in der virtuellen Szene zu verfolgen. So kann man berechnen, wie das Licht von verschiedenen Objekten reflektiert, gebrochen oder gestreut wird.

- Bildsyntheseverfahren
- Beim Raytracing schickt man Lichtstrahlen durch die "Öffnung" durch die Pixel der Bildebene in die Szene, um zu berechnen, wie das Bild aussehen würde. 
	- -> [[Pinhole Camera]]
	- Da es keine Linse gibt, ist das Bild immer scharf, was das Modell vereinfacht und für die Berechnungen praktisch macht.

## Grundidee: Whitted-Style Raytracing
### Primärstrahl (Sichtstrahl):
- Für jeden Pixel des Bildes wird ein Strahl (Ray) von der Kamera aus durch das Pixel in die Szene geschickt. Dieser Strahl wird "Primärstrahl" oder "Sichtstrahl" genannt.
- Der Strahl bewegt sich entlang seiner Richtung und trifft auf das nächstgelegene Objekt in der Szene.
![[Pasted image 20241108082747.png|400]]
#### Erzeugung der Sichtstrahlen (Ray Generation)
Für jedes Pixel wird ein Sichtstrahl berechnet, der von der Kameraposition `e` zu einem bestimmten Punkt auf der Bildfläche verläuft:
- Der Vektor `s`, der die Richtung des Strahls angibt, wird berechnet durch:
  $$
  \mathbf{s} = \mathbf{u} \cdot u + \mathbf{v} \cdot v - d \cdot \mathbf{w}
  $$
  Dabei variieren `u` und `v` entsprechend den Pixelkoordinaten, wobei $u \in [l, r]$ und $v \in [b, t]$.
#### Pseudocode: ein Zielpunkt pro Pixel 
```python
for ( y = 0; y < height; y++ ) { 
	for ( x = 0; x < width; x++ ) { 
		u = l + (r-l) * (x+0.5) / width; 
		v = t + (b-t) * (y+0.5) / height; 
		... 
	} 
}
```
- Durch Hinzufügen von `0.5` wird die Mitte des Pixels berücksichtigt, was das sogenannte "Pixel-Center Sampling" ist und zu einer präziseren Berechnung führt.
### Schnittberechnung (ray casting, ray intersection)
- finde Dreieck, das den Sichtstrahl am nächsten zur Kamera schneidet
![[Pasted image 20241108091258.png|400]]
- Dieser Code geht durch jedes Pixel des Bildes, berechnet für jedes Pixel einen Sichtstrahl, und prüft dann, ob dieser Strahl ein Objekt in der Szene trifft. Wenn ja, wird das nächste Objekt (also das Objekt, das der Kamera am nächsten liegt) gespeichert.
```python
for ( y = 0; y < height; y++ ) {
    for ( x = 0; x < width; x++ ) {
		u = l + (r-l) * (x+0.5) / width;
		v = t + (b-t) * (y+0.5) / height;
		s = ...;
		d = normalize( s );

		# Finde nächsten Schnittpunk
		intersection = NULL;
		float t = FLOAT_MAX;

		for ( each object ) {
		    t' = intersect( object, e, d );
		    if ( t' > 0 && t' < t ) {
		        intersection = object;
		        t = t';
		    }
		}

```
- `s` ist der Vektor, der von der Kameraposition `e` zum Punkt $(u,v)$ auf der Bildfläche zeigt. `d` ist dann der normalisierte Richtungsvektor des Sichtstrahls, der in die Szene gesendet wird.
- `intersection` ist die Variable, die das nächstliegende getroffene Objekt speichert, und `t` speichert die Distanz des nächsten Schnittpunkts. `FLOAT_MAX` ist ein großer Wert, der die maximale Distanz symbolisiert.
- Für jedes Objekt in der Szene wird die Funktion intersect aufgerufen, um zu berechnen, ob und wo der Sichtstrahl das Objekt schneidet.
- t' gibt die Distanz des Schnittpunkts zwischen dem Strahl und dem Objekt an.
- Wenn t' > 0, liegt der Schnittpunkt vor der Kamera (wir interessieren uns nur für Objekte vor der Kamera).
- Wenn t' < t, ist der Schnittpunkt näher an der Kamera als vorherige Schnittpunkte, und intersection wird auf das aktuelle Objekt gesetzt, und t wird auf t' aktualisiert.
### Schattierung (Lighting/Illumination):
- Sobald der Primärstrahl ein Objekt trifft, wird die Beleuchtung an diesem Punkt berechnet.
- Die Berechnung umfasst die Einflüsse aller Lichtquellen in der Szene, wodurch die Helligkeit und die Farbe der Oberfläche bestimmt werden.
![[Pasted image 20241108082813.png|400]]
- Schattierung ist essentiell für dreidimensionalen Eindruck und ist ein Ergebnis von 
	- Emission der Lichtquellen (Intensität, Farbe, Position, …) und 
	- Oberflächeneigenschaften (Material, Rauheit, Orientierung zur LQ, …
#### Licht-Material Interaktion
**Was passiert, wenn Licht auf eine glatte Oberfläche trifft?**
- Wenn Licht auf eine glatte (d.h., ebene und polierte) Oberfläche trifft, teilt es sich auf: Ein Teil des Lichts wird reflektiert, während der andere Teil in das Material eindringt.
![[Pasted image 20241108092013.png|100]]
- „Glatt“ bedeutet, dass die Oberfläche keine sichtbaren Unregelmäßigkeiten oder Rauheit hat. Eine glatte Oberfläche reflektiert das Licht in eine bestimmte Richtung (spiegelnd), im Gegensatz zu einer rauen Oberfläche, die das Licht in viele Richtungen streut.
- Das Verhältnis zwischen reflektiertem und eindringendem Licht hängt vom **Einfallswinkel** des Lichts und den **Brechungsindizes** beider Materialien ab (also des Mediums, aus dem das Licht kommt, und des Materials, in das es eindringt).
- Diese Abhängigkeit wird durch den **Fresnel-Effekt** beschrieben. Bei flachem Einfallswinkel wird mehr Licht reflektiert, während bei steilerem Winkel mehr Licht in das Material eindringt.
- Wenn Licht in das Material eindringt, kann es:
	- **Absorbiert werden** (besonders bei Metallen): Das Licht verliert dabei schnell seine Energie und wird in Wärme umgewandelt, was das typische Verhalten von Metallen erklärt.
	- **Aus dem Material wieder austreten**: Bei durchsichtigen oder transluzenten Materialien (wie Glas) kann das Licht das Material an einer anderen Stelle verlassen, was zu Effekten wie Lichtbrechung und Transparenz führt.
- [[Lokales Beleuchtungsmodell]]
#### Pseudocode
```python
function computeDirectLight(intersection i, light_position pos, light_intensity I_L):
    // Initialisiere die Lichtintensität mit dem ambienten Term
    I = i.material.ka * I_L

    // Berechne den Lichtvektor von der Oberfläche zur Lichtquelle
    L = normalize(pos - i.position)

    // Berechne den Winkel zwischen der Normalen und dem Lichtvektor
    NdotL = dot(i.normal, L)

    // Diffuser Term: Trage bei, wenn der Punkt von der Lichtquelle beleuchtet wird
    if (NdotL > 0):
        I += i.material.kd * I_L * NdotL

    // Spekularer Term: Berechne Reflexionsvektor und Glanzlicht
    R = 2 * i.normal * NdotL - L
    RdotV = dot(R, i.view_direction)

    if (RdotV > 0):
        I += i.material.ks * I_L * (RdotV ^ i.material.shininess)

    // Optional: Punktlichtquelle mit Distanzabnahme
    distance = length(pos - i.position)
    I += (i.material.kd * I_L * NdotL) / (distance ^ 2)

    return I

```
- Der Ambient-Term repräsentiert die allgemeine Umgebungsbeleuchtung, die auf den Punkt trifft. Dieser Wert wird durch den Ambient-Koeffizienten `ka` des Materials und die Lichtintensität `I_L` skaliert.
- Dieser Term ist unabhängig von der Licht- oder Blickrichtung und sorgt dafür, dass auch in schattigen Bereichen eine Grundbeleuchtung vorhanden ist.
- `L` ist der normalisierte Vektor von der Oberfläche zur Lichtquelle.
- `NdotL` ist das Skalarprodukt zwischen der Oberflächennormalen `i.normal` und dem Lichtvektor `L`. Dieses Produkt beschreibt, wie stark das Licht auf die Oberfläche fällt. Wenn `NdotL` positiv ist, wird die Fläche von der Lichtquelle beleuchtet.
- Wenn `NdotL > 0`, addiert der Code den diffusen Beleuchtungsterm zur Lichtintensität `I`. Der Faktor `kd` ist der diffuse Reflexionskoeffizient des Materials, der die Farbe und Reflexionsstärke bestimmt.
- `R` ist der Reflexionsvektor, der durch die Spiegelung des Lichtvektors `L` an der Normalen berechnet wird.
- `RdotV` ist das Skalarprodukt zwischen dem Reflexionsvektor `R` und der Blickrichtung `i.view_direction`. Dieser Wert beschreibt, wie stark die Reflexion in Richtung des Betrachters erfolgt.
- Wenn `RdotV > 0`, wird der spekulare Term zur Lichtintensität `I` addiert. Der spekulare Koeffizient `ks` und der Phong-Exponent `shininess` des Materials bestimmen die Intensität und Größe des Glanzlichts.
- Bei einer Punktlichtquelle wird die Lichtintensität durch den Abstand zur Lichtquelle beeinflusst.
- Hier wird `distance` berechnet, und der diffuse Term wird durch das Quadrat der Entfernung geteilt, um die Distanzabnahme zu simulieren. Dies sorgt dafür, dass weiter entfernte Punkte weniger stark beleuchtet werden.
- Die gesamte berechnete Lichtintensität `I` wird zurückgegeben. Sie kombiniert den Ambient-, Diffus- und Spekular-Termin und ergibt die Lichtstärke am gegebenen Punkt auf der Oberfläche.
### Schattenstrahl (Shadow Ray):
- Für jeden sichtbaren Punkt wird zusätzlich ein "Schattenstrahl" von diesem Punkt zu den Lichtquellen geschickt.
- Wenn ein Schattenstrahl ein Objekt auf dem Weg zur Lichtquelle trifft, bedeutet das, dass das Objekt im Schatten ist und entsprechend weniger beleuchtet wird. Dieser Vorgang erzeugt realistische Schatten im Bild.
![[Pasted image 20241108084815.png|400]]
### Schattierung von Dreiecksnetzen
Die **Schattierung von Dreiecksnetzen** bezieht sich darauf, wie Licht auf einem polygonalen Modell dargestellt wird, um eine realistische Beleuchtung und eine glatte Oberfläche zu simulieren. Hier sind die wichtigsten Konzepte und Techniken zur Schattierung von Dreiecksnetzen:
#### Normale für Dreiecksnetze und Glättung
- **Vertex-Normalen**: Um eine glatte, gekrümmte Fläche zu simulieren, wird an jedem Eckpunkt (Vertex) eine Normale definiert, anstatt nur eine Normale für jedes Dreieck. Diese sogenannten "Vertex-Normalen" geben die Richtung des Lichtreflexionsvektors am Vertex an.
- **Interpolation der Normalen**: Die Normalen an den Dreiecks-Vertices werden interpoliert, um die Illusion einer glatten Fläche zu erzeugen, auch wenn das Modell aus flachen Dreiecken besteht.
![[Pasted image 20241108100810.png#invert|200]]
#### Berechnung der Vertex-Normalen
- Die Vertex-Normalen können entweder:
  - **Berechnet** werden (zum Beispiel bei parametrischen Flächen), oder
  - **Direkt aus dem Dreiecksnetz** bestimmt werden.
  Bei der Bestimmung aus dem Dreiecksnetz werden die Normalen der angrenzenden Dreiecke kombiniert, um die Normale an einem Vertex zu berechnen.
![[Pasted image 20241108100825.png#invert|200]]
#### Vorgehensweise zur Bestimmung der Normalen
- **Berechne die Normale für jedes Dreieck**: Die Normale eines Dreiecks wird durch das Kreuzprodukt zweier Kanten des Dreiecks berechnet.
- **Summiere die Normalen an jedem Vertex**: Für jeden Vertex wird die Summe der Normalen aller angrenzenden Dreiecke gebildet.
  - Optional: Skaliere die Normalen mit der Fläche des Dreiecks, um die Gewichtung der angrenzenden Flächen zu berücksichtigen.
- **Normalisiere die Vertex-Normalen**: Da durch das Summieren die Länge der Normalen verzerrt sein kann, müssen die Normalen normiert werden.
#### Scharfe Kanten
- **Scharfe Kanten erfordern mehrere Normalen**: An scharfen Kanten (d.h., wenn der Winkel zwischen zwei benachbarten Dreiecken zu groß ist) wird die glatte Interpolation unterbrochen. An diesen Stellen kann es notwendig sein, separate Normalen für jede Seite der Kante zu definieren, um den kantigen Effekt zu erhalten.
- Bei Dreiecksnetzen wird pro Dreieck üblicherweise ein Set aus drei Eckpunkten und drei Normalen gespeichert, um die Flexibilität für glatte und scharfe Kanten zu gewährleisten.
![[Pasted image 20241108100855.png#invert|200]]
#### Interpolation von Normalen
- Um die Beleuchtung entlang der Oberfläche zwischen den Eckpunkten zu berechnen, werden die Normalen **linear interpoliert**. Diese Interpolation erfolgt auf Basis der baryzentrischen Koordinaten des Punktes innerhalb des Dreiecks.
- Für einen Punkt $Q$ innerhalb eines Dreiecks mit den Eckpunkten $P_1$, $P_2$, und $P_3$ und den Normalen $\mathbf{n}_1$, $\mathbf{n}_2$, und $\mathbf{n}_3$:
  - Berechne die baryzentrischen Gewichte $\lambda_1$, $\lambda_2$, $\lambda_3$.
  - Die interpolierte Normale $\mathbf{n}_Q$ ist dann:
    $$
    \mathbf{n}_Q = \lambda_1 \mathbf{n}_1 + \lambda_2 \mathbf{n}_2 + \lambda_3 \mathbf{n}_3
    $$
- **Phong Shading**: Diese Technik, bei der die Normalen interpoliert und für die Beleuchtungsberechnung verwendet werden, wird **Phong Shading** genannt und unterscheidet sich vom Phong-Beleuchtungsmodell.
#### Wichtige Hinweise zur Interpolation
- Die **lineare Interpolation** der Normalen kann dazu führen, dass die Länge der interpolierten Normalen nicht konstant bleibt. Daher muss die interpolierte Normale $\mathbf{n}_Q$ in der Regel normalisiert werden.
### Sekundärstrahlen für Reflexion und Brechung:
- Wenn die getroffene Oberfläche spiegelnd ist, wird ein neuer Strahl, der "Reflexionsstrahl," von der Oberfläche aus reflektiert. Dieser Strahl bewegt sich in die Reflexionsrichtung und trifft auf ein weiteres Objekt in der Szene.
- Für transparente Objekte wird ein "Brechungsstrahl" erzeugt, der das Licht durch das Objekt hindurch weiterverfolgt und die Lichtbrechung simuliert.
- Diese Sekundärstrahlen setzen die Strahlverfolgung (Raytracing) fort und werden rekursiv weiterverfolgt, bis sie eine bestimmte maximale Tiefe erreichen oder keinen weiteren Effekt haben.
![[University Lectures/Current/Computergrafik/notes/images/Untitled.png|400]]
- Rekursives Raytracing:
  ![[Pasted image 20241113092607.png|400]]
### Kombination der Ergebnisse:
- Die Ergebnisse der Schattierung, Schattenstrahlen und Sekundärstrahlen (für Reflexion und Brechung) werden kombiniert, um die endgültige Farbe und Helligkeit des Pixels festzulegen.
- Dieser Prozess wird für jedes Pixel des Bildes wiederholt, um ein vollständiges, realistisches Bild zu erstellen.
![[University Lectures/Current/Computergrafik/notes/images/Untitled 1.png|400]]