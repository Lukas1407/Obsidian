- **Kleine Öffnung:** Die Lochkamera hat eine kleine Öffnung an der Vorderseite und eine Bildebene gegenüber der Öffnung, auf die das Licht projiziert wird.
- **Bildentstehung:** Lichtstrahlen, die durch diese kleine Öffnung eintreten, erzeugen ein Bild auf der Bildebene – dabei wird das Bild spiegelverkehrt und auf den Kopf gestellt.
- **Unbegrenzte Schärfentiefe:** Da es nur eine kleine Öffnung und keine Linse gibt, ist das gesamte Bild scharf, unabhängig von der Entfernung zum Objekt.
![[Pasted image 20241108082542.png|400]]
## Definition der Kamera in der Szene
![[Pasted image 20241108085917.png|100]]
Ich erkläre gern die Details zur **Ray Generation** anhand der Bilder und Punkte:

### 1. Definition der Kamera in der Szene
In der Computergrafik wird eine virtuelle Kamera, oft nach dem Lochkamera-Modell, definiert, um die Sichtstrahlen (rays) zu erzeugen:
- **Position** (`e`): Das ist das Projektionzentrum, oft als „eye“ oder Kameraposition bezeichnet.
- **Zielpunkt** (`z`): Das ist der Punkt, auf den die Kamera fokussiert ist, also der Blickpunkt in der Szene.
- **Blickrichtung** (`w`): Die negative Blickrichtung wird durch den Vektor $\mathbf{w} = \frac{e - z}{|e - z|}$ berechnet. Er zeigt von der Kamera weg in die entgegengesetzte Richtung des Blicks.
- **Up-Vektor** (`up`): Dies ist ein Vektor, der angibt, welche Richtung „oben“ ist, meist normiert auf eine Länge von 1.
![[Pasted image 20241108085947.png|200]]
### 2. Aufbau der lokalen Kamera-Koordinaten
Um die Orientierung der Kamera vollständig zu definieren, benötigen wir drei orthogonale Vektoren:
- **Vektor $\mathbf{u}$**: Wird durch das Kreuzprodukt von `up` und `w` berechnet. Er gibt die horizontale Richtung der Kamera an.
- **Vektor $\mathbf{v}$**: Wird durch das Kreuzprodukt von `w` und `u` gebildet und gibt die vertikale Richtung an.
- **Normalisierung**: Beide Vektoren $\mathbf{u}$ und $\mathbf{v}$ werden normiert, um eine konsistente Skala und Orientierung zu haben.
### 3. Festlegen der Bildfläche (Image Plane)
Die Bildfläche, die in einem bestimmten Abstand $d$ zur Kamera liegt, definiert, wohin die Sichtstrahlen geschickt werden:
- Die Bildfläche hat linke (`l`), rechte (`r`), untere (`b`) und obere (`t`) Grenzen, die bestimmen, wie groß der sichtbare Bereich ist.
- Der Abstand `d` zur Kamera ist die Distanz zwischen der Kameraposition und der Bildfläche.
![[Untitled 2.png|200]]

### 5. Beispiel für typisches Sichtfeld und Pseudocode
- Ein Sichtfeld von 90° ist typisch, was zu symmetrischen Grenzen führt (z.B., $l = -r$ und $b = -t$).
- Der **Pseudocode** zeigt, wie man für jedes Pixel `u` und `v` berechnet und damit den Zielpunkt bestimmt. Hier wird jeder Pixelkoordinate ein Punkt auf der Bildfläche zugewiesen, von dem aus ein Strahl in die Szene geschickt wird.

Zusammengefasst erzeugt diese Methode eine präzise Kameraansicht in der Szene, von der aus für jedes Pixel ein Strahl ausgeht, um zu berechnen, welche Objekte sichtbar sind und wie sie dargestellt werden.

