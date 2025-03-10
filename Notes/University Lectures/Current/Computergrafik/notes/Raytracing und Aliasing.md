Beim Raytracing wird das Bild einer 3D-Szene als ein **zweidimensionales Signal** dargestellt, das aus diskreten Pixelwerten besteht. Für jeden Pixel wird ein Strahl durch dessen Mittelpunkt verfolgt, um zu bestimmen, welche Objekte sichtbar sind und welche Farbe der Pixel erhält.
- **Abtastung**: <mark style="background: #FFB86CA6;">Die Farben werden durch Abtastung der Szene an den Pixelmittelpunkten berechnet</mark>, was als Stückweise-konstante-Funktion verstanden werden kann.
- **Nicht bandbegrenztes Bildsignal**: Objektkanten haben oft hohe Frequenzkomponenten, was bedeutet, dass das Bildsignal nicht bandbegrenzt ist.
- **Treppeneffekte**: <mark style="background: #FFB86CA6;">Die quadratische Form der Pixel kann dazu führen, dass Kanten in der Szene als „Treppenstufen“</mark> (Jaggies) dargestellt werden. Dieser Effekt tritt bei der Rekonstruktion des Signals auf.
### 2. Beispiel für Aliasing
<mark style="background: #FFB86CA6;">Aliasing tritt auf, wenn die Abtastung eines Signals nicht fein genug ist</mark>. Dies führt dazu, dass im <mark style="background: #FFB86CA6;">rekonstruierten Bild Frequenzen auftauchen, die im Original nicht vorhanden sind</mark> (siehe rechte Seite der Abbildung). Dies passiert typischerweise bei fein strukturierten Details oder Kanten im Bild.
### 3. Nyquist-Shannon-Abtasttheorem
<mark style="background: #FFB86CA6;">Um Aliasing zu vermeiden, muss ein Signal mit einer Abtastfrequenz, die mindestens doppelt so hoch ist wie die höchste Frequenz im Signal</mark>, abgetastet werden:
$$
f_{\text{abtast}} > 2 f_{\text{max}}
$$
Nur so kann das Originalsignal exakt aus dem diskreten Signal rekonstruiert werden.
![[Pasted image 20241113093355.png#invert|300]]
### 4. Aliasing in der Computergrafik
Aliasing ist besonders problematisch in der Computergrafik, da <mark style="background: #FFB86CA6;">feine Details (z.B. Kanten, Texturen) oft hochfrequente Signalanteile</mark> haben:
- **Detallierte Geometrie**: <mark style="background: #FFB86CA6;">Kleine geometrische Details verursachen Aliasing</mark>, wenn sie nicht ausreichend aufgelöst werden.
- **Texturen und Textursignale**: <mark style="background: #FFB86CA6;">Hohe Frequenzen in Texturen können Aliasing verursachen</mark>, wie z.B. Farbübergänge oder Normalmaps.
- **Shading**: <mark style="background: #FFB86CA6;">Glanzlichter oder andere Schattierungseffekte auf gekrümmten Oberflächen</mark> können ebenfalls zu Aliasing führen.
![[Pasted image 20241113093407.png|500]]
## Lösungsansätze
### Vorfilterung des Signals
- <mark style="background: #FFB86CA6;">Bevor das Signal abgetastet wird, wird eine Vorfilterung angewendet, um hohe Frequenzen zu entfernen</mark>. In Texturen kann diese Methode helfen, aber im Allgemeinen ist es schwierig, hohe Frequenzen vollständig zu eliminieren. Ohne Vorfilterung könnten hohe Frequenzen im abgetasteten Bild als Aliasing-Effekte auftreten.
### Erhöhung der Abtastrate (Supersampling)
- <mark style="background: #FFB86CA6;">Bei einer höheren Abtastrate wird das Bild mit mehr Probenpunkten pro Pixel abgetastet, und die Werte dieser Probenpunkte werden dann gemittelt</mark>. Diese Methode, oft als Supersampling bezeichnet, reduziert Aliasing-Effekte, da mehr Informationen über das Signal gesammelt werden. Im rechten Bild sieht man, dass durch Überabtastung die Details klarer und weniger „kantig“ erscheinen.
![[Pasted image 20241113100446.png#invert|400]]
- Im Bild wird gezeigt, dass unterschiedliche geometrische Formen, die mehrere Pixel überdecken, durch mehr Abtastpunkte feiner aufgelöst werden. Ein Pixel kann dadurch die Farbe von angrenzenden Objekten genauer wiedergeben.
### **Uniformes Supersampling**:
- Bei uniformem Supersampling wird der <mark style="background: #FFB86CA6;">gesamte Pixelbereich in ein regelmäßiges Gitter unterteilt</mark>, in dem gleichmäßig verteilt Proben genommen werden. So hat man bei einer uniformen Abtastung (z. B. 4x oder 9x Sampling) eine <mark style="background: #FFB86CA6;">äquidistante Verteilung der Sample-Punkte im Pixel</mark>.
- Die Werte dieser Samples werden gemittelt, um die finale Farbe des Pixels zu bestimmen. So wird das Ergebnis glatter und die Kanten erscheinen weicher.
![[Pasted image 20241113100530.png#invert|300]]
### Verschiedene Abtastdichten
- Das Bild zeigt, dass die Abtastdichte pro Pixel variieren kann. Von 1 Sample pro Pixel bis zu 9 Samples pro Pixel kann der Pixelinhalt unterschiedlich genau erfasst werden. <mark style="background: #FFB86CA6;">Je mehr Samples, desto detaillierter ist die Farbinformation im Pixel, was zu glatteren Übergängen und weniger Aliasing führt.</mark>
### Stochastisches Sampling (Zufällige Verteilung der Samples):
- **Zufällige Verteilung (Stochastisches Sampling)**: <mark style="background: #FFB86CA6;">Anstatt eines festen Gitters werden die Sample-Punkte zufällig verteilt</mark>. Das <mark style="background: #FFB86CA6;">reduziert Aliasing, kann aber leicht Rauschen im Bild erzeugen</mark>. Die zufälligen Positionen der Samples stören das gleichmäßige Rastern und <mark style="background: #FFB86CA6;">vermeiden so harte Kanten</mark>.
- **Stratifiziertes Sampling**: Bei stratifiziertem Sampling wird der <mark style="background: #FFB86CA6;">Pixelbereich in </mark><mark style="background: #FFB86CA6;">kleinere Bereiche (Zellen) unterteilt, und in jeder dieser Zellen wird ein Sample zufällig gesetzt.</mark> Diese Methode kombiniert Zufall mit einer Art Ordnung und wird oft verwendet, da sie <mark style="background: #FFB86CA6;">besser gegen Rauschen</mark> hilft.
![[Pasted image 20241113100711.png#invert|300]]
### Blue Noise Sampling
- Diese Sampling-Methode <mark style="background: #FFB86CA6;">versucht, die Abstände zwischen den Samples so gleichmäßig wie möglich zu halten, aber ohne regelmäßiges Raster</mark>. Diese Verteilung hat wenig niederfrequente Elemente, was <mark style="background: #FFB86CA6;">weniger Rauschen erzeugt und gleichzeitig effektiv gegen Aliasing ist</mark>. Blue Noise Sampling wird oft bei Echtzeit-Anwendungen verwendet, weil es weniger Samples braucht und dennoch gute Ergebnisse liefert.