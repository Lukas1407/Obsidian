> [!abstract] Definition
> Rasterbilder sind <mark style="background: #FFB86CA6;">Bilder, die aus einem 2D Pixel Array bestehen</mark>.

## Konsequenzen von Rasterbildern:
### 1. **Auflösung:**
Die **Auflösung** eines Bildes gibt an, <mark style="background: #FFB86CA6;">wie viele Pixel es in der Höhe und Breite hat</mark>. Eine höhere Auflösung bedeutet mehr Pixel und damit detailliertere Bilder.
- **Beispiel:** Ein Bild mit 1920x1080 Pixeln hat eine höhere Auflösung als ein Bild mit 800x600 Pixeln.
#### **Konsequenzen der Auflösung:**
- **Detailgrad:** <mark style="background: #FFB86CA6;">Höhere Auflösungen ermöglichen es, feinere Details im Bild darzustellen</mark>. Wenn du ein Bild vergrößerst, das eine niedrige Auflösung hat, wirst du bemerken, dass es pixelig und unscharf wird.
- **Dateigröße:** Bilder mit höherer Auflösung benötigen <mark style="background: #FFB86CA6;">mehr Speicherplatz</mark>, da sie mehr Pixel enthalten, die gespeichert werden müssen.
### 2. **Aliasing:**
**Aliasing** tritt auf, wenn <mark style="background: #FFB86CA6;">glatte Kanten in einem Bild erscheinen „gestuft“ oder „gezackt</mark>“. Dies geschieht, <mark style="background: #FFB86CA6;">weil die begrenzte Anzahl von Pixeln nicht ausreicht, um feine Details genau darzustellen</mark>.
- **Beispiel:** Stell dir vor, du versuchst, eine diagonale Linie auf einem Gitter von Quadraten zu zeichnen. Da die Linie nicht genau entlang der Kanten der Quadrate verläuft, entsteht ein stufenförmiges Aussehen.
#### **Konsequenzen von Aliasing:**
- **Unschärfe und Verzerrung:** <mark style="background: #FFB86CA6;">Feine Details und glatte Kanten können verzerrt oder unscharf wirken</mark>. Besonders bei der Darstellung von Schrift oder diagonalen Linien wird Aliasing deutlich sichtbar.
- **Kantenglättung:** Um Aliasing zu reduzieren, verwendet man Techniken wie die Kantenglättung (<mark style="background: #FFB86CA6;">Anti-Aliasing</mark>), die versuchen, die Kanten weicher zu machen und die „Zacken“ zu minimieren.
## Charakteristika
- **Bildaufbau:** Der wiederholte Bildaufbau ist unabhängig von der Komplexität der Szene, da jedes Bild in Form von Pixeln gespeichert und dargestellt wird.
- **Verwendung:** Typischerweise für ausgefüllte, schattierte Flächen wie in Fotografien und komplexen Grafiken.
- **Beispiele:** JPEG, PNG, BMP.
- **Probleme:** Endliche Anzahl von Pixeln kann zu „Aliasing“-Effekten führen, wo Kanten gezackt erscheinen, und zu Moiré-Effekten, wo feine Muster stören.
## Farbtiefe
Die **Farbtiefe** eines Bildes gibt an, wie <mark style="background: #FFB86CA6;">viele Bits verwendet werden, um die Farbe eines jeden Pixels zu repräsentieren</mark>. Je höher die Farbtiefe, desto mehr Farben können dargestellt werden.
![[Pasted image 20240621104752.png|500]]