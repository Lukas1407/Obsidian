> [!abstract] Definition
> Rasterbilder sind Bilder, die aus einem Raster von kleinen, farbigen Punkten bestehen, die man als **Pixel** (Picture Elements) bezeichnet. Diese Pixel sind in einem 2D-Array organisiert, das man sich wie ein Gitter vorstellen kann. Jeder Pixel trägt eine bestimmte Farbe. 
Stell dir vor, du hast ein Bild, das aus 10x10 Pixeln besteht. Das bedeutet, es gibt 100 kleine Quadrate, die zusammen das Bild formen. Jede dieser Quadrate hat eine bestimmte Farbe.

## Konsequenzen von Rasterbildern:
### 1. **Auflösung:**
Die **Auflösung** eines Bildes gibt an, wie viele Pixel es in der Höhe und Breite hat. Eine höhere Auflösung bedeutet mehr Pixel und damit detailliertere Bilder.
- **Beispiel:** Ein Bild mit 1920x1080 Pixeln hat eine höhere Auflösung als ein Bild mit 800x600 Pixeln.
#### **Konsequenzen der Auflösung:**
- **Detailgrad:** Höhere Auflösungen ermöglichen es, feinere Details im Bild darzustellen. Wenn du ein Bild vergrößerst, das eine niedrige Auflösung hat, wirst du bemerken, dass es pixelig und unscharf wird.
- **Dateigröße:** Bilder mit höherer Auflösung benötigen mehr Speicherplatz, da sie mehr Pixel enthalten, die gespeichert werden müssen.
### 2. **Aliasing:**
**Aliasing** tritt auf, wenn glatte Kanten oder feine Details in einem Bild erscheinen „gestuft“ oder „gezackt“. Dies geschieht, weil die begrenzte Anzahl von Pixeln nicht ausreicht, um feine Details genau darzustellen.
- **Beispiel:** Stell dir vor, du versuchst, eine diagonale Linie auf einem Gitter von Quadraten zu zeichnen. Da die Linie nicht genau entlang der Kanten der Quadrate verläuft, entsteht ein stufenförmiges Aussehen.
#### **Konsequenzen von Aliasing:**
- **Unschärfe und Verzerrung:** Feine Details und glatte Kanten können verzerrt oder unscharf wirken. Besonders bei der Darstellung von Schrift oder diagonalen Linien wird Aliasing deutlich sichtbar.
- **Kantenglättung:** Um Aliasing zu reduzieren, verwendet man Techniken wie die Kantenglättung (Anti-Aliasing), die versuchen, die Kanten weicher zu machen und die „Zacken“ zu minimieren.
## Charakteristika
- **Bildaufbau:** Der wiederholte Bildaufbau ist unabhängig von der Komplexität der Szene, da jedes Bild in Form von Pixeln gespeichert und dargestellt wird.
- **Verwendung:** Typischerweise für ausgefüllte, schattierte Flächen wie in Fotografien und komplexen Grafiken.
- **Beispiele:** JPEG, PNG, BMP.
- **Probleme:** Endliche Anzahl von Pixeln kann zu „Aliasing“-Effekten führen, wo Kanten gezackt erscheinen, und zu Moiré-Effekten, wo feine Muster stören.
## Farbtiefe
Die **Farbtiefe** eines Bildes gibt an, wie viele Bits verwendet werden, um die Farbe eines jeden Pixels zu repräsentieren. Je höher die Farbtiefe, desto mehr Farben können dargestellt werden.
![[Pasted image 20240621104752.png|500]]