![[Pasted image 20241126095525.png#invert|300]]
Path Tracing ist ein Rendering-Verfahren, das dem Distributed Raytracing ähnlich ist, jedoch folgende Unterschiede aufweist:
1. **Verfahren**:
    - Pro Pixel werden mehrere Pfade verfolgt, indem ein einziger Strahl aus der Kamera herausgeschickt wird.
    - Bei jedem Schnittpunkt wird nur ein zufällig ausgewählter weiterer Strahl verfolgt.
2. **Sampling**:
    - Nah an der Kamera werden mehr Richtungen abgetastet, was zu detaillierten und realistischen Ergebnissen in diesen Bereichen führt.
    - Weiter entfernte Flächen erhalten weniger Abtastung, was für die meisten Szenen sinnvoll ist, da entfernte Details für den Betrachter oft weniger relevant sind.
3. **Vorteile**:
    - Realistische Darstellung von Effekten wie Global Illumination (z. B. indirekte Beleuchtung) und weiche Schatten.
    - Effizient für komplexe Szenen mit vielen Lichtquellen und reflektierenden Oberflächen.
4. **Nachteil**:
    - Hoher Rechenaufwand, da für jede Pixelrichtung viele Strahlenverfolgungen erforderlich sind.
