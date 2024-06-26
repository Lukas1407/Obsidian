#### **Funktion des Frame Buffers:**
- Ein **Frame Buffer** ist ein Speicherbereich, in dem ein vollständiges Bild gespeichert wird, bevor es auf den Bildschirm übertragen wird.
- Jeder Pixel des Bildes wird als eine Kombination von Farbwerten für Rot, Grün und Blau gespeichert. Diese Werte liegen typischerweise zwischen 0 und 255, was die Intensität der Farbe bestimmt.

#### **Beispiel für Frame Buffer:**
```c
#define WIDTH 1920
#define HEIGHT 1080

// Array: 8-Bit RGB-Frame Buffer
unsigned char buffer[ WIDTH * HEIGHT * 3 ];

for ( int y = 0; y < HEIGHT; y++ ) {
    for ( int x = 0; x < WIDTH; x++ ) {
        // Rot-Wert
        buffer[ ( x + y * WIDTH ) * 3 + 0 ] = 255;  // Beispielwert für volle Intensität
        // Grün-Wert
        buffer[ ( x + y * WIDTH ) * 3 + 1 ] = 0;    // Beispielwert für keine Intensität
        // Blau-Wert
        buffer[ ( x + y * WIDTH ) * 3 + 2 ] = 0;    // Beispielwert für keine Intensität
    }
}

// Bild auf den Bildschirm kopieren
CopyImageToScreen( buffer );
```

#### **Vorteil gegenüber Vektormonitoren:**
- Einmal im Frame Buffer gespeichert, kann das Bild unabhängig von dessen Berechnungsaufwand dargestellt werden. Dies ermöglicht eine flimmerfreie und stabile Darstellung, da das Bild kontinuierlich vom Frame Buffer gelesen und auf den Bildschirm übertragen wird.

#### **Darstellung über API-Funktionen:**
- Moderne Systeme verwenden API-Funktionen, um die Bilder aus dem Frame Buffer auf den Bildschirm zu übertragen. APIs wie OpenGL oder DirectX ermöglichen eine effiziente Handhabung und Darstellung von Bildern.
### Typische Werte für Frame Buffer
Ein **Frame Buffer** ist ein Speicherbereich, in dem die vollständigen Bilddaten gespeichert werden, die an ein Display gesendet werden sollen. Die typischen Werte und Anforderungen an Frame Buffers variieren je nach Auflösung, Farbtiefe und Bildwiederholrate. Hier sind einige Beispiele und Erklärungen:
### Beispielwerte für Frame Buffer
#### **1. Full HD (1920 x 1080) bei 60 Hz**
- **Auflösung:** 1920 × 1080 = 2,073,600 Pixel.
- **Farbtiefe:** 24 Bit (8 Bit für Rot, Grün, Blau).
- **Datenmenge pro Frame:** \( 2,073,600 \, \text{Pixel} \times 24 \, \text{Bit/Pixel} \div 8 \, \text{Bit/Byte} \approx 6 \, \text{MB} \).
- **Bandbreite bei 60 Hz:** \( 6 \, \text{MB/Frame} \times 60 \, \text{Frames/Sekunde} = 360 \, \text{MB/Sekunde} \).
#### **2. 4K-Auflösung mit HDR bei 60 Hz**
- **Auflösung:** 3840 × 2160 = 8,294,400 Pixel.
- **Farbtiefe:** 36 Bit (12 Bit pro Farbe: Rot, Grün, Blau).
- **Datenmenge pro Frame:** \( 8,294,400 \, \text{Pixel} \times 36 \, \text{Bit/Pixel} \div 8 \, \text{Bit/Byte} = 36 \, \text{MB} \).
- **Bandbreite bei 60 Hz:** \( 36 \, \text{MB/Frame} \times 60 \, \text{Frames/Sekunde} = 2.16 \, \text{GB/Sekunde} \).
### Historische Frame Buffer
#### **16-Bit Frame Buffer (HiColor)**
- **Farbtiefe:** 16 Bit insgesamt, aufgeteilt in 5 Bit für Rot, 6 Bit für Grün, 5 Bit für Blau.
- **Warum 6 Bit für Grün?** Das menschliche Auge ist besonders empfindlich für Grün, daher wurde Grün mit einem zusätzlichen Bit berücksichtigt, um eine bessere Farbwahrnehmung zu erreichen.
#### **8-Bit Frame Buffer**
- **Farbtiefe:** 8 Bit pro Pixel.
- **Farbauswahl:** 256 frei wählbare Farben aus einer Palette von 16,777,216 (24 Bit).
- **Verwendung:** War in VGA-Grafikkarten und dem GIF-Dateiformat weit verbreitet.
### Dithering und Fehlerdiffusion
#### **Dithering:**
- **Beschreibung:** Eine Technik zur Nachbildung fehlender Farben durch die Anordnung vorhandener Farben in einem bestimmten Muster. Dies erzeugt den visuellen Effekt von Mischfarben.
- **Beispiel:** Anstatt eine mittelgraue Farbe direkt darzustellen, können schwarze und weiße Pixel so angeordnet werden, dass sie zusammen ein Grau simulieren.
![[Pasted image 20240621104936.png|500]]
#### **Fehlerdiffusion:**
- **Funktionsweise:** Beim Dithering wird der Fehler (die Differenz zwischen der gewünschten Farbe und der tatsächlich dargestellten Farbe) auf die umliegenden Pixel verteilt. Dadurch entsteht ein Bild, das trotz begrenzter Farbpalette natürlicher aussieht.
### Weitere Details und Überlegungen
- **Speicherbedarf und Bandbreite:** Moderne Frame Buffer müssen große Mengen an Daten speichern und übertragen, insbesondere bei hohen Auflösungen und Bildwiederholraten. Dies erfordert schnelle Speicher und Datenbusse, um ein flimmerfreies und stabiles Bild zu gewährleisten.
- **Effiziente Datenverarbeitung:** Für hochauflösende Bilder wie 4K mit HDR erfordert die Verarbeitung und Übertragung der Bilddaten leistungsstarke Hardware, die in der Lage ist, große Datenmengen schnell zu bewältigen.
