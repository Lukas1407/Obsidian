
### Prinzip der Darstellung:
- Ein **Röhrenmonitor** (auch als CRT, Cathode Ray Tube, bekannt) <mark style="background: #FFB86CA6;">baut das Bild zeilenweise auf, indem ein Elektronenstrahl über die Bildschirmfläche geführt wird</mark>.
![[Pasted image 20241028081327.png#invert|200]]
- Der Elektronenstrahl bewegt sich <mark style="background: #FFB86CA6;">von links nach rechts und von oben nach unten</mark>, um jeden Pixel der Bildschirmfläche zu treffen. Dabei wird die <mark style="background: #FFB86CA6;">Helligkeit und Farbe jedes Pixels durch die Intensität und die Farbe des Elektronenstrahls</mark> bestimmt.
### Zeilenweises Aufbauen:
- Der Strahl beginnt oben links und bewegt sich schnell von links nach rechts über die erste Zeile, dann zurück zur linken Seite der nächsten Zeile, und so weiter, bis das gesamte Bild aufgebaut ist.
- Das Bild muss schnell genug aktualisiert werden (typisch mehr als 50 Hz), um ein Flimmern zu vermeiden.
### Flimmerfreie Darstellung:
- <mark style="background: #FFB86CA6;">Um flimmerfrei zu sein, muss das Bild mindestens 50-mal pro Sekunde (50 Hz) neu gezeichnet werden</mark>. Moderne Monitore arbeiten oft mit 60 Hz oder mehr, um eine stabile Darstellung zu gewährleisten.
### Elektronenstrahl und Helligkeit:
- Die Intensität des Elektronenstrahls bestimmt die Helligkeit des getroffenen Pixels. Die Farben werden durch unterschiedliche Intensitäten der Elektronen für die roten, grünen und blauen Pixel bestimmt.
### Bildspeicherung:
- Ein Bild wird als 2D-Array von Pixeln gespeichert, wobei jeder Pixel seine Farbe durch drei Werte für Rot, Grün und Blau repräsentiert.
