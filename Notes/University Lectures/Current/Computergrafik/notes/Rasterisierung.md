Die Rasterisierung dient dazu, geometrische Formen (wie Dreiecke) auf einem Pixelraster darzustellen. Zwei Methoden werden erklärt:
1. **Scanline-Verfahren (links):**
    - Die Kanten des Dreiecks werden links und rechts interpoliert.
    - Zeilenweise wird zwischen den Kanten interpoliert, um zu bestimmen, welche Pixel innerhalb des Dreiecks liegen.
    - Vorteile: Effizient und einfach zu implementieren.
2. **Edge-based Verfahren (rechts):**
    - Kanten des Dreiecks werden mathematisch definiert (z. B. als Halbebenen).
    - Ein Pixel liegt innerhalb des Dreiecks, wenn es in der **positiven Halbebene** aller drei Kanten liegt.
    - Vorteile: Robuster, da es direkt mit den Kanten arbeitet.
![[Pasted image 20250122124025.png|600]]
#### **2. Clipping**
- **Definition:** Clipping bedeutet das Abschneiden von Teilen eines Polygons oder Dreiecks, die außerhalb des sichtbaren Bereichs (z. B. Bildschirm oder Kamera) liegen.
- **Ziele:**
    - Effizienz: Es wird nur der sichtbare Bereich rasterisiert.
    - Vermeidung von Fehlern bei Projektionen, insbesondere für Objekte hinter der Kamera.
- Clipping ist ein wesentlicher Schritt, um sicherzustellen, dass nur relevante Geometrie verarbeitet wird.
![[Pasted image 20250122124048.png|200]]
### Maler-Algorithmus
- Sortiert Dreiecke von hinten nach vorne und überzeichnet verdeckte Flächen.
    - Probleme:
        - Funktioniert nicht, wenn sich Dreiecke überschneiden (sichtbar in der rechten Illustration).
        - Ineffizient, da Sortierung für alle Dreiecke erforderlich ist.
- **Moderne Lösung: Z-Buffering (siehe unten).**
### Tiefenpuffer (Z-Buffering)
- **Prinzip:**
    - Jeder Pixel speichert einen Tiefenwert (Z-Wert), der die Distanz zur nächsten sichtbaren Fläche angibt.
    - Während der Rasterisierung wird geprüft, ob ein neuer Pixel näher ist als der gespeicherte Wert.
    - Wird zusätzlich zum Farbwert-Puffer verwendet.
![[Pasted image 20250122124148.png|400]]
- **Vorteile:**
    - Dreiecke können in beliebiger Reihenfolge verarbeitet werden.
    - Standard in Grafik-Hardware.
- **Nachteile:**
    - Zusätzlicher Speicherbedarf (heute weniger kritisch).
    - Probleme bei begrenzter Genauigkeit (Z-Aliasing).
    - Transparentes Rendering erfordert spezielle Techniken.