Die Folien erklären das Thema Rasterisierung in der Computergrafik, welches die Umwandlung von geometrischen Objekten in diskrete Pixel zur Darstellung auf einem Bildschirm umfasst. Ich werde die wichtigen Punkte zusammenfassen und erklären:

---

### **1. Weber-Fechner-Gesetz**
- Das Weber-Fechner-Gesetz beschreibt, wie Sinneseindrücke logarithmisch zur physikalischen Intensität wahrgenommen werden. 
- In der Computergrafik ist das besonders bei der Darstellung von Helligkeitsunterschieden wichtig. Die minimal wahrnehmbaren Unterschiede (JND) liegen bei etwa 1–2 % der Hintergrundhelligkeit.

---

### **2. Rasterisierung von Linien**
![[Pasted image 20241203152744.png#invert|300]]
![[Pasted image 20241203152754.png#invert|300]]
#### **Grundprinzip**
- Ziel: Eine Linie, definiert durch ihre Endpunkte $(p_x, p_y)$ und $(q_x, q_y)$, wird in eine Menge diskreter Pixel übersetzt, die den Verlauf möglichst genau darstellen.
- Herausforderung: Linien müssen ohne Lücken und so effizient wie möglich gezeichnet werden.
#### **Liniengleichung**
Die Grundform der Linie ist gegeben durch:
$$
y = mx + c
$$
- $m = \frac{q_y - p_y}{q_x - p_x}$: Steigung der Linie.
- $c = p_y - m p_x = \frac{p_y q_x - q_y p_x}{q_x - p_x}$: Y-Achsenabschnitt.

##### **Beispiele**
1. Linie von $(0, 0)$ nach $(6, 6)$:
   - $m = \frac{6 - 0}{6 - 0} = 1$
   - $c = 0 - 1 \cdot 0 = 0$
   - Gleichung: $y = x$

2. Linie von $(0, 0)$ nach $(8, 4)$:
   - $m = \frac{4 - 0}{8 - 0} = 0.5$
   - $c = 0 - 0.5 \cdot 0 = 0$
   - Gleichung: $y = 0.5x$
#### **Brute-Force-Algorithmus**
- Setzt für jede $x$-Koordinate den zugehörigen $y$-Wert nach der Geradengleichung:
```cpp
float m = (qy - py) / (qx - px);
float c = (py * qx - qy * px) / (qx - px);

for (int x = px; x <= qx; x++) {
    float y = m * x + c;
    set_pixel(x, round(y));
}
```
- Nachteile:
  - Drei Gleitkommaoperationen pro Pixel: Multiplikation, Addition und Rundung.
  - Nicht effizient.
#### **Verbesserung: Inkrementelle Berechnung**
- Nutzen der Beziehung:
$$
y_{n+1} = y_n + m
$$
- Algorithmus reduziert die Gleitkommaoperationen auf Addition und Rundung:
```cpp
float m = (qy - py) / (qx - px);
float c = (py * qx - qy * px) / (qx - px);

float y = m * px + c;

for (int x = px; x <= qx; x++) {
    set_pixel(x, round(y));
    y += m;
}
```
#### **Implizite Darstellung**
- Linie wird durch eine Funktion $F(x, y)$ beschrieben:
$$
F(x, y) = y - p_y - m(x - p_x)
$$
- Eigenschaften:
  - $F(x, y) > 0$: Punkt liegt oberhalb der Linie.
  - $F(x, y) < 0$: Punkt liegt unterhalb der Linie.
  - $F(x, y) = 0$: Punkt liegt exakt auf der Linie.

##### **Entscheidung anhand der Pixelmitte**
- Für den Pixel in der Mitte $(x + 1, y + 0.5)$ zwischen den Kandidaten (E, NE):
  - $F(x+1, y+0.5) < 0$: Wähle NE.
  - $F(x+1, y+0.5) \geq 0$: Wähle E.
#### **Bresenham-Algorithmus**
- Verbesserter Algorithmus, der nur mit Ganzzahlen arbeitet.
- Entscheidungsvariable $d$:
  - Initialisierung: 
$$
d = F(p_x + 1, p_y + 0.5)
$$
  - Updates:
    - Für $d < 0$ (NE):
$$
d = d + 2\Delta y - 2\Delta x
$$
    - Für $d \geq 0$ (E):
$$
d = d + 2\Delta y
$$
- Algorithmus:
```cpp
int y = py;
int d = 2 * (qy - py) - (qx - px); // Initialisierung

for (int x = px; x <= qx; x++) {
    set_pixel(x, y);
    if (d < 0) { // Gehe nach NE
        y += 1;
        d += 2 * (qy - py) - 2 * (qx - px);
    } else {     // Gehe nach E
        d += 2 * (qy - py);
    }
}
```

---

#### **Antialiasing**
- Problem: Original-Bresenham setzt Pixel binär (gesetzt/nicht gesetzt), was zu sichtbaren Kanten führt (Treppeneffekt).
- Lösung: Berücksichtige die Intensität:
  - Berechne den (vorzeichenbehafteten) Abstand $a$ der Linie zum Mittelpunkt des Pixels.
  - Setze Pixelintensitäten proportional zum Abstand.
---

### **4. Rasterisierung von Polygonen**
#### **Ziele**
- Gegeben: Ein Polygon (eine geschlossene Kantenmenge) in 2D.
- Ziel: Alle Pixel innerhalb des Polygons korrekt einfärben.

#### **Ansätze**
1. **Brute-Force-Test**: Testet für jedes Pixel, ob es innerhalb des Polygons liegt (aufwendig).
2. **Scanline-Algorithmus**: Bearbeitet das Polygon Zeile für Zeile. Dabei werden Schnittpunkte der Scanline mit den Polygonkanten berechnet, um die zu setzenden Pixel zu bestimmen.

#### **Konsistenzregeln**
- Regeln legen fest, wie Pixel an Kanten oder Eckpunkten behandelt werden, um Überlappungen oder Lücken bei benachbarten Polygonen zu vermeiden.

---

### **5. Sichtbarkeitsproblem**
- Bei mehreren Polygonen in einer Szene muss entschieden werden, welches Polygon sichtbar ist.
- **Maler-Algorithmus**: Sortiert Polygone nach ihrer Entfernung zur Kamera und zeichnet sie in dieser Reihenfolge. Probleme treten bei Zyklen und falschen Sortierkriterien auf.
- **Z-Puffer-Verfahren**: Speichert die Tiefenwerte jedes Pixels, um das nächstgelegene Polygon korrekt darzustellen.

---

Wenn du einen bestimmten Aspekt genauer verstehen möchtest, lass es mich wissen!

