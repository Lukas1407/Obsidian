## Eigenschaften
### Konvexe Hülle Eigenschaft
- Eine Bézier-Kurve F(u) liegt immer in der konvexen Hülle der Kontrollpunkte $\{b_i\}$.
- Dadurch ist F(u) eine **Konvexkombination** der Kontrollpunkte
### Endpunktinterpolation
- Die Bézier-Kurve geht immer durch ihre ersten und letzten Kontrollpunkte
### Tangentenbedingung
- Die Richtung der ersten beiden bzw. letzten beiden Kontrollpunkte gibt die Tangente an.
### Affine Invarianz
- Eine affine Transformation (z.B. Skalierung, Rotation, Translation) kann direkt auf die Kontrollpunkte angewendet werden
### Variationsreduzierung
- Wenn eine Gerade H mit der Bézier-Kurve F schneidet, dann ist die Anzahl der Schnittpunkte maximal so groß wie die Anzahl der Schnittpunkte mit dem Kontrollpolygon
![[Pasted image 20250128090308.png#invert|700]]

## Casteljau-Algorithmus
![[Pasted image 20250128092325.png#invert|400]]
Die Formel für den de Casteljau-Algorithmus ist:
$$
b_i^k = (1 - u) b_i^{k-1} + u b_{i+1}^{k-1}
$$
### Beispiel
Die Kontrollpunkte der Bézier-Kurve sind:
$$
b_0^0 = (8,9), \quad b_1^0 = (4,-7), \quad b_2^0 = (0,-7), \quad b_3^0 = (-4,9)
$$
Einsetzen von $u = \frac{1}{4}$:
1. **Berechnung von $b_0^1$:**
   $$
   b_0^1 = (1 - \frac{1}{4}) b_0^0 + \frac{1}{4} b_1^0
   $$
   $$
   = \frac{3}{4} (8,9) + \frac{1}{4} (4,-7)
   $$
   $$
   = \left(\frac{3}{4} \cdot 8 + \frac{1}{4} \cdot 4, \frac{3}{4} \cdot 9 + \frac{1}{4} \cdot (-7) \right)
   $$
   $$
   = \left(\frac{24}{4} + \frac{4}{4}, \frac{27}{4} - \frac{7}{4} \right) = \left(\frac{28}{4}, \frac{20}{4}\right) = (7,5)
   $$
### Tangentenrichtung
$$
F'(u) = (1 - u) b_0^2 + u b_1^2
$$