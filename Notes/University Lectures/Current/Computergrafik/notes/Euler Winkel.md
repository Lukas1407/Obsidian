#### **Definition**
- Jede 3D-Rotation kann durch **drei aufeinanderfolgende Rotationen um die Hauptachsen ($x, y, z$)** dargestellt werden.
- Reihenfolge und Achsen sind frei wählbar (z. B. $z-y-x$, bekannt als **Yaw-Pitch-Roll**).
- Euler-Winkel:
  - $\psi$: Drehung um $z$-Achse (Yaw).
  - $\theta$: Drehung um $y$-Achse (Pitch).
  - $\phi$: Drehung um $x$-Achse (Roll).

#### **Rotationsmatrix**
- Kombination der Rotationen:
  $$
  R = R_z(\psi) R_y(\theta) R_x(\phi)
  $$
  - Die Elemente der Matrix werden durch Sinus- und Kosinusfunktionen der Winkel bestimmt.

---

### **Besonderheiten der Euler-Rotation**

#### **Gimbal Lock (Kardanische Blockade)**
- Problem:
  - Wenn zwei Rotationsachsen (z. B. $x$ und $z$) übereinstimmen, verliert das System einen Freiheitsgrad.
  - Beispiel: $Pitch = \pm 90^\circ$, dann sind Yaw und Roll nicht mehr eindeutig definiert.
- Lösung:
  - Verwendung von **Quaternions** oder anderen Darstellungen, die Gimbal Lock vermeiden.
