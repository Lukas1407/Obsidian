- Die Basiliarmembran (BM) zeigt eine zunehmende Breite und abnehmende Dicke von der Basis bis zur Spitze (Apsis) der Cochlea.
- Sie ist empfindlich für hohe Frequenzen an der Basis und für tiefe Frequenzen an der Apsis.
- Die Schallausbreitung erfolgt über Wanderwellen, wobei Laufzeitunterschiede für hohe und tiefe Frequenzen bestehen.
- Es gibt einen aktiven, nichtlinearen Verstärkungsmechanismus für niedrige Schallpegel.
## Dynamik der Basiliarmembran: Das Helmholtz-Modell
- **Oszillatorbank-Ansatz**:
    - Die BM kann als eine Bank von harmonischen Oszillatoren betrachtet werden.
    - Diese Oszillatoren sind entlang der Länge der BM angeordnet und reagieren auf verschiedene Frequenzen.
    - Die Frequenzen der Oszillatoren nehmen von der Basis (nahe dem ovalen Fenster) zum Apex (nahe dem runden Fenster) exponentiell ab.
- **Elementare Differenzialgleichung des getriebenen, harmonischen Oszillators**:
    - Die Bewegung eines einzelnen Oszillators wird durch die folgende Differenzialgleichung beschrieben:$$m \frac{d^2x}{dt^2} + h \frac{dx}{dt} + kx = f(t)$$
    - Hierbei sind:
        - (x) die Auslenkung des Oszillators.
        - (m) die Oszillatormasse (entspricht dem Flüssigkeitsvolumen der BM).
        - (h) die Reibung (Viskosität).
        - (k) die Federkraft (Steifigkeit der BM).
        - (f(t)) die antreibende, externe Kraft.
- **Erweiterung auf eine Oszillatorbank**:
    - Die BM besteht aus einer Vielzahl solcher Oszillatoren.
    - Wir erweitern die Differenzialgleichung auf ein Set von (N) Oszillatoren:$$m_i \frac{d^2x_i}{dt^2} + h_i \frac{dx_i}{dt} + k_i x_i = f_i(t) \quad \text{für } i = 1, 2, \ldots, N $$
    - Der Index (i) wird so gewählt, dass die Frequenzauflösung und -bandbreite der Cochlea berücksichtigt werden.
- **Antriebskraft und hydrodynamische Kopplung**:
    - Die Antriebskraft für die Oszillatorbank wird durch die Beschleunigung (a_s(t)) der Wassermasse über die Stapesbewegung erzeugt.
    - Diese Antriebskraft wird über die hydrodynamische Kopplung (G_i) auf die Cochlea übertragen.
## Dynamit der Basiliarmembran: Wanderwellen-Modell nach Békésy
Georg von Békésy entdeckte durch seine Experimente, dass sich Schallwellen in der Cochlea als sogenannte Wanderwellen ausbreiten. Hier sind die Kernpunkte dieses Modells:
1. **Hydrodynamische Interaktion**:
    - Békésy beobachtete, dass benachbarte Bereiche der Cochlea miteinander interagieren, was zu einer komplexen Bewegung der BM führt.
    - Diese Bewegung wird als Wanderwelle bezeichnet, die entlang der BM läuft.
2. **Druckunterschiede**:
    - Die Auslenkung der BM wird durch den Druckunterschied zwischen der Scala vestibuli und der Scala tympani verursacht.
    - Wenn die Flussgeschwindigkeiten gering sind und die Flüssigkeit als inkompressibel angenommen wird, kann die Viskosität vernachlässigt werden.
    - Druckänderungen breiten sich unter diesen Annahmen instantan aus, und das Druckfeld ist proportional zur lokalen Flüssigkeitsbeschleunigung.
3. **Vereinfachte Euler-Gleichung**:
    - Unter den oben genannten Annahmen vereinfacht sich die Euler-Gleichung der Strömungsmechanik.
    - Die auf die BM wirkenden hydrodynamischen Kräfte hängen hauptsächlich von der Masseträgheit ab und weniger von der Viskosität.
4. **Green’sche Funktion**:
    - Die Lösung der Differenzialgleichung für die Bewegung der BM wird über die Green’sche Funktion ermittelt.
    - ( G_s(x) ) ist der hydrodynamische Kopplungsfaktor, der die Kraft beschreibt, die an einem Ort ( x ) der Cochlea aufgrund der Stapesbeschleunigung ( a_s ) entsteht.
    - ( G(x’, x) ) ist die Kraft, die an einem Ort ( x ) aufgrund der Beschleunigung der BM an einem anderen Ort ( x’ ) entsteht.