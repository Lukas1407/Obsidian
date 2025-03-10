Um die Darstellung von Bildern und die Umwandlung von RGB-Werten in sichtbare Helligkeit besser zu verstehen, müssen wir uns die Transferfunktion und die physikalischen Eigenschaften des Displays anschauen. Diese bestimmen, <mark style="background: #FFB86CA6;">wie die digitalen Farbwerte in wahrgenommene Helligkeit umgewandelt werden.</mark>
### Transferfunktion für Helligkeit
#### Was ist eine Transferfunktion?
<mark style="background: #FFB86CA6;">Eine Transferfunktion beschreibt die Beziehung zwischen den Eingabewerten (z.B. den RGB-Werten) und der resultierenden Helligkeit auf einem Display</mark>. Diese Funktion ist <mark style="background: #FFB86CA6;">oft nicht linear, da das menschliche Auge Licht und Farben nicht linear wahrnimmt</mark>.
- **Mathematisch:** Die Transferfunktion $T$ wandelt einen digitalen Wert $x$ (z.B. zwischen 0 und 255) in einen Helligkeitswert $L$ um, <mark style="background: #FFB86CA6;">der zwischen einem minimalen und einem maximalen Helligkeitswert liegt</mark>.
  $$
  T: [0, x_{\text{max}}] \to [L_{\text{min}}, L_{\text{max}}]
  $$
- **Beispiel:** Für ein Graustufenbild könnte $x_{\text{max}} = 255$ sein, wobei $0$ für Schwarz und $255$ für Weiß steht.
### Helligkeit und RGB-Werte
<mark style="background: #FFB86CA6;">Höhere RGB-Werte bedeuten hellere Farben</mark>, aber wie genau diese Werte auf dem Display erscheinen, hängt von der Transferfunktion und den Eigenschaften des Displays ab.
- **RGB-Werte:** Jeder Pixelwert in einem Bild ist eine Kombination aus Rot-, Grün- und Blau-Werten. Höhere Werte für jeden dieser Kanäle ergeben eine hellere Farbe.
- **Helligkeit:** Die wahrgenommene Helligkeit hängt davon ab, wie die Werte in Lichtintensität umgewandelt werden.
### Transferfunktion
#### Nichtlineare Transferfunktion
- Die Umwandlung von digitalen Werten in Helligkeit ist häufig nicht linear, weil das menschliche Auge <mark style="background: #FFB86CA6;">Unterschiede in der Helligkeit in dunklen Bereichen stärker wahrnimmt als in hellen Bereichen</mark>.
- Eine <mark style="background: #FFB86CA6;">gängige Form der Transferfunktion ist die Gamma-Korrektur</mark>, die eine Potenzfunktion verwendet:
  $$
  L = T(x) = \left( \frac{x}{x_{\text{max}}} \right)^\gamma
  $$

  Hierbei ist $\gamma$ der Gamma-Wert, der <mark style="background: #FFB86CA6;">typischerweise bei 2.2</mark> liegt. Diese Funktion <mark style="background: #FFB86CA6;">sorgt dafür, dass die Helligkeit in einer Weise angepasst wird, die der menschlichen Wahrnehmung entspricht</mark>.
#### Lineare Transferfunktion
- <mark style="background: #FFB86CA6;">Für technische Anwendungen oder HDR-Displays wird oft eine lineare Transferfunktion verwendet</mark>, bei der die Helligkeit direkt proportional zum RGB-Wert ist:
  $$
  L = \frac{x}{x_{\text{max}}} \times L_{\text{max}}
  $$
![[Pasted image 20240622083309.png|400]]
![[Pasted image 20240622083321.png|400]]
![[Pasted image 20240622083334.png|400]]
### Physikalische Eigenschaften des Displays
Die <mark style="background: #FFB86CA6;">Transferfunktion hängt stark von den physikalischen Eigenschaften des Displays</mark> ab, wie z.B.:
- <mark style="background: #FFB86CA6;">Leuchtdichte: Dies ist die Lichtmenge, die das Display pro Flächeneinheit emittiert</mark>.
- <mark style="background: #FFB86CA6;">Kontrastverhältnis: Dies beschreibt das Verhältnis zwischen der maximalen und minimalen Leuchtdichte</mark>.
- <mark style="background: #FFB86CA6;">Farbwiedergabe</mark>: Die Fähigkeit des Displays, verschiedene Farben genau darzustellen.
### Gewünschte Darstellungscharakteristika
Für die gewünschte Darstellung gibt es mehrere wichtige Aspekte:
- <mark style="background: #FFB86CA6;">Natürliche Farbwiedergabe: Das Display sollte Farben so darstellen, dass sie der menschlichen Wahrnehmung entsprechen</mark>.
- <mark style="background: #FFB86CA6;">Hoher Kontrast</mark>: Ein guter Kontrast hilft dabei, <mark style="background: #FFB86CA6;">Details in Bildern klar zu erkennen</mark>.
- **Glatte Übergänge:** Die Transferfunktion sollte so gestaltet sein, dass <mark style="background: #FFB86CA6;">Übergänge zwischen unterschiedlichen Helligkeiten und Farben glatt und natürlich wirken</mark>.
### Transferfunktion für Graustufenbilder
Bei einem **Graustufenbild** handelt es sich um eine spezielle Form der Farbdarstellung, bei der jeder Pixel nur einen Wert für die Helligkeit hat:
- **0** entspricht Schwarz.
- **255** (oder der Maximalwert in einem anderen System) entspricht Weiß.
Die <mark style="background: #FFB86CA6;">Transferfunktion für Graustufenbilder</mark> kann wie folgt aussehen:
$$
T(x) = \left( \frac{x}{x_{\text{max}}} \right)^\gamma
$$
Hierbei ist $x_{\text{max}}$ der maximale Wert für den Grauwert (z.B. 255), und $\gamma$ ist der Gamma-Wert, der die Nichtlinearität der Helligkeitswahrnehmung kompensiert.
### Transferfunktion für Farbbilder
Für Farbbilder wird die <mark style="background: #FFB86CA6;">Transferfunktion für jeden Farbkanal (Rot, Grün, Blau) separat angewendet</mark>:
$$
L_{\text{rot}} = T_{\text{rot}}(x_{\text{rot}}), \quad L_{\text{grün}} = T_{\text{grün}}(x_{\text{grün}}), \quad L_{\text{blau}} = T_{\text{blau}}(x_{\text{blau}})
$$
<mark style="background: #FFB86CA6;">Jeder dieser Kanäle wird dann kombiniert, um die endgültige Farbe des Pixels zu erzeugen</mark>.

### Transferfunktion und ihre Einschränkungen
#### Maximale Displayhelligkeit $L_{\text{max}}$
**Maximale Displayhelligkeit** ist die <mark style="background: #FFB86CA6;">höchste Helligkeit, die ein Display erzeugen kann</mark>, typischerweise gemessen in $\text{cd/m}^2$ (Candela pro Quadratmeter) oder $\text{W/m}^2$ (Watt pro Quadratmeter).
- **Einflussfaktoren:**
  - **LCDs:** Ein einfaches LCD-Display kann nur weniger als 10% der Helligkeit der Hintergrundbeleuchtung nutzen. Die restlichen 90% gehen durch Filter und die Polarisierung verloren.
  - **Projektoren:** Die Helligkeit hängt von der Leistung der Lampe, den Filtern, der Optik und der Größe der Projektionsfläche ab. Ein größerer Bildschirm verteilt das Licht und verringert die Helligkeit pro Flächeneinheit.
**Beispielwerte:**
- **Einfache LCDs:** < 10% der Hintergrundbeleuchtung.
- **Moderne LCDs:** 300 bis 600 $\text{cd/m}^2$.
- **HDR-Displays:** Über 1000 $\text{cd/m}^2$.
- **Projektoren:** Typische Helligkeit liegt bei 1000 bis 3000 Lumen, abhängig von der Projektionsgröße.
#### Minimale Displayhelligkeit $L_{\text{min}}$
**Minimale Displayhelligkeit** ist die <mark style="background: #FFB86CA6;">geringste Lichtmenge, die ein Display abgibt, wenn ein Pixel schwarz ist</mark>.
- **Einflussfaktoren:**
  - **CRT-Monitore:** Streuung von Elektronen kann dazu führen, dass auch schwarze Pixel eine geringe Helligkeit haben.
  - **LCDs:** Die Qualität der Polarisation und die Fähigkeit, Licht vollständig zu blockieren, beeinflussen, wie dunkel ein Pixel erscheinen kann. Bei einfachen LCDs wird oft etwas Licht durchgelassen, selbst bei schwarzen Pixeln.
**Beispielwerte:**
- **Gute LCDs:** Um 0.1 $\text{cd/m}^2$.
- **OLEDs:** Praktisch 0 $\text{cd/m}^2$, da sie das Licht einzelner Pixel komplett abschalten können.
- **Projektoren:** Oft höher aufgrund des Restlichts und der Projektionstechnik.
#### Umgebungslicht $L_{\text{ambient}}$
Das **Umgebungslicht** ist das Licht, das <mark style="background: #FFB86CA6;">von der Umgebung des Displays reflektiert wird und die wahrgenommene Helligkeit und den Kontrast beeinflusst</mark>.
- **Einfluss:** <mark style="background: #FFB86CA6;">Helles Umgebungslicht kann die Dunkelheit und den Kontrast eines Displays stark reduzieren</mark>. Es wird oft als ein Prozentsatz der maximalen Displayhelligkeit angegeben.
- **Beispielwert:** Ein typischer Wert in Büroumgebungen ist, dass 5% des Umgebungslichts von der maximalen Displayhelligkeit reflektiert werden.
**Maßnahmen zur Reduktion:**
- **Dunkle Wände/Decken:** <mark style="background: #FFB86CA6;">In Kinos werden diese verwendet</mark>, um Reflexionen zu minimieren.
- <mark style="background: #FFB86CA6;">Antireflexbeschichtungen</mark>: Reduzieren die Menge des reflektierten Umgebungslichts.
### Dynamikumfang
Der <mark style="background: #FFB86CA6;">Dynamikumfang beschreibt das Verhältnis der maximalen zur minimalen Helligkeit, die ein Display darstellen kann</mark>. Er wird oft als Kontrastverhältnis angegeben:
$$
D = \frac{L_{\text{max}} + L_{\text{ambient}}}{L_{\text{min}} + L_{\text{ambient}}}
$$
- **Beispielwerte:**
  - **Einfaches LCD unter schlechten/guten Bedingungen:** 20:1 bis 100:1.
  - **Fotodruck:** 30:1 bis 80:1.
  - **Film (Negative direkt betrachtet):** 1000:1.
  - **High Dynamic Range (HDR) Displays:** Über 10000:1.
**Höherer Dynamikumfang:**
- **Vorteile:** Bessere Darstellung von Details in dunklen und hellen Bereichen.
- **Nachteile:** Mit geringer Farbtiefe können sichtbare Abstufungen in glatten Bildbereichen auftreten, da die verfügbare Anzahl an Farbnuancen nicht ausreicht, um die Helligkeitsunterschiede glatt darzustellen.
## Gammakorrektur
Gamma-Korrektur ist ein wichtiger Prozess in der Bilddarstellung, der <mark style="background: #FFB86CA6;">sicherstellt, dass die Helligkeit von Bildern auf einem Display korrekt wiedergegeben wird</mark>. Lassen uns dies in einfache Schritte aufteilen, um es besser zu verstehen.
###  Was ist Gamma-Korrektur?
Gamma-Korrektur ist eine Technik, die verwendet wird, <mark style="background: #FFB86CA6;">um die Nichtlinearität in der Helligkeitseinstellung von Displays zu korrigieren</mark>. Sie sorgt dafür, dass <mark style="background: #FFB86CA6;">die wahrgenommene Helligkeit eines Bildes auf dem Bildschirm mit der Helligkeit des Bildes übereinstimmt</mark>, die <mark style="background: #FFB86CA6;">in einem linearen Farb- oder Helligkeitsraum berechnet wurde</mark>.
Die Gamma-Korrektur ist ein wichtiger Aspekt bei der Darstellung von Bildern auf einem Monitor. Schauen wir uns das Bild und den dazugehörigen Text genauer an, um die Konzepte besser zu verstehen.
![[Pasted image 20240622084241.png#invert|500]]
1. **Ohne Gamma-Korrektur:**
   - **Pixel-Intensität:** <mark style="background: #FFB86CA6;">Ohne Korrektur wird die Helligkeit der Pixel linear interpretiert, was bedeutet, dass eine Verdoppelung des Pixelwerts auch eine Verdoppelung der Helligkeit zur Folge hat</mark>.
   - **Anzeigekurve:** Dies wird durch eine gerade Linie in der Grafik dargestellt, die jedoch zu einer ungleichmäßigen Helligkeitsverteilung führt.
2. **Mit Gamma-Korrektur:**
   - **Technische Gründe:** CRT-Displays (Kathodenstrahlröhren) haben eine nichtlineare Helligkeitswiedergabe, die sich proportional zum Quadrat der Spannung verhält (Intensität ∝ (Volt)²). Diese Nichtlinearität wird durch den Gamma-Wert \(\gamma\) charakterisiert.
   - **Korrektur:** Um die nichtlineare Darstellung zu kompensieren, wird die Gamma-Korrektur angewendet. Sie passt die Pixelwerte so an, dass die resultierende Helligkeit linear zur ursprünglichen Bildintensität ist.
3. **Falsche Korrektur:**
   - **Korrektur in die falsche Richtung:** Hierbei wird die Korrektur falsch angewendet, was zu einer weiteren Verfälschung der Helligkeit führt. Dies wird durch eine falsch gerichtete Kurve in der Grafik dargestellt.
4. **Zu starke Korrektur:**
   - **Überkompensation:** Eine zu starke Korrektur überkompensiert die Helligkeit, was zu einer unnatürlichen Darstellung führt, bei der die Helligkeit zu stark angehoben wird.
![[Pasted image 20240622084331.png|500]]
### Was passiert im Display?
Ein Monitor stellt einen Pixelwert $p$, für $p\in[0,1]$ auf die Intensität $I(p)$ dar. Diese Intensität folgt oft einer <mark style="background: #FFB86CA6;">Potenzfunktion</mark>:
$$I(n) \propto p^\gamma$$
- **Gamma-Wert $\gamma$:** Dieser Wert beschreibt das nichtlineare Verhalten des Displays.
- **Lineare Darstellung:** In der Computergrafik werden Pixelwerte oft in einem linearen Raum berechnet, um eine proportionale Helligkeitswiedergabe zu erzielen („doppelte berechnete Beleuchtung, doppelte Emission auf Display“).
### Warum ist das wichtig?
Um eine korrekte und natürliche Darstellung von Bildern auf verschiedenen Displays zu gewährleisten, muss das nichtlineare Verhalten des Displays kompensiert werden. Dies wird durch die Gamma-Korrektur erreicht, die sicherstellt, dass die Helligkeit proportional zur ursprünglichen Bildinformation bleibt.
#### Gamma-Korrektur vor der Darstellung
  - <mark style="background: #FFB86CA6;">Pixelwerte müssen angepasst werden, bevor sie auf einem Display dargestellt werden, um die Helligkeit korrekt wiederzugeben</mark>.
  - **Formel für die Korrektur:** Die Intensität $I(n)$ eines Pixels ist proportional zur ursprünglichen Helligkeit $a$ nach der Formel $n \propto a^{1/\gamma}$.
  - **Unabhängige Korrektur für Primärfarben:** <mark style="background: #FFB86CA6;">Diese Korrektur wird für jede Primärfarbe (Rot, Grün, Blau) unabhängig durchgeführt</mark>.
![[Pasted image 20240622084624.png#invert|400]]
- **Diagramm zur Korrektur:**
  - Die Grafik zeigt, wie die Korrektur die nichtlineare Helligkeitswiedergabe (blau) auf eine lineare Helligkeit (orange) umwandelt.
#### Umrechnung von Wert zu Helligkeit
- **Transferfunktion:** Die Helligkeit $I(n)$ wird durch eine Transferfunktion $f$ von einem Wert $n$ im Bereich $[0, N]$ auf eine Helligkeit im Bereich $[I_{\text{min}}, I_{\text{max}}]$ abgebildet.
- **Beispiel:** Eine Helligkeit $a = 0.5$ wird durch die Gamma-Korrektur $\gamma \approx 1.4$ in einen Pixelwert umgewandelt, der $\approx 155$ entspricht. Dies bedeutet, dass $\left(0.5\right)^{1/1.4} \cdot 255 \approx 155$ ergibt.
![[Pasted image 20240622084657.png#invert|400]]
#### Ideale Transferfunktion
- <mark style="background: #FFB86CA6;">aufeinander folgende Pixelwerte sollen keine sichtbaren Helligkeitsstufen verursachen</mark> – sonst würde man in glatten Bildbereichen Bänder erkennen (so wie im Bild)
![[Pasted image 20241028082806.png|200]]
- Experimente haben gezeigt, dass das <mark style="background: #FFB86CA6;">menschliche Auge Helligkeitsunterschiede von etwa 1%-2% erkennen kann</mark>.
- In <mark style="background: #FFB86CA6;">dunkleren Bildbereichen sind kleinere Helligkeitsschritte erforderlich, da das menschliche Auge dort empfindlicher</mark> auf Unterschiede reagiert. Absolut gesehen, muss der Unterschied (z. B. der Helligkeitswert zwischen zwei Pixeln) kleiner sein, um keine sichtbaren "Treppenstufen" oder Bänder zu erzeugen.
##### **Just Noticeable Difference (JND)**:
- <mark style="background: #FFB86CA6;">beschreibt den kleinsten Unterschied in der Helligkeit, den das menschliche Auge wahrnehmen kann</mark>.
$$\Delta \frac{L_{JND}}{L}=\text{const}\approx 1\%\text{ bis }2\%$$
- $L$ ist die Hintergrundhelligkeit.
- Das Verhältnis zeigt, dass der Unterschied bei 1%-2% der Ausgangshelligkeit liegt. Je dunkler der Hintergrund, desto feiner müssen die Helligkeitsstufen sein.
![[Pasted image 20241028083133.png|400]]
##### Bestimmen der idealen Transferfunktion
   - **Helligkeitsunterschied**: <mark style="background: #FFB86CA6;">Ein Unterschied von etwa 2% in der Helligkeit führt zu einer exponentiellen Transferfunktion</mark>, um Farbverläufe ohne sichtbare Stufen zu erzeugen.
   - **Exponentielle Skalierung**: <mark style="background: #FFB86CA6;">Bei jedem Schritt wird die Helligkeit um 2% erhöht</mark> ($1.02 \times I_{\text{min}}$, $1.02^2 \times I_{\text{min}}$, usw.), wodurch ein sanfter Übergang gewährleistet wird.
   - **Logarithmische Berechnung**: Ein kleiner Anstieg von ca. $\log(1.02) \approx \frac{1}{120}$ erfordert ca. 120 Schritte für eine Verdopplung des Dynamikbereichs. <mark style="background: #FFB86CA6;">Je größer der Dynamikumfang des Displays, desto mehr Schritte sind nötig</mark> -> <mark style="background: #FFB86CA6;">Berechnung mit Logarithmus</mark>!
   - **8-Bit-Bilder**: Diese Zahl ist <mark style="background: #FFB86CA6;">oft ausreichend für LCDs</mark>, um eine adäquate Bildrepräsentation zu erreichen.
##### Exponentielle vs. Lineare Quantisierung
   - **Lineare Quantisierung**: Hier werden gleiche Helligkeitsschritte für das gesamte Spektrum verwendet, was bei Helligkeitsunterschieden unter 2% problematisch sein kann.
   - **Anzahl der Schritte**: Um den Bereich zwischen minimaler und maximaler Helligkeit abzudecken, sind bei linearer Quantisierung etwa 50 mal der Dynamikumfang in Schritte erforderlich.
   - **Formel für Dynamikumfang**: Der Dynamikumfang $R_d$ gibt das Verhältnis zwischen $I_{\text{max}}$ und $I_{\text{min}}$ an. LCDs und HDR-Displays haben hohe Dynamikbereiche (z.B. 100:1 für LCDs und 500000:1 für HDR).
##### Abbildung Wert → Helligkeit und Gamma-Korrektur
   - **Transferfunktion $f$**: Diese Funktion ordnet Werte zwischen [0, 1] den Helligkeiten $[I_{\text{min}}, I_{\text{max}}]$ zu. Diese Abbildung ist nicht linear.
   - **Wahrnehmung**: <mark style="background: #FFB86CA6;">Unsere Wahrnehmung erfordert eine exponentielle Transferfunktion, um Farben gleichmäßig darzustellen. Im Gegensatz dazu verwendet der Frame Buffer oft eine Potenzfunktion ($I \propto p^\gamma$), was zu nicht-linearen Helligkeitsverläufen führt.</mark>
   - **Gamma-Korrektur**: Diese Korrektur wird verwendet, um lineare Helligkeitsverläufe darzustellen. Sie gleicht die exponentielle Wahrnehmung des Auges aus.
##### Praxis: Lineare Quantisierung und Speicherung von Bildern
   - **Lineare Quantisierung für Berechnungen**: Einfach und praktisch für Rechenoperationen, jedoch muss eine Gamma-Korrektur durchgeführt werden, bevor das Bild im Frame Buffer gespeichert wird.
   - **Speicherung in potenzbasierten Werten**: Die Werte werden als Potenzfunktion $a^{1/\gamma}$ gespeichert, um die Wahrnehmung gleichmäßig zu verteilen. Typische Systeme speichern in 8-Bit (für SDR) oder 12-Bit (für HDR).
   - **sRGB-Beispiel**: sRGB wurde entwickelt, um eine standardisierte Gamma-Korrektur zu ermöglichen und ist auf 8-Bit kodiert, damit es auf älteren Bildschirmen gut funktioniert.
   - **Linearisierung vor Berechnungen**: Vor der Bildverarbeitung müssen die Werte in lineare Intensitäten umgewandelt werden.
   ![[Pasted image 20241028083445.png|800]]
#### Implementierung der Gamma-Korrektur in Code
  ```c
  // Array: 8-Bit RGB-Frame Buffer
  unsigned char buffer[ WIDTH * HEIGHT * 3 ];
  ...
  // Berechnung der Farbe, RGB-Tripel mit linearen Werten
  float r, g, b;
  r = ...; g = ...; b = ...;
  r += r2; ...
  // Gamma-Korrektur mit gamma-Wert des Displaysystems
  r = pow( r, 1.0 / gamma );
  g = pow( g, 1.0 / gamma );
  b = pow( b, 1.0 / gamma ); 
  buffer[ ( x + y * WIDTH ) * 3 + 0 ] = min( 255, 255.0f * r );
  buffer[ ( x + y * WIDTH ) * 3 + 1 ] = min( 255, 255.0f * g );
  buffer[ ( x + y * WIDTH ) * 3 + 2 ] = min( 255, 255.0f * b );
  ...
  CopyImageToScreen( buffer );
  ```

- **Wichtige Punkte im Code:**
  - **Initialisierung des Buffers:** Der Frame-Buffer wird als ein Array definiert, das die RGB-Werte der Bildpunkte enthält.
  - **Berechnung der Farbwerte:** Die Farbwerte $r$, $g$ und $b$ werden in einem linearen Farbraum berechnet.
  - **Anwendung der Gamma-Korrektur:** Die Farbwerte werden mit der Funktion `pow` angepasst, wobei $\gamma$ der Gamma-Wert des Displays ist.
  - **Clipping:** Die Werte werden auf 255 beschränkt, um sicherzustellen, dass sie in den 8-Bit-Bereich (0 bis 255) passen.
  - **Speichern in den Buffer:** Die korrigierten Farbwerte werden im Frame-Buffer gespeichert.
  - **Anzeigen des Bildes:** Der Frame-Buffer wird schließlich auf dem Bildschirm dargestellt.
