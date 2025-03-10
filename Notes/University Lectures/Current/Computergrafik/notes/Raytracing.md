Raytracing ist eine Methode in der Computergrafik, die realistische Bilder erzeugt, indem sie <mark style="background: #FFB86CA6;">das Verhalten von Lichtstrahlen (Rays) simuliert</mark>. Die Idee ist, <mark style="background: #FFB86CA6;">Lichtstrahlen von der Kamera aus durch jedes Pixel eines Bildes</mark> zu senden und deren Weg in der virtuellen Szene zu verfolgen. So kann man berechnen, <mark style="background: #FFB86CA6;">wie das Licht von verschiedenen Objekten reflektiert, gebrochen oder gestreut</mark> wird.

- Bildsyntheseverfahren
- Beim Raytracing schickt man Lichtstrahlen durch die "Öffnung" durch die Pixel der Bildebene in die Szene, um zu berechnen, wie das Bild aussehen würde. 
	- -> [[Pinhole Camera]]
	- Da es keine Linse gibt, ist das Bild immer scharf, was das Modell vereinfacht und für die Berechnungen praktisch macht.

## Grundidee: Whitted-Style Raytracing
### Primärstrahl (Sichtstrahl):
- <mark style="background: #FFB86CA6;">Für jeden Pixel des Bildes wird ein Strahl (Ray) von der Kamera aus durch das Pixel in die Szene geschickt</mark>. Dieser Strahl wird "Primärstrahl" oder "Sichtstrahl" genannt.
- Der Strahl bewegt sich entlang seiner Richtung und <mark style="background: #FFB86CA6;">trifft auf das nächstgelegene Objekt</mark> in der Szene.
![[Pasted image 20241108082747.png|400]]
### Schnittberechnung (ray casting, ray intersection)
- <mark style="background: #FFB86CA6;">finde Dreieck, das den Sichtstrahl am nächsten zur Kamera schneidet</mark>
![[Pasted image 20241108091258.png|400]]
### Schattierung (Lighting/Illumination):
- <mark style="background: #FFB86CA6;">Sobald der Primärstrahl ein Objekt trifft, wird die Beleuchtung an diesem Punkt berechnet</mark>.
- Die Berechnung <mark style="background: #FFB86CA6;">umfasst die Einflüsse aller Lichtquellen in der Szene</mark>, wodurch die Helligkeit und die Farbe der Oberfläche bestimmt werden.
![[Pasted image 20241108082813.png|400]]
- Schattierung ist <mark style="background: #FFB86CA6;">essentiell für dreidimensionalen Eindruck</mark> und ist ein Ergebnis von 
	- Emission der Lichtquellen (Intensität, Farbe, Position, …) und 
	- Oberflächeneigenschaften (Material, Rauheit, Orientierung zur LQ, …
- Das wird mit Hilfe von <mark style="background: #FFB86CA6;">lokale Beleuchtungsberechnungen</mark> und <mark style="background: #FFB86CA6;">globale Schattenstrahlen</mark> berechnet
#### Licht-Material Interaktion
**Was passiert, wenn Licht auf eine glatte Oberfläche trifft?**
- <mark style="background: #FFB86CA6;">Wenn Licht auf eine glatte Oberfläche trifft, teilt es sich auf</mark>: Ein Teil des Lichts wird <mark style="background: #FFB86CA6;">reflektiert</mark>, während der andere Teil <mark style="background: #FFB86CA6;">in das Material eindringt</mark>.
![[Pasted image 20241108092013.png|200]]
- „Glatt“ bedeutet, dass die Oberfläche keine sichtbaren Unregelmäßigkeiten oder Rauheit hat. <mark style="background: #FFB86CA6;">Eine glatte Oberfläche reflektiert das Licht in eine bestimmte Richtung (spiegelnd), im Gegensatz zu einer rauen Oberfläche, die das Licht in viele Richtungen streut</mark>.
- <mark style="background: #FFB86CA6;">Das Verhältnis zwischen reflektiertem und eindringendem Licht hängt vom Einfallswinkel des Lichts und den Brechungsindizes beider Materialien ab</mark> (also des Mediums, aus dem das Licht kommt, und des Materials, in das es eindringt).
- <mark style="background: #FFB86CA6;">Diese Abhängigkeit wird durch den Fresnel-Effekt beschrieben</mark>. Bei <mark style="background: #FFB86CA6;">flachem Einfallswinkel wird mehr Licht reflektiert</mark>, während bei <mark style="background: #FFB86CA6;">steilerem Winkel mehr Licht in das Material eindringt</mark>.
- Wenn Licht in das Material eindringt, kann es:
	- <mark style="background: #FFB86CA6;">Absorbiert werden</mark> (besonders bei Metallen): Das Licht verliert dabei schnell seine Energie und wird <mark style="background: #FFB86CA6;">in Wärme umgewandelt</mark>, was das typische Verhalten von Metallen erklärt.
	- <mark style="background: #FFB86CA6;">Aus dem Material wieder austreten</mark>: Bei durchsichtigen oder transluzenten Materialien (<mark style="background: #FFB86CA6;">wie Glas</mark>) kann das Licht das Material <mark style="background: #FFB86CA6;">an einer anderen Stelle verlassen</mark>, was zu Effekten wie Lichtbrechung und Transparenz führt.
### Schattenstrahl (Shadow Ray):
- <mark style="background: #FFB86CA6;">Für jeden sichtbaren Punkt wird zusätzlich ein "Schattenstrahl" von diesem Punkt zu den Lichtquellen geschickt</mark>.
- Wenn ein Schattenstrahl ein Objekt auf dem Weg zur Lichtquelle trifft, bedeutet das, dass <mark style="background: #FFB86CA6;">das Objekt im Schatten</mark> ist und <mark style="background: #FFB86CA6;">entsprechend weniger beleuchtet</mark> wird. Dieser Vorgang erzeugt realistische Schatten im Bild.
![[Pasted image 20241108084815.png|400]]
### Sekundärstrahlen für Reflexion und Brechung:
- <mark style="background: #FFB86CA6;">Wenn die getroffene Oberfläche spiegelnd ist, wird ein neuer Strahl, der "Reflexionsstrahl," von der Oberfläche aus reflektiert</mark>. Dieser Strahl bewegt sich in die Reflexionsrichtung und trifft auf ein weiteres Objekt in der Szene.
- Für transparente Objekte wird ein "Brechungsstrahl" erzeugt, der das Licht durch das Objekt hindurch weiterverfolgt und die Lichtbrechung simuliert.
- <mark style="background: #FFB86CA6;">Diese Sekundärstrahlen setzen die Strahlverfolgung (Raytracing) fort</mark> und werden rekursiv weiterverfolgt, bis sie eine bestimmte maximale Tiefe erreichen oder keinen weiteren Effekt haben.
![[University Lectures/Current/Computergrafik/notes/images/Untitled.png|400]]

## Arten von Stahlen im Whitted Style Raytracing
  ![[Pasted image 20241113092607.png|600]]
### Primärstrahlen
### Schattenstrahlen
### Reflexionsstrahlen
- Strahlen, die an einer <mark style="background: #FFB86CA6;">spiegelnden Oberfläche</mark> gemäß dem Reflexionsgesetz abgelenkt werden.
- <mark style="background: #FFB86CA6;">Reflexionswinkel ist gleich dem Einfallswinkel</mark>
- **Rekursion:** Reflexionsstrahlen können weitere Reflexionsstrahlen erzeugen, solange die maximale Rekursionstiefe nicht erreicht ist.
### Brechungsstrahlen (Transmissionsstrahlen)
- Strahlen, die <mark style="background: #FFB86CA6;">durch transparente oder durchscheinende Materialien dringen</mark> und dabei gemäß dem **Snelliusschen Brechungsgesetz** abgelenkt werden
- Simuliert die Lichtbrechung bei Materialien wie Glas oder Wasser.
### Sekundärstrahlen
- Allgemeiner Begriff für alle Strahlen, die von einem Schnittpunkt ausgesendet werden, z. B. <mark style="background: #FFB86CA6;">Reflexions-, Brechungs- oder Schattenstrahlen.</mark>
- werden rekursiv verfolgt
### Kombination der Ergebnisse:
- Die <mark style="background: #FFB86CA6;">Ergebnisse der Schattierung, Schattenstrahlen und Sekundärstrahlen (für Reflexion und Brechung) werden kombiniert, um die endgültige Farbe und Helligkeit des Pixels festzulegen.</mark>
- Dieser Prozess wird für jedes Pixel des Bildes wiederholt, um ein vollständiges, realistisches Bild zu erstellen.
![[University Lectures/Current/Computergrafik/notes/images/Untitled 1.png|400]]
### Exponentieller Anstieg der Berechnung
Die Anzahl der Strahlen wächst <mark style="background: #FFB86CA6;">exponentiell mit der Rekursionstiefe</mark>:
- Bei einer Tiefe von $n$ können bis zu $2^n$ Sekundärstrahlen entstehen:
    - Jeder Strahl kann <mark style="background: #FFB86CA6;">im schlimmsten Fall einen Reflexionsstrahl und einen Transmissionsstrahl erzeugen</mark>.
    - Schattenstrahlen werden zusätzlich für jede Lichtquelle berechnet, sind aber in der Rekursion nicht enthalten.
## Parameter
### $k_d$ (Diffusionsanteil)
- Beschreibt den Anteil des Lichts, der <mark style="background: #FFB86CA6;">gleichmäßig in alle Richtungen gestreut wird</mark>.
- **kd​>0:** Das Material hat eine diffuse Reflexion. In diesem Fall reflektiert das Material Licht in alle Richtungen (z. B. wie matte Oberflächen).
- Whitted-Style Raytracing unterstützt nur **spekulare Reflexionen** (glatte, spiegelnde Oberflächen). <mark style="background: #FF5582A6;">Diffuse Reflexionen werden nicht durch zusätzliche Sekundärstrahlen modelliert</mark>.
### $k_a$ (Umgebungsanteil)
- Beschreibt den Umgebungslichtanteil, der <mark style="background: #FFB86CA6;">unabhängig von der Lichtquelle auf das Material einwirkt</mark>.
- $k_{a}>0$: Das Material reflektiert kein Umgebungslicht. Es gibt keine zusätzliche Grundbeleuchtung.
### $k_s$ (Spekularanteil)
- Beschreibt die <mark style="background: #FFB86CA6;">spiegelnde Reflexion des Materials</mark> 
- -> Dies ist das Licht, das <mark style="background: #FFB86CA6;">in einer bestimmten Richtung reflektiert</mark> wird, wie bei glänzenden Oberflächen.
- **ks​=0:** Das Material hat keine spekulare Reflexion. Es wirkt matt und hat keine glänzenden Highlights.
### $k_t$​ (Transmissionsanteil):
- **Beschreibung:** Beschreibt den Anteil des <mark style="background: #FFB86CA6;">Lichts, der durch das Material hindurchtritt</mark> (Transparenz).
- $k_{t}=0$ Das Material ist nicht transparent. Es lässt kein Licht durch.
### **$k_r$​ (Reflexionsanteil):**
- **Beschreibung:** Beschreibt die Menge des Lichts, die <mark style="background: #FFB86CA6;">vom Material in Spiegelrichtung reflektiert wird</mark> (spiegelnde Reflexion).
- $k_{r}=0$ Das Material hat keine reflektierende Oberfläche.
### **$\eta_{Block}$​ (Brechungsindex des Glasblocks):**
- **Beschreibung:** Beschreibt das <mark style="background: #FFB86CA6;">Verhältnis der Lichtgeschwindigkeit im Vakuum zur Lichtgeschwindigkeit im Material</mark> (Brechungsindex).
- $\eta_{Block} > 1$ Der Glasblock hat einen Brechungsindex größer als 1, was typisch für optisch dichte Materialien wie Glas ist. Das Licht wird gebrochen, wenn es in das Material eintritt oder es verlässt.

## Punktlichtquellen erzeugen harte Schatten 
- Eine <mark style="background: #FFB86CA6;">Punktlichtquelle L erzeugt harte Schatten</mark>. Die <mark style="background: #FFB86CA6;">Übergänge zwischen beleuchteten und unbeleuchteten Bereichen sind abrupt</mark>, da das Licht von einem einzigen Punkt ausgeht.
- Whitted-Style Raytracing modelliert Punktlichtquellen und berechnet keine weiche Schattenbildung durch Flächenlichtquellen (es sei denn, zusätzliche Techniken wie Area Sampling werden verwendet).
- Whitted-Style Raytracing <mark style="background: #FFB86CA6;">unterstützt nur spiegelnde Reflexionen</mark> von Punktlichtquellen. Diffuse Reflexionen (wie von Wänden oder Flächenlichtquellen) werden nicht in Spiegelungen berücksichtigt.