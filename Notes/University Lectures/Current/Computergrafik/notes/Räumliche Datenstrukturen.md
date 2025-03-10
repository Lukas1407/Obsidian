## Optimierung von Raytracing
- Schnittpunkt Berechnung von Raytracing nimmt beinah die gesammte Berechnungszeit in anspruch: $O(n)$ für $n$ Objekte/Primitive
- Lösung: Vermeidung von Schnittberechnungen
	- frühzeitiges Ausschließen geometrischer Primitive, die nicht geschnitten werden 
	- finde so potentiell geschnittene Objekte schneller 
	- Unterteile den Raum oder die Menge der Primitiv
	- -> Hüllkörper
## Hüllkörper
- schneller Ausschluss von Schnitttests mit Primitiven (Dreiecken) durch Schnitt mit einer <mark style="background: #FFB86CA6;">einschließenden Geometrie („Hüllkörper“)</mark>
![[Pasted image 20250125082635.png#invert|400]]
- sollen möglichst enganliegend/klein sein
- <mark style="background: #FFB86CA6;">Trade-off zwischen Effizienz und Speicher/Kosten</mark> sowohl bei <mark style="background: #FFB86CA6;">Berechnung</mark> als auch <mark style="background: #FFB86CA6;">beim Schnitttest</mark>
### Kugel
- schneller Schnittalgorithmus 
- oft schlechte Effizienz, da zu groß
### Achsenparallele Box (AABB)
- einfache Berechnung (min/max) zur Bestimmung der Eckpunkte 
- wichtigster Hüllkörper
![[Pasted image 20250125082945.png|200]]
- Objekt kann transformiert werden um bounding box zu verkleinern
![[Pasted image 20250125083418.png|400]]
- <mark style="background: #FFB86CA6;">erhöhte Kosten für Bestimmung</mark> der AABB, aber i.d.R. <mark style="background: #FFB86CA6;">deutlich kleinerer Hüllkörper und somit weniger Falschpositive</mark>
#### Schnittberechnung
- **Strahl**:  $r(t) = e + td$, wobei $e$ der Ursprung des Strahls und $d$ die Richtung ist.
- Für jede Dimension berechne die Parameter $t_1$ und $t_2$, die die <mark style="background: #FFB86CA6;">Eintritts- und Austrittspunkte des Strahls entlang der Achse bestimmen</mark>:$$
  t_1 = \frac{X_1 - e_x}{d_x}, \quad t_2 = \frac{X_2 - e_x}{d_x}
  $$
- Wenn $t_1 > t_2$, tausche die Werte, sodass immer $t_1 \leq t_2$ gilt. 
- Initialisiere $t_{\text{near}}$ und $t_{\text{far}}$ mit $-\infty$ bzw. $+\infty$.
- Aktualisiere $t_{\text{near}}$ und $t_{\text{far}}$ für jede Dimension:
  $$
  t_{\text{near}} = \max(t_{\text{near}}, t_1), \quad t_{\text{far}} = \min(t_{\text{far}}, t_2)
  $$
##### Abbruchbedingungen prüfen
- Wenn $t_{\text{near}} > t_{\text{far}}$: Der Strahl verfehlt die Box.
- Wenn $t_{\text{far}} < 0$: Der Strahl verläuft hinter dem Ursprung.
##### Ergebnis
- Liegt $t_{\text{near}} \leq t_{\text{far}}$, gibt es eine Überschneidung.
- Das Intervall $[t_{\text{near}}, t_{\text{far}}]$ beschreibt den Abschnitt des Strahls, der sich in der AABB befindet.
### Orientierte Bounding Box (OBB)
- aufwändigere Berechnung 
![[Pasted image 20250125082959.png|200]]
### allg. konvexe Region
- Spezialfall „Slabs“: <mark style="background: #FFB86CA6;">Schnitt von Paaren paralleler Halbebenen</mark> 
- gute Effizienz, schnelle Berechnung
![[Pasted image 20250125083052.png|200]]
## Hierarchie von einschließenden Hüllkörpern (bounding volume hierarchy, BVH)
- Beschleunigt Raytracing
- **Hierarchie**: <mark style="background: #FFB86CA6;">Objekte werden in Gruppen zusammengefasst</mark>, wobei jede <mark style="background: #FFB86CA6;">Gruppe von einem größeren Hüllkörper</mark> (z. B. einer AABB) umschlossen wird.
- Diese Hüllkörper werden hierarchisch organisiert, sodass eine rekursive Suche durch die Hierarchie erfolgen kann.
![[Pasted image 20250125084758.png|500]]
1. **Baumstruktur**:
    - Die Hierarchie wird als Baum dargestellt, wobei die <mark style="background: #FFB86CA6;">Wurzel den gesamten Raum umfasst</mark>.
    - Die <mark style="background: #FFB86CA6;">Blätter des Baums repräsentieren einzelne Objekte.</mark>
### **Vorteile der Hierarchie**
- **Schnelle Suche**: <mark style="background: #FFB86CA6;">Anhand des Strahlschnitts</mark> mit den Hüllkörpern <mark style="background: #FFB86CA6;">können ganze Gruppen von Objekten ausgeschlossen werden, ohne dass einzelne Objekte überprüft werden müssen.</mark>
- **Effizienz**: Anstatt alle $n$ Objekte individuell zu prüfen (Komplexität $O(n)$), wird die Suche auf <mark style="background: #FFB86CA6;">logarithmische Komplexität</mark> $O(\log(n))$ reduziert.
### **Ablauf der Suche**
1. **Ray-Bounding-Box-Test**:
    - Der <mark style="background: #FFB86CA6;">Strahl wird zuerst mit dem Hüllkörper der Wurzel getestet</mark>.
    - Liegt <mark style="background: #FFB86CA6;">kein Schnittpunkt vor, können alle untergeordneten Objekte ausgeschlossen werden</mark>.
2. **Rekursive Verfeinerung**:
    - Wenn ein Schnittpunkt vorliegt, wird <mark style="background: #FFB86CA6;">die Suche rekursiv in die untergeordneten Hüllkörper fortgesetzt</mark>.
    - Am <mark style="background: #FFB86CA6;">Ende wird nur mit den tatsächlich relevanten Objekten</mark> interagiert.
3. **Ausschluss von Gruppen**:
    - Wenn ein <mark style="background: #FFB86CA6;">Hüllkörper keinen Schnittpunkt mit dem Strahl hat, werden alle darin enthaltenen Objekte ausgeschlossen</mark>.
### Aufbau des BVH
- erfolgt nach einem <mark style="background: #FFB86CA6;">Top-Down-Ansatz</mark>
#### 1. Bestimme die Wurzel, also den gemeinsamen Hüllkörper aller Primitiven
#### 2. Teile die Primitive in zwei Gruppen auf
- Rekursiv entlang einer **Trennlinie** in zwei Gruppen. Diese Trennlinie kann auf verschiedene Weisen definiert werden:
    - Senkrecht zur Achse der größten Ausdehnung.
    - Entlang der xxx-, yyy- oder zzz-Achse.
    - Basierend auf einer Kostenfunktion wie der **Surface Area Heuristik** (minimiert die erwarteten Schnittkosten).
![[Pasted image 20250125085606.png|400]]
- Dieser Prozess <mark style="background: #FFB86CA6;">wird rekursiv fortgeführt, bis</mark>:
    - <mark style="background: #FFB86CA6;">Jede Gruppe nur noch ein einzelnes Primitive enthält</mark> (Blätter des Baums), oder
    - <mark style="background: #FFB86CA6;">Eine vordefinierte Bedingung (z. B. maximale Baumtiefe) erreicht ist</mark>.
#### Speicherstruktur
- **Innere Knoten**:
    - Jeder Knoten speichert den Hüllkörper seiner Kindknoten sowie Verweise auf diese Kindknoten.
- **Blätter**:
    - Blätter speichern die Primitive direkt.
- **Caching**:
    - Die für die <mark style="background: #FFB86CA6;">Traversierung benötigten Informationen werden effizient gespeichert, um die Traversierung zu beschleunigen</mark>.
![[University Lectures/Current/Computergrafik/notes/images/Untitled 3.png|400]]
- <mark style="background: #FFB86CA6;">Vorsicht: liefere einen gefundenen Schnittpunkt nicht sofort zurück, es könnte einen näheren Schnittpunkt in einem anderen Knoten geben </mark>
![[Pasted image 20250125090002.png|400]]
- <mark style="background: #FFB86CA6;">Grund: Hüllkörper können sich überlappen ebenfalls eine der Ursachen für Aufwand schlechter</mark> als $O(\log(n))$ 

## Raumunterteilung durch adaptive Gitter
- Der <mark style="background: #FFB86CA6;">Raum wird in Zellen gleicher Größe und Form unterteilt</mark>. Dies wird oft aus der Bounding Box der Szene abgeleitet.
- Der <mark style="background: #FFB86CA6;">Raum wird nur dort weiter unterteilt, wo es nötig ist</mark>. Das heißt, die Unterteilung ist adaptiv an die Verteilung der Objekte angepasst.
## Binary Space Partitioning Tree (BSP-Baum)
- <mark style="background: #FFB86CA6;">Teilen den Raum rekursiv mit beliebig orientierten Ebenen</mark>.
![[Pasted image 20250125090640.png|400]]
![[Pasted image 20250125090657.png|400]]
### Traversierung
- Ein Strahl  wird durch die Szene geschossen. 
- Die <mark style="background: #FFB86CA6;">Split-Ebene eines Knotens wird getestet, und der Strahl wird rekursiv durch die Kindknoten traversiert</mark>.
## kD-Baum
- Verwenden <mark style="background: #FFB86CA6;">Ebenen, die immer senkrecht zu einer der Koordinatenachsen (x, y, z) stehen</mark>.
![[Pasted image 20250125090840.png|400]]
### Traversierung
- Beginn bei der Wurzel: Der Stack enthält den Wurzelknoten.
- Prüfe, ob der Strahl die Split-Ebene eines Knotens schneidet.
![[Pasted image 20250125091040.png|500]]
- Falls ja:
    - Traverse zuerst den Kindknoten, der den Strahl enthält.
    - Danach den anderen Kindknoten, falls nötig.
![[Pasted image 20250125091114.png|500]]
![[Pasted image 20250125091234.png|500]]
- Falls der Strahl ein Blatt erreicht:
	- Teste auf Schnittpunkte mit den enthaltenen Primitiven.
	- Gib den nächsten Schnittpunkt zurück, falls er im Intervall $[t_{min}, t_{max}]$ liegt.
![[Pasted image 20250125091431.png|500]]
![[Pasted image 20250125091441.png|500]]
![[Pasted image 20250125114314.png|500]]
### **Vorteile**:
- <mark style="background: #FFB86CA6;">BSP- und kD-Bäume sind einfach zu konstruieren</mark>.
- <mark style="background: #FFB86CA6;">Effizient bei Traversierung</mark>: Reduziert die Anzahl der getesteten Primitiven.
### **Herausforderungen**:
- Ein Objekt kann in mehreren Knoten vorkommen, da die Split-Ebene ein Objekt schneiden kann. Dies führt zu mehrfacher Indizierung.
- Eine gute Unterteilung zu finden, kann aufwändig sein
## Unterschiede BSP-Baum und kD-Baum
- BSP-Bäume sind flexibler, weil die Trennebenen beliebig orientiert sein können, während <mark style="background: #FFB86CA6;">kD-Bäume einfacher aufgebaut sind und bevorzugt verwendet</mark> werden.
### Kombination mit AABBs
- manchmal speichert man für die Primitive eines Knotens <mark style="background: #FFB86CA6;">zusätzlich eine AABB</mark>
- hilfreich, wenn Primitive nur einen kleinen Teil des Raums einnehmen 
- für Schnitttest pro Knoten bedeutet das: teste nur auf Schnitt mit Primitiven, wenn die AABB geschnitten wird
![[Pasted image 20250125091807.png|500]]

## Geschickter Aufbau von BVH und kD-Bäumen
- <mark style="background: #FFB86CA6;">um effiziente Traversierung und minimalen Rechenaufwand zu ermöglichen</mark>
### Räumliches Mittel (Spatial Median)
- Teilt die Bounding Volume (Raum) <mark style="background: #FFB86CA6;">in der Mitte entlang der Achse mit der größten Ausdehnung</mark>.
- <mark style="background: #FFB86CA6;">Alternativ: Iterative Teilung entlang der x-, y-, und z-Achse</mark>.
- Vorteil: Der Baum erreicht eine <mark style="background: #FFB86CA6;">logarithmische Tiefe</mark> $O(\log n)$, <mark style="background: #FFB86CA6;">wenn die Primitive gleichmäßig verteilt sind</mark>.
- <mark style="background: #FF5582A6;">Nachteil</mark>: Kann bei <mark style="background: #FF5582A6;">ungleichmäßig verteilten Objekten zu unausgeglichenen Teilbäumen</mark> führen.
![[Pasted image 20250125092240.png|700]]
### Objektmedian (Object Median)
- Teilt die Objekte so, dass jede <mark style="background: #FFB86CA6;">Teilmenge etwa gleich viele Primitive enthält</mark>.
- <mark style="background: #FFB86CA6;">Typischerweise wird entlang der Achse mit der größten Ausdehnung geteilt</mark>.
- <mark style="background: #FFB86CA6;">Aufbau erfordert</mark> $O(n \log^2 n)$, <mark style="background: #FFB86CA6;">da eine Sortierung entlang der Achse notwendig ist</mark>.
- Vorteil: Führt zu besser <mark style="background: #FFB86CA6;">balancierten Bäumen</mark>.
### **Kostenfunktion (Surface Area Heuristic, SAH)**
- <mark style="background: #FFB86CA6;">Ziel: Minimierung der erwarteten Kosten für die Traversierung</mark>.
- SAH <mark style="background: #FFB86CA6;">berücksichtigt die Wahrscheinlichkeit, dass ein Strahl einen der beiden Kindknoten trifft, basierend auf den Oberflächenverhältnissen der Boxen</mark>.
- Ergebnis: <mark style="background: #FFB86CA6;">Balancierte Bäume mit minimierten Traversierungskosten</mark>.
- Aufbau kann $O(n \log^2 n)$ betragen.
#### Kostenfunktion
![[Pasted image 20250125092813.png|600]]
![[Pasted image 20250125093410.png|600]]

### Optimierung
- **Approximative Konstruktion**: Wenige Split-Kandidaten entlang der x-, y-, z-Achsen testen.
- **Inkrementelle Berechnung**: <mark style="background: #FFB86CA6;">Effiziente Berechnung der SAH-Werte durch Sortierung der Primitive</mark> und Speicherung der Bounding Volumes.

## Suche im kD-/BSP-Baum
- <mark style="background: #FFB86CA6;">Suche nach dem nächsten Objekt</mark> oder der nächsten Primitiven
#### **1. Start an der Wurzel des Baumes**
- Der Punkt $e$, von dem aus gesucht wird, wird in die <mark style="background: #FFB86CA6;">entsprechende Hälfte der Split-Ebene eingeordnet</mark>.
- **Split-Ebene**: Eine geometrische Trennebene, die den Raum in zwei Hälften teilt, z. B. entlang der x- oder y-Achse.
- Beispiel: Wenn eee sich im **positiven Halbraum** befindet, werden die **negativen Halbräume** weiter „hinten“ im Baum als weniger relevant angesehen.
![[Pasted image 20250125093905.png|500]]
#### **2. Suche im lokalen Teilraum**
- Die Suche geht in die Richtung, in der sich $e$ befindet. Dies ist der sogenannte "lokale Teilbaum".
- **Achtung**: Dieses Verfahren findet den Unterraum, in dem sich $e$ befindet, garantiert. Es identifiziert jedoch nicht immer das **nächste Objekt**.
![[Pasted image 20250125093959.png|500]]
#### **3. Tiefer suchen**
- Im nächsten Schritt wird nach dem **nächsten Teilbaum** gesucht, in dem $e$ liegt.
- Alle **anderen Teilräume** (die eventuell nicht direkt naheliegen) können ebenfalls mögliche Kandidaten enthalten, falls sie näher an eee liegen als die bisher betrachteten Primitiven.
#### **4. Wiederhole für jeden Eltern-Knoten**
- Nachdem ein Unterraum überprüft wurde, wird der nächsthöhere **Eltern-Knoten** untersucht. Hierbei prüft man, ob der gegenüberliegende Kind-Knoten im Baum näher an eee liegen könnte.
- Beispiel: Wenn sich ein Objekt im negativen Halbraum befindet, das eee näher ist als das bisher gefundene nächste Primitive, wird der Teilbaum erneut durchsucht.
#### **5. Tiefensuche**
- Der Algorithmus steigt in den relevanten Teilbäumen weiter ab, um sicherzustellen, dass alle näheren Objekte berücksichtigt werden.
- Dies geschieht mit einer **Tiefensuche (Depth-First Traversal)**:
    - Zuerst der **nähere Teilbaum**, dann der weiter entfernte.
    - Effizienz: Im Mittel $O(\log n)$, im Worst Case jedoch $O(n)$