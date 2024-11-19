## Messmethoden
### Absolutdruck
- Diese Sensoren messen den Druck im Vergleich zu einem perfekten Vakuum. 
- Das Referenzvakuum sollte dabei so klein sein, dass es im Vergleich zum zu messenden Druck vernachlässigbar ist.
- In [[Blutdruckmessgeräte|Blutdruckmessgeräten]] werden sie nicht typischerweise verwendet, da sie vor allem zur Messung des Luftdrucks in der Meteorologie oder Höhenmessung eingesetzt werden.
### Relativdruck
- Diese Sensoren messen den Druck im Vergleich zum aktuellen Luftdruck der Umgebung. 
- In der Medizin wird der Blutdruck oft als Relativdruck gemessen, wobei der atmosphärische Druck als Referenz dient. 
- Wenn der gemessene Druck $p_{\text{mess}}$ größer als der atmosphärische Druck $p_{\text{atm}}$ ist, spricht man von Überdruck. Ist er kleiner, handelt es sich um Unterdruck. 
- Die meisten [[Blutdruckmessgeräte]] nutzen diese Methode.
### Differenzdruck
- Diese Sensoren messen den Unterschied zwischen zwei Drücken. 
- Sie werden in der Medizintechnik verwendet, um beispielsweise den Atemfluss oder den Gasfluss zu messen, was für die Überwachung von Patienten in kritischen Zuständen wichtig sein kann.

## Sensortechnologien
### Piezoresitive Drucksensoren
#### Aufbau
- Piezoresistive Drucksensoren bestehen aus einer Druckmembran, in die Dehnungsmesstreifen integriert sind. 
- Diese Streifen sind meist aus Silizium-Halbleitermaterial gefertigt, da dieses Material eine hohe Empfindlichkeit für Druckänderungen aufweist.
#### Messbrücke
- Um Temperatureffekte zu kompensieren und präzise Messungen zu ermöglichen, werden vier dieser Dehnungsmesstreifen zu einer sogenannten Wheatstone-Brücke (Vollbrücke) zusammengeschaltet. 
- Diese Konfiguration ermöglicht es, kleine Widerstandsänderungen, die durch Druckänderungen verursacht werden, genau zu messen.
#### Wiederstandsänderung
- Die relative Änderung des Widerstands $\Delta R$ im Verhältnis zum ursprünglichen Widerstand $R$ ist proportional zur relativen Dehnung $\epsilon$ des Materials. Der Proportionalitätsfaktor $k$ ist für Halbleitermaterialien etwa 100-mal größer als für metallische Dehnungsmesstreifen (DMS), was zu einer höheren Sensitivität führt: $$\frac{\Delta R}{R}=k\frac{\Delta I}{i}=k\epsilon$$
#### Signalumwandlung
- Wenn Druck auf die Membran ausgeübt wird, dehnen sich zwei der Widerstände ($R_1$ und $R_4$) und zwei werden gestaucht ($R_2$ und $R_3$). Das Verhältnis der Ausgangsspannung $U_O$ zur Eingangsspannung $U_B$ ist proportional zur relativen Widerstandsänderung:$$\frac{U_{O}}{U_{B}}=\frac{R_{2}R_{3}-R_{1}R_{4}}{(R_{1}+R_{2})(R_{3}R_{4})}$$
- Unter der Annahme, dass $R_{1}=R_{4}=R+\Delta R$ und $R_{2}=R_{3}=R-\Delta R$ vereinfacht sich die Gleichung zu: $$\frac{U_{O}}{U_{B}}=- \frac{\Delta R}{R}$$
- Diese Gleichung zeigt, dass die Ausgangsspannung $U_O$direkt proportional zur Widerstandsänderung $\Delta R$ ist, was eine direkte Messung des Drucks ermöglicht.
### Piezoelektrische Drucksensoren
- In einem [[Piezoelectric Effect|piezoelektrischen Kristall]] wird eine anliegende Kraft in eine elektrische Spannung (Oberflächenladung) umgewandelt. 
- Die resultierende Spannung ist direkt proportional zur Kraft pro Fläche = Druck
- Da ein Piezokristall eine sehr hochohrige Spannungsquelle darstellt, muss möglichst stromlos gemessen werden (Ladungsverstärkung).
#### Übungsaufgabe: Welche Operationsverstärkerschaltung wird benötigt?
Für piezoelektrische Drucksensoren, die eine hochohmige Spannungsquelle darstellen, wird in der Regel eine Operationsverstärkerschaltung benötigt, die als Ladungsverstärker (Charge Amplifier) fungiert. Diese Schaltung wandelt die von einem piezoelektrischen Sensor erzeugte Ladung in ein proportionales Spannungssignal um, das dann leichter zu messen und zu verarbeiten ist.
Die grundlegende Operationsverstärkerschaltung für diese Anwendung ist ein **Invertierender Ladungsverstärker**, der folgende Komponenten umfasst:
- **Operationsverstärker**: Ein hochwertiger Op-Amp mit niedrigem Eingangsstrom und hoher Eingangsimpedanz.
- **Feedback-Kapazität (Cf)**: Diese Kapazität ist parallel zum Feedback-Widerstand geschaltet und bestimmt zusammen mit dem Eingangswiderstand die Verstärkung des Ladungsverstärkers.
- **Eingangswiderstand (Ri)**: Ein hochohmiger Widerstand, der den Eingangsstrom des Verstärkers begrenzt und zur Stabilisierung der Schaltung beiträgt.
- **Ausgang**: Der Ausgang des Verstärkers liefert ein Spannungssignal, das proportional zur am Sensor anliegenden Ladung ist.
Die Schaltung arbeitet so, dass die Ladung, die vom piezoelektrischen Sensor erzeugt wird, auf die Feedback-Kapazität übertragen wird. Der Operationsverstärker integriert die Ladung über die Zeit, was zu einer Ausgangsspannung führt, die proportional zum Druck ist, der auf den Sensor ausgeübt wird.
Die genaue Konfiguration und die Werte der Komponenten hängen von den spezifischen Anforderungen der Messanwendung ab, wie z.B. der gewünschten Bandbreite, der Empfindlichkeit und der Temperaturstabilität.
### Vergleich
![[Pasted image 20240429123720.png#invert|700]]