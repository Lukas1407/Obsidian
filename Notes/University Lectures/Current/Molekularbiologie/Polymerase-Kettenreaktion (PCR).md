Die Polymerase-Kettenreaktion (PCR) ist eine Labormethode, die es ermöglicht, bestimmte DNA-Abschnitte in kurzer Zeit millionenfach zu vervielfältigen.
![[Pasted image 20241112093310.png#invert|400]]
- Der Zyklus wird typischerweise 30 mal wiederholt, wonach man 1.073.741.824 Kopien erhält
- Das Vorgehen geht nicht exponentiell weiter, sondern plateut
![[Pasted image 20241112093513.png#invert|200]]
## Elemente der PCR
- **Template-DNA**: Das ist die DNA, die vervielfältigt werden soll. Sie enthält die Zielsequenz, die durch die PCR vervielfältigt wird.
- **Zwei kurze Oligonukleotide (Primer)**: Diese <mark style="background: #FFB86CA6;">kurzen DNA-Stücke sind komplementär zu den Enden der Zielsequenz</mark> auf der Template-DNA. Sie <mark style="background: #FFB86CA6;">markieren den Start- und Endpunkt</mark> für die DNA-Polymerase und sorgen dafür, dass die richtige Region vervielfältigt wird.
- **dNTPs (Desoxynukleosidtriphosphate)**: Das sind die <mark style="background: #FFB86CA6;">Bausteine der neuen DNA-Stränge</mark> (dATP, dTTP, dGTP und dCTP). Die DNA-Polymerase verwendet diese Nukleotide, um die komplementären Stränge zu der Template-DNA zu bilden.
- **Puffer mit Mg²⁺**: Der Puffer stellt den optimalen pH-Wert und die Bedingungen für die DNA-Polymerase sicher. Mg²⁺-Ionen sind dabei besonders wichtig, weil sie als <mark style="background: #FFB86CA6;">Cofaktoren für die Aktivität der Polymerase</mark> wirken und deren Funktion stabilisieren.
- **Hitzestabile DNA-Polymerase**: Diese spezielle Polymerase, oft <mark style="background: #FFB86CA6;">Taq-Polymerase</mark> genannt, stammt aus thermophilen (hitzeresistenten) Bakterien und bleibt auch bei hohen Temperaturen aktiv. Das ist wichtig, weil die PCR hohe Temperaturen verwendet, um die DNA zu denaturieren (in Einzelstränge zu trennen).
- **Thermoblock (Thermocycler)**: Das Gerät führt die Temperaturzyklen (Denaturierung, Primer-Anlagerung, Elongation) automatisch und präzise durch. Da die PCR viele Zyklen benötigt, um die DNA zu vervielfältigen, ist ein zuverlässiges, genau gesteuertes Heizen und Abkühlen unerlässlich.
## Schritte
![[Pasted image 20241112093922.png#invert|500]]
### Denaturierung
- Die DNA-Probe wird erhitzt (auf etwa 94–98 °C), um die beiden Stränge der doppelsträngigen DNA zu trennen. Die <mark style="background: #FFB86CA6;">Hitze löst die Wasserstoffbrückenbindungen</mark> zwischen den Basenpaaren auf und erzeugt so zwei Einzelstränge.
### Primer-Annealing
- Die Temperatur wird auf etwa 50–65 °C gesenkt, sodass die sogenannten Primer (kleine DNA-Stücke, die zu den spezifischen Zielregionen passen) an die Einzelstränge binden können. Diese Primer sind notwendig, um die DNA-Polymerase (das Enzym, das DNA kopiert) an die richtige Stelle zu bringen.
- Es gibt 2 Arten von Primern: Forward- und Reverse-Primer
- Der Forward Primer ist identisch zum Forward Strang der DNA-Probe und der Reverse Primer zum Ende des Komplementär Strangs
#### Beispiel
5' TCCGCAT...TTCGCATT 3'
3' AGGCGTA...AAGCGTAA 5'
- Forward Primer: TCCGCAT
- Reverse Primer: AATGCGAA
- -> Immer in 5' 3' Richtung! Da so die clonierten Stränge synthetisiert werden
### Primer-Extension
- Die Temperatur wird auf etwa 72 °C erhöht, die <mark style="background: #FFB86CA6;">optimale Temperatur für die DNA-Polymerase</mark>. Dieses Enzym beginnt an den Primern, neue DNA-Stränge aufzubauen, indem es freie Nukleotide anlagert und so die komplementären Stränge zu den Einzelsträngen bildet.
![[Pasted image 20241112095057.png#invert|400]]
## Länge der clonierten Stränge
![[Pasted image 20241112095248.png#invert|300]]
![[Pasted image 20241112095323.png#invert|300]]

Im ersten Zyklus der PCR entstehen längere DNA-Stränge als die gewünschte Zielsequenz (Target-Sequenz), weil die DNA-Polymerase nicht weiß, wo sie aufhören soll. Sie beginnt an den Primern, baut Nukleotide an, bis sie den Strangende erreicht, und erstellt so Stränge, die etwas länger sind als die Zielsequenz.
### Im Detail
1. **Erster Zyklus**: Die DNA-Polymerase startet an einem Primer und synthetisiert den neuen Strang, bis sie an das Ende des Templates gelangt. Dabei werden also Bereiche hinter dem Zielbereich kopiert, was zu längeren Strängen führt.
2. **Zweiter Zyklus**: Ähnlich wie im ersten Zyklus entstehen jetzt Stränge, die an einem der beiden Enden die richtige Länge haben, aber am anderen Ende länger sind. Das heißt, es gibt Stränge, die teilweise auf die Zielsequenz zugeschnitten sind.
3. **Weitere Zyklen**: <mark style="background: #FFB86CA6;">Ab dem dritten Zyklus beginnen jedoch Kopien der Zielsequenz zu entstehen</mark>, die genau von Primer zu Primer reichen. Diese korrekt zugeschnittenen Stränge werden jetzt bevorzugt als Vorlage genutzt. 
<mark style="background: #FFB86CA6;">Mit jedem weiteren Zyklus steigt die Anzahl der korrekt zugeschnittenen Zielsequenzen exponentiell</mark>, während die längeren, <mark style="background: #FFB86CA6;">unspezifischen Stränge nur linear zunehmen</mark>. So wird die Zielsequenz mit zunehmender Zahl an Zyklen dominierend.
-> Nach 30 Zyklen haben nur 60 Stränge nicht die korrekte Größe!
![PCR - Polymerase Chain reaction](https://www.youtube.com/watch?v=ZmqqRPISg0g)
