> [!abstract] Definition
>  Die Photometrie ist ein Messverfahren, bei dem die Konzentration von gelösten Substanzen mithilfe eines Photometers bestimmt wird. In der Regel verwendet man dafür Lösungen von farbigen Verbindungen. Die häufigste Methode ist die Messung über die Transmission, also wie viel Licht durch ein optisches Medium hindurchgeht

## Lambert-Beer'sche Gesetz
- Das **Lambert-Beer’sche Gesetz** ist ein fundamentales Prinzip in der Photometrie, das die Wechselwirkung von Licht mit Materie beschreibt. Es erklärt, wie die Intensität des Lichts abnimmt, wenn es durch ein Medium mit einer absorbierenden Substanz hindurchgeht. Hier ist eine genauere Erklärung:
- Die Intensität ( I ) des Lichts, das durch ein Medium strahlt, wird durch die absorbierende Substanz ( A ) geschwächt. Diese Abschwächung hängt von zwei Faktoren ab:
	1. Der **Konzentration ( c )** der absorbierenden Substanz im Medium.
	2. Der **Schichtdicke ( d )** des Mediums, durch das das Licht hindurchgeht.
- Das Gesetz lässt sich mathematisch durch die folgende Formel ausdrücken:$$I=I_{0}\cdot10^{-\epsilon\cdot c \cdot d}$$
- Hierbei ist:
	- $I_{0}$ die Intensität des einfallenden Lichts vor dem Eintritt in das Medium.
	- $\epsilon$ der molarer Absorptionskoeffizient oder Extinktionskoeffizient, eine substanzspezifische Konstante, die angibt, wie stark die Substanz das Licht bei einer bestimmten Wellenlänge absorbiert.
	- $c$ die Konzentration der absorbierenden Substanz.
	- $d$ die Schichtdicke des Mediums.

## Photometrische Bestimmung der Hämoglobinkonzentrationen im [[Gewebe]]
- Die photometrische Bestimmung der Hämoglobinkonzentrationen im Gewebe ist aus mehreren Gründen schwierig:
	- **Streuverluste vs. Absorption**: In biologischem Gewebe ist die Streuung von Licht oft stärker als die Absorption. Das bedeutet, dass ein Großteil des Lichts in verschiedene Richtungen gestreut wird, bevor es absorbiert werden kann. Dies erschwert die genaue Messung der Lichtabsorption, die für die Bestimmung der Hämoglobinkonzentration notwendig ist.
	- **Optische Weglänge und Schichtdicke**: Die optische Weglänge bezieht sich auf den tatsächlichen Pfad, den das Licht durch das Gewebe zurücklegt. Aufgrund der Streuung ist dieser Pfad oft länger als die physische Schichtdicke des Gewebes. Dies führt zu einer weiteren Komplikation bei der Anwendung des Lambert-Beer’schen Gesetzes, da die tatsächliche Weglänge des Lichts schwer zu bestimmen ist.
	- **Gewebeabsorption**: Gewebe hat eine signifikante Eigenabsorption im untersuchten Spektralbereich, was bedeutet, dass nicht nur das Hämoglobin, sondern auch andere Bestandteile des Gewebes das Licht absorbieren. Dies kann die Messergebnisse verfälschen, da es schwierig ist, die Absorption durch Hämoglobin von der Gesamtabsorption zu unterscheiden.
	- **Transmissions- oder Remissionsmessung**: Diese Messungen finden außerhalb des Gültigkeitsbereichs des Lambert-Beer’schen Gesetzes statt, weil die oben genannten Faktoren die lineare Beziehung zwischen Absorption und Konzentration stören. Daher erfordern sie eine aufwändige Kalibrierung, um genaue Ergebnisse zu liefern.
	- **Spektralphotometrische Bestimmung der Sauerstoffsättigung**: Diese Methode erfasst das gesamte gemischtvenöse Blutvolumen, was bedeutet, dass sie die Sauerstoffsättigung des Blutes misst, das aus verschiedenen Körperteilen zurück zum Herzen fließt. Dies kann nützlich sein, um einen Überblick über die Sauerstoffversorgung des Körpers zu erhalten.

## Anwendung: Cerebral Oximetry ([[Nahinfrarotspektroskopie (NIRS)|NIRS]])
- Die Nahinfrarotspektroskopie (NIRS) wird für die zerebrale Oximetrie verwendet, um die Sauerstoffsättigung im Gehirn zu messen. 
- Sie nutzt das “optische Fenster”, bei dem Licht einer bestimmten Wellenlänge tief in das Gewebe eindringen kann, ohne stark absorbiert zu werden. 
- Die interne Referenzierung über unterschiedliche Streupfade hilft dabei, die Messungen zu korrigieren und genauer zu machen

## Spektral-photometrische Auswertung des Volumenpulses
Die spektralphotometrische Auswertung des Volumenpulses ist ein interessantes Verfahren, das auf der Wechselwirkung von Licht mit durchblutetem Gewebe basiert. Hier sind die Erklärungen zu den beiden Beobachtungen:
1. **Pulssynchroner Signalanteil**:
    - Bei der optischen Transmissions- oder Remissionsmessung an durchblutetem Gewebe beobachtet man einen zeitlich veränderlichen Signalanteil, der synchron zum Herzschlag pulsiert.
    - Dieser pulssynchrone Anteil im optischen Signal entspricht der **Volumenänderung im arteriellen Gefäßsystem**, die durch den Herzschlag verursacht wird.
    - Wenn das Herz schlägt, dehnen sich die Arterien aus (systolische Phase) und ziehen sich zusammen (diastolische Phase). Diese Volumenänderungen beeinflussen die Lichtabsorption im Gewebe.
2. **Abhängigkeit der optischen Pulsamplitude von der Wellenlänge**:
    - Die optische Pulsamplitude, also die Variation der Lichtintensität im Takt des Herzschlags, hängt von der **Wellenlänge des verwendeten Lichts** ab.
    - Der **Absorptionskoeffizient** im pulsatilen Volumenanteil (durchblutetes Gewebe) variiert mit der Wellenlänge. Unterschiedliche Wellenlängen werden unterschiedlich stark absorbiert.
    - Durch die Messung der Pulsamplitude bei **zwei verschiedenen Wellenlängen** kann eine spektralphotometrische Oxymetrie im arteriellen Gefäßsystem erfolgen. Dies ermöglicht die Bestimmung der **Sauerstoffsättigung** im Blut.
**Wichtig**:
- Da die Werte für den Absorptionskoeffizienten und die optische Weglänge im Körpergewebe nicht konstant sind, wird die **Pulsoximetrie empirisch kalibriert**. Das bedeutet, dass die Geräte anhand bekannter Referenzwerte und Messungen justiert werden, um genaue Ergebnisse zu liefern.

## Photmetrische Messtechnik
Die photometrische Messtechnik in der Pulsoximetrie nutzt die unterschiedlichen Absorptionseigenschaften von oxygeniertem und desoxygeniertem Hämoglobin. Ein Pulsoximeter sendet Licht durch die Haut und misst, wie viel Licht von diesen beiden Formen des Hämoglobins absorbiert wird. Die Messung erfolgt typischerweise an gut durchbluteten Körperstellen wie dem Finger oder Ohrläppchen
## Systemarchitektur
Die **Systemarchitektur** eines Pulsoximeters besteht aus mehreren Komponenten:
- **LEDs**: Diese senden Licht in zwei Wellenlängen aus, um die Sauerstoffsättigung zu messen.
- **Photodiode**: Sie detektiert das durch das Gewebe hindurchgegangene oder reflektierte Licht.
- **Strom-Spannungs-Wandler**: Wandelt den von der Photodiode erzeugten Strom in ein Spannungssignal um.
- **Impedanzwandler (Buffer)**: Stabilisiert das Signal für eine genaue Messung.
- **A/D-Wandler**: Konvertiert das analoge Signal in ein digitales Signal für die Verarbeitung.
- **Mikrocontroller**: Verarbeitet die digitalen Signale und steuert die Funktionen des Pulsoximeters.
- **D/A-Wandler**: Wird für die Ansteuerung von Schaltern oder analogen Regelschleifen verwendet