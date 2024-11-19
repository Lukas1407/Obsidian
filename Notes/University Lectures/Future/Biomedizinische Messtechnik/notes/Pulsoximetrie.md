> [!summary] Definition
>  Die Pulsoximetrie ist ein nicht-invasives Verfahren zur Ermittlung der arteriellen Sauerstoffsättigung des Blutes. Dabei wird ein kleines Gerät, das Pulsoximeter, meist an der Fingerspitze oder am Ohrläppchen angebracht. Es misst die Sauerstoffsättigung, indem es Licht verschiedener Wellenlängen durch die Haut sendet und die Absorption dieses Lichts durch das Blut analysiert.
>  - Puls: [[Plethysmographie|Photoplethysmographische]] Messung des arteriellen Volumenpulses 
>  - Oxymetrie: [[Photometrie|Spektralphotometrische]] Messung des an das Hämoglobin gebundenen Sauerstoffs

## Messprinzip
Das Messprinzip basiert darauf, dass oxygeniertes Hämoglobin (HbO2) – also mit Sauerstoff beladenes Hämoglobin – und desoxygeniertes Hämoglobin (Hb) – Hämoglobin, dessen Transportplätze für Sauerstoff noch frei sind – das Licht unterschiedlich absorbieren. Bei einer Wellenlänge von etwa 680 nm absorbiert desoxygeniertes Hämoglobin mehr Licht als oxygeniertes Hämoglobin, was das Pulsoximeter nutzt, um die Sauerstoffsättigung zu berechnen
## Kalibrierung von Pulsoximetern
Die **Kalibrierung von Pulsoximetern** ist ein wichtiger Prozess, um die Genauigkeit der Geräte zu gewährleisten, die zur Messung der Sauerstoffsättigung im Blut verwendet werden. Hier ist eine detaillierte Erklärung des Kalibrierungsprozesses:
1. **Einstellung unterschiedlicher Sauerstoff-Mischungsverhältnisse in der Atemluft**: Um die Genauigkeit des Pulsoximeters über einen breiten Bereich von Sauerstoffsättigungswerten zu testen, atmen Probanden Luftgemische mit verschiedenen Sauerstoffkonzentrationen ein. Dies simuliert unterschiedliche Sauerstoffsättigungslevel im Blut.
2. **Arterielle Blutentnahme**: Während die Probanden die verschiedenen Sauerstoffmischungen atmen, wird arterielles Blut entnommen. Arterielles Blut gibt den genauesten Wert der Sauerstoffsättigung wieder, da es direkt aus den Arterien kommt, die das sauerstoffreiche Blut vom Herzen weg transportieren.
3. **Registrierung der Pulsoximetrie-Parameter an speziellen Kalibriersystemen**: Gleichzeitig mit der Blutentnahme werden die Werte des Pulsoximeters aufgezeichnet. Diese Werte werden dann mit den tatsächlichen Sauerstoffsättigungswerten aus der Blutgasanalyse verglichen.
4. **Durchführung der Blutgasanalyse an der arteriellen Blutprobe**: Die entnommenen Blutproben werden analysiert, um die tatsächliche Sauerstoffsättigung zu bestimmen. Diese Analyse liefert die Referenzwerte, mit denen die Pulsoximeterdaten verglichen werden.
5. **Erstellung einer Kalibriertabelle für die Kalibriersysteme (Gold standards)**: Die Ergebnisse der Blutgasanalyse werden verwendet, um eine Kalibriertabelle zu erstellen. Diese Tabelle dient als Referenz (Goldstandard) für die Kalibrierung der Pulsoximeter.
**Entsättigung unter 70%**: Es ist bekannt, dass die Sauerstoffbindungskurve des Hämoglobins eine steile Flanke bei niedrigen Sauerstoffsättigungswerten hat. Dies bedeutet, dass kleine Änderungen im Partialdruck des Sauerstoffs zu großen Änderungen in der Sauerstoffsättigung führen können. Entsättigungswerte unter 70% sind daher schwer zu stabilisieren und erfordern eine sorgfältige Überwachung, um sicherzustellen, dass der Proband nicht gefährdet wird. Aus diesem Grund wird die Entsättigung unter ständiger Kontrolle eines Anästhesisten durchgeführt

## Messkette
Die **Messkette** beschreibt den gesamten Prozess von der Erfassung des Lichtsignals bis zur Anzeige der Messwerte:
1. **Lichtemission**: LEDs senden Licht aus, das durch das Gewebe geht.
2. **Lichtdetektion**: Die Photodiode registriert das durchgelassene oder reflektierte Licht.
3. **Signalwandlung**: Das Lichtsignal wird in ein elektrisches Signal umgewandelt.
4. **Signalverarbeitung**: Der Mikrocontroller verarbeitet das Signal und berechnet die Sauerstoffsättigung.
5. **Anzeige**: Die berechneten Werte werden auf einem Display angezeigt, typischerweise als Prozentsatz der Sauerstoffsättigung und Pulsfrequenz