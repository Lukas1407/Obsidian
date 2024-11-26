Spektrales Rendering erweitert klassische RGB-Modelle, indem es Licht als kontinuierliches Spektrum behandelt.
1. **Unterschied zu RGB-Rendering**:
    - RGB-Rendering arbeitet mit drei Farbkanälen (Rot, Grün, Blau), die das sichtbare Licht approximieren.
    - Spektrales Rendering verwendet ein feiner aufgelöstes Spektrum, z. B. in 5 nm-Schritten, und berücksichtigt die Wellenlängenabhängigkeit von Emission, Reflektanz und Lichtmaterialinteraktionen.
2. **Vorteile von spektralem Rendering**:
    - **Genauigkeit**:
        - Exakte Reproduktion von Dispersion (z. B. bei Glas und Regenbögen), Fluoreszenz und spezifischen Materialeigenschaften.
    - **Realistischere Farben**:
        - Natürliche Farbwahrnehmung wird besser modelliert, insbesondere bei mehrfacher Reflexion und subtilen Farbverschiebungen.
3. **Herausforderungen**:
    - **Rechenaufwand**:
        - Höherer Aufwand für Berechnungen und Speicherbedarf, da mehr Daten verarbeitet werden müssen.
    - **Komplexität**:
        - Spektrale Texturen und Lichtquellen benötigen mehr Speicherplatz.
4. **Beispiel für spektrale Effekte**:
    - Das Produkt aus Emissionsspektrum (Lichtquelle), Reflektanzspektrum (Materialoberfläche) und den CIE-Farbmatching-Funktionen (Wahrnehmungsmodell) ergibt die genaue Lichtverteilung.
### **Vergleich Tristimulus- und spektrales Rendering**
1. **Tristimulus (RGB)**:
    - Einfachere Darstellung mit nur drei Werten.
    - Bei mehrfacher Reflexion (n>1n > 1) treten jedoch Fehler auf, da Farbverschiebungen nicht korrekt modelliert werden.
2. **Spektral**:
    - Natürliche Reflexionsspektren sind oft glatt, was präzisere Berechnungen erlaubt.
    - Stärkere physikalische Grundlage durch Wellenlängenabhängigkeit.
### **Fluoreszenz und gesättigte Farben**
1. **Fluoreszenz**:
    - Fluoreszierende Materialien können gesättigte Farben erzeugen, die durch einfache Reflektanzmodelle nicht abgedeckt werden.
    - Simulation erfordert erweiterte spektrale Modelle.
2. **Forschungsthema**:
    - Die Simulation solcher Effekte ist ein aktives Forschungsgebiet, da viele physikalische Wechselwirkungen schwer modellierbar sind.
