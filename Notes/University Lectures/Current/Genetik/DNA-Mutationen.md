- Es entstehen bis zu 500.000 DNA-Schäden pro Zelle pro Tag
- -> Die meisten werden reparitert
- Gäbe es keine Schäden/Veränderungen gäbe es keine Evolution! Da das Erbgut konstant bliebe
## Mutationsklassen
### Punktmutationen
![[Pasted image 20241121130412.png#invert|500]]
- **Transitionen**:
    - Eine Base wird durch eine andere Base der gleichen Klasse ersetzt (Purin ↔ Purin oder Pyrimidin ↔ Pyrimidin).
    - Beispiel: Adenin (A) wird zu Guanin (G) oder Thymin (T) wird zu Cytosin (C).
- **Transversionen**:
    - Eine Base wird durch eine Base der anderen Klasse ersetzt (Purin ↔ Pyrimidin).
    - Beispiel: Adenin (A) wird zu Thymin (T) oder Guanin (G) wird zu Cytosin (C).
### Leserastermutationen
![[Pasted image 20241121130434.png#invert|500]]
- Diese Mutationen betreffen das Leseschema der DNA, was große Auswirkungen auf die Proteinproduktion haben kann.
- **Insertionen**:
    - Hinzufügen von Basen in die DNA-Sequenz, was das Leseraster verschiebt.
    - Beispiel: Ein zusätzliches „T“ wird eingefügt, wodurch die Lesung der nachfolgenden Codons gestört wird.
- **Deletionen**:
    - Entfernen von Basen aus der DNA-Sequenz, was ebenfalls das Leseraster verschiebt.
    - Beispiel: Eine Base wird entfernt, wodurch alle Codons nach der Mutation verschoben werden.
### Chromosomenmutationen
![[University Lectures/Current/Genetik/images/Untitled.png#invert|500]]
- Veränderungen in der Struktur einzelner Chromosomen.
- **Translokationen/Rearrangements**:
    - Ein Teil eines Chromosoms wird verschoben oder mit einem anderen Chromosom ausgetauscht.
- **Verluste (Deletionen)**:
    - Teile eines Chromosoms gehen verloren, was zu einer verringerten Genanzahl führen kann.
### Genommutationen
![[University Lectures/Current/Genetik/images/Untitled 1.png#invert|500]]
- Veränderungen der Anzahl der Chromosomen im Genom.
- **Aneuploidie**:
    - Einzelne Chromosomen fehlen oder sind zusätzlich vorhanden.
    - Beispiel: Trisomie 21 (Down-Syndrom), bei dem das 21. Chromosom dreifach vorhanden ist.
- **Polyploidie**:
    - Der komplette Chromosomensatz liegt mehrfach vor.
    - Beispiel: Bei Pflanzen kommt es oft vor, dass der Chromosomensatz verdoppelt oder vervielfacht wird.
## Entstehung von Mutationen
### Während der Replikation durch Falscheinbau von Basen
![[Pasted image 20241121130635.png#invert|500]]
- Nach der Strangteilung wird die Base falsch komplementiert
- Meistens wird der Falscheinbau aber vorher repariert
#### "Wobble" Basenpaare
- Auch falsche Basenpaare bilden OH-Brücken, diese nennt man dann Wobble Basenpaare
![[Pasted image 20241121131351.png|300]]
### Durch DNA-Schäden (Veränderung der kodierenden Eigenschaften der Basen)
![[Pasted image 20241121130837.png#invert|500]]
- **Mechanismus**:
    - **Deaminierung**: Eine chemische Reaktion, bei der eine Aminogruppe (-NH₂) von einer Base entfernt wird.
        - Cytosin (C) wird durch Deaminierung zu Uracil (U) umgewandelt.
        - Uracil ist normalerweise nicht in DNA vorhanden, was als "Schaden" erkannt werden kann.
    - Dieser Schaden kann zu einer Fehlpaarung führen: Uracil paart mit Adenin (A), was schließlich zu einer dauerhaften Mutation (C → T) führen kann.
- **Ablauf**:
    - **Schritt 1**: Cytosin wird deaminiert und in Uracil umgewandelt.
    - **Schritt 2**: Während der Replikation wird Uracil mit Adenin gepaart.
    - **Schritt 3**: In der nächsten Runde der DNA-Replikation wird Adenin mit Thymin (T) gepaart, wodurch die Mutation entsteht.
### Durch fehlerhafte Reparatur von DNA-Schäden
![[Pasted image 20241121130919.png#invert|500]]
- **Mechanismus**:
    - **Depurinierung**: Der Verlust einer Purinbase (Adenin oder Guanin) aus der DNA, was zu einer "apurinischen" Stelle (ohne Base) führt.
    - Ohne Reparatur kann diese Stelle während der DNA-Replikation zu einer falschen Basenpaarung führen.
- **Ablauf**:
    - **Schritt 1**: Verlust einer Purinbase, was eine beschädigte DNA-Sequenz hinterlässt.
    - **Schritt 2**: Während der Replikation wird die fehlende Base möglicherweise mit einer zufälligen Base aufgefüllt ("Bypass").
    - **Schritt 3**: Die zufällige Base kann zu einer falschen Basenpaarung führen, wodurch eine Mutation entsteht.
## Mutationsarten
- **Forward-Mutationen**:
    - Verlust einer Funktion, z. B. von lacZ⁺ zu lacZ⁻ (Funktion geht verloren).
    - Resistenzentwicklung, z. B. von strS (sensitive) zu strR (resistent).
- **Reversionen**:
    - Wiederherstellung einer Funktion, z. B. von lacZ⁻ zu lacZ⁺.
    - Oft nur bei Punktmutationen möglich.
## Mechanismus der postreplikativen Fehlpaarungreparatur bei E. coli
![[Pasted image 20241121131523.png#invert|500]]
1. **Postreplikative Fehlpaarungsreparatur**:
    - **MutS** erkennt Fehlpaarungen durch "Scanning".
    - **MutL** wird rekrutiert und dient als Vermittler zwischen MutS und MutH.
    - **MutH** bindet an hemimethylierte Stellen (erkennt den parentalen Strang an der Methylierung).
    - MutH erzeugt einen **Einzelstrangbruch (Nick)** im unmethylierten, neu synthetisierten Strang.
2. **Exonuklease-Aktivität**:
    - Eine **Exonuklease** (Exo I, Exo VII, oder RecJ) entfernt die DNA vom Bruch bis über die Fehlpaarung hinweg.
    - Helicase II und ATP unterstützen die Entfernung.
3. **DNA-Reparatur**:
    - Die Lücke wird durch die **DNA-Polymerase III** gefüllt und durch **DNA-Ligase** versiegelt.
### Strangunterscheidung
- **Dam-Methylierung**:
    - Methylierung an **GATC-Sequenzen** (durch Dam-Methylase) markiert den parentalen Strang.
    - Während der DNA-Replikation ist der neu synthetisierte Strang noch **unmethyliert** (hemimethyliert).
- **Fehlpaarungsreparatur-Zeitfenster**:
    - Die Unterscheidung zwischen parentalem und neuem Strang ist nur möglich, solange der neue Strang noch unmethyliert ist.
    - Dieses "Zeitfenster" ist entscheidend für die Fehlpaarungsreparatur.
- **MutH**:
    - Bindet spezifisch an hemimethylierte GATC-Stellen und erzeugt einen Einzelstrangbruch im unmethylierten Strang.
## Fehlpaarungsreparatur bei Eukaryoten
- **MutS-Homologe (MSH)**:
    - Erkennen Fehlpaarungen durch "Scanning".
    - Mehrere Varianten existieren, die auf verschiedene Fehlpaarungen oder Insertions-/Deletionsschleifen spezialisiert sind.
- **MutL-Homologe (MLH, PMS)**:
    - Vermitteln zwischen den Erkennungsproteinen (MutS-Homologe) und den nachfolgenden Reparaturenzymen.
- **Kein MutH-Homolog**:
    - Anders als in _E. coli_ fehlt bei Eukaryoten ein Homolog für MutH.
    - **Problem**: Eukaryoten haben keine Dam-Methylierung, sodass die Unterscheidung zwischen parentalem und neuem Strang schwieriger ist.
- **Strangunterscheidung bei Eukaryoten**:
    - Strangunterscheidung erfolgt vermutlich durch die Präsenz von Okazaki-Fragmenten oder durch Enden der Replikation.