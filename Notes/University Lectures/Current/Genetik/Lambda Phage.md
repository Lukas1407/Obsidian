## Überblick
- **Temperenter Bakteriophage**: Der Phage kann zwei Lebenszyklen durchlaufen – den **lytischen Zyklus** und den **lysogenen Zyklus**.
- **Genom**:
    - 48 kb lineare DNA.
    - Codiert für etwa 70 Proteine.
- **Struktur**: Besteht aus einem Kopf, der das Genom schützt, und einem Schwanz, der für die Bindung an die Wirtszelle verantwortlich ist.
## Lebenszyklus von Lambda: Lysogenie und Lytischer Zyklus
![[University Lectures/Current/Genetik/images/Untitled 4.png#invert|400]]
- **Infektion**:
    - Lambda injiziert sein Genom in das Bakterium _E. coli_.
    - Es folgen zwei mögliche Wege:
        - **Lytischer Zyklus**: Das Phagen-Genom wird repliziert und neue Phagen-Partikel werden produziert, bis die Wirtszelle platzt (Lyse).
        - **Lysogener Zyklus**: Das Phagen-Genom wird in das bakterielle Chromosom integriert und bleibt dort als **Prophage**.
- **Entscheidung zwischen Lysogenie und Lysis**:
    - Die Entscheidung hängt von Umweltbedingungen ab.
    - Der λ-Repressor (cI-Protein) fördert die Lysogenie, indem er die Expression von lytischen Genen blockiert.
- **Induktion**:
    - Stressfaktoren wie UV-Strahlung können den lysogenen Zustand beenden, indem der λ-Repressor abgebaut wird.
    - Der Phage tritt in den lytischen Zyklus ein.

## Integration des Bakteriophagen Lambda
- **Hauptidee**: Der Bakteriophage Lambda integriert sein Genom an eine **spezifische Stelle** im Genom von _E. coli_. Dies erfolgt über eine **integrative Rekombination**, bei der die Phagen-DNA und die bakterielle DNA durch spezielle Rekombinationsstellen verbunden werden.
![[Pasted image 20241128123422.png#invert|400]]
- **Schlüsselmechanismen**:
    - _Phagen-Rekombinationsstelle_: Eine spezifische Sequenz auf der Phagen-DNA.
    - _Bakterielle Rekombinationsstelle_: Die Zielsequenz auf dem bakteriellen Chromosom.
    - Integration führt zur Bildung eines **Prophagen**, bei dem die Phagen-DNA Teil des bakteriellen Genoms wird.
### Details
- Lambda integriert sich an eine **spezifische Stelle** im _E. coli_-Genom (attB-Stelle) durch **Integrative Rekombination**.
- **Mechanismus**:
    - **attP**: Stelle auf dem Phagen-Genom.
    - **attB**: Stelle auf dem bakteriellen Genom.
    - **Integrase** (ein Enzym): Vermittelt die Rekombination zwischen attP und attB.
#### attP und attB-Stellen:
- **attP** enthält Core-Elemente (z. B. C und C'), die Bindungsstellen für die λ-Integrase und andere Proteine wie IHF (Integration Host Factor) sind
- **attB** ist einfacher aufgebaut und enthält nur Core-Elemente (B und B').
#### Beteiligte Proteine:
- **Integrase (λ-Int)**: Katalysiert die Rekombination.
- **IHF**: Krümmt die DNA, sodass attP und attB näher zueinander gebracht werden.
- **Xis-Protein** (für Exzision): Unterstützt die Entfernung des Prophagen bei der Rückkehr in den lytischen Zyklus.
#### DNA-Krümmung durch IHF
   - Die DNA wird durch IHF stark gekrümmt, was die Bindung der λ-Integrase an die attP- und attB-Stellen erleichtert.
### **Exzision des Phagen-Genoms**
- Bei der Induktion des lytischen Zyklus muss das λ-Genom aus dem bakteriellen Chromosom entfernt werden.
- **Mechanismus**:
    - **attL** und **attR** (veränderte Stellen nach der Integration) werden durch die λ-Integrase und das Xis-Protein rekombiniert.
    - Das Phagen-Genom wird freigesetzt.
## Anwendung der $\lambda$ Rekombination: Gateway Cloning
Gateway Cloning ist eine effiziente und hochspezifische Methode zur Klonierung und Expression von Genen, basierend auf dem λ-Rekombinationssystem. Dieses System ermöglicht den Austausch von DNA-Segmenten zwischen Plasmiden unter Verwendung von **attP**, **attB**, **attL**, und **attR**-Sequenzen.
### Grundlagen der Reaktion
#### **BP-Reaktion (attB × attP → attL + attR)**:
- Ein **Donorvektor** mit **attP**-Stellen wird mit einem DNA-Fragment (Open Reading Frame, ORF) kombiniert, das mit **attB**-Sequenzen flankiert ist.
- Ergebnis: Bildung eines **Entry-Klons** (Plasmid mit attL und attR-Stellen).
#### **LR-Reaktion (attL × attR → attB + attP)**:
- Der Entry-Klon wird mit einem **Zielvektor (Destination Vector)** kombiniert, der **attR**-Stellen enthält.
- Ergebnis: Bildung eines **Expressionsklons**, der die Ziel-DNA für die Expression in verschiedenen Systemen enthält.
### Plasmidtypen
- **Donorvektor**: Enthält **attP**-Stellen und Gene wie **ccdB**, das das Wachstum von Bakterien ohne rekombinierte DNA verhindert.
- **Entry-Klon**: Ergebnis der BP-Reaktion, enthält **attL**-Stellen, die für die LR-Reaktion verwendet werden.
- **Destination-Vektor**: Enthält **attR**-Stellen und Promotoren für die Genexpression in einem Zielorganismus.
- **Expressionsklon**: Das finale Produkt der LR-Reaktion, enthält die ORF-Sequenz an der richtigen Position für die Expression.
### **Kontrollmechanismus – ccdB-System**
- **ccdB-Gen** (Control of Cell Death B):
    - Das ccdB-Gen hemmt die Gyrase in Bakterien, wodurch Bakterien mit nicht-rekombinierten Vektoren absterben.
    - Nur Bakterien mit rekombinierten Plasmiden (korrekt inserierte DNA) können überleben.
- **ccdA-Gen**: Hemmt die Wirkung von ccdB und schützt Plasmide während der Klonierung.
### **Anwendung in der Forschung**
#### **Vielseitige Einsatzmöglichkeiten (Diagramm D)**:
- Gateway Cloning ermöglicht das schnelle Klonieren von Genen in verschiedene Vektoren für:
    - **Proteinexpression**: Insertion von Tags wie His6 oder GST für die Reinigung.
    - **Zelllinien-Expression**: Verwendung von CMV-Promotoren oder Baculovirus-Systemen.
    - **Funktionsanalysen**: Reporter wie GFP oder für Zwei-Hybrid-Systeme zur Protein-Protein-Interaktionsanalyse.
#### **Toxische Gene (Diagramm C)**:
- Die Methode kann für Gene verwendet werden, deren Expression in Standardbakterien toxisch ist. Durch den Einsatz spezifischer Kontrollvektoren (RC Donor Vector) wird sichergestellt, dass toxische Gene nicht spontan exprimiert werden.
### **Vorteile von Gateway Cloning**
- **Einfachheit und Effizienz**:
    - Keine Restriktionsenzyme oder Ligase erforderlich.
    - Wiederverwendbare Vektoren und standardisierte Prozesse.
- **Flexibilität**:
    - Gene können schnell in unterschiedliche Expressionssysteme kloniert werden.
- **Spezifität**:
    - Verwendung spezifischer Sequenzen (att-Stellen) minimiert Fehler.