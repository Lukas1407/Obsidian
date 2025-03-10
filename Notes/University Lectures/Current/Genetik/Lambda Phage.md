## Überblick
- **Temperenter Bakteriophage**: Der Phage kann zwei Lebenszyklen durchlaufen – den **lytischen Zyklus** und den **lysogenen Zyklus**.
- **Genom**:
    - 48 kb lineare DNA.
    - Codiert für etwa 70 Proteine.
- **Struktur**: Besteht aus <mark style="background: #FFB86CA6;">einem Kopf, der das Genom schützt</mark>, und <mark style="background: #FFB86CA6;">einem Schwanz, der für die Bindung an die Wirtszelle</mark> verantwortlich ist.
## Lebenszyklus von Lambda: Lysogenie und Lytischer Zyklus
![[University Lectures/Current/Genetik/images/Untitled 4.png#invert|400]]
- **Infektion**:
    - Lambda injiziert sein Genom in das Bakterium _E. coli_.
    - Es folgen zwei mögliche Wege:
        - **Lytischer Zyklus**: Das Phagen-Genom wird repliziert und neue Phagen-Partikel werden produziert, bis die Wirtszelle platzt (Lyse).
        - **Lysogener Zyklus**: Das Phagen-Genom wird in das bakterielle Chromosom integriert und bleibt dort als **Prophage**.
- **Entscheidung zwischen Lysogenie und Lysis**:
    - Die Entscheidung hängt von Umweltbedingungen ab.
    - Der <mark style="background: #FFB86CA6;">λ-Repressor (cI-Protein) fördert die Lysogenie, indem er die Expression von lytischen Genen blockiert</mark>.
- **Induktion**:
    - <mark style="background: #FFB86CA6;">Stressfaktoren wie UV-Strahlung</mark> können den lysogenen Zustand beenden, <mark style="background: #FFB86CA6;">indem der λ-Repressor abgebaut</mark> wird.
    - Der Phage tritt in den lytischen Zyklus ein.

## Integration des Bakteriophagen Lambda
- **Hauptidee**: Der Bakteriophage Lambda integriert sein Genom an eine **spezifische Stelle** im Genom von _E. coli_. Dies erfolgt über eine **integrative Rekombination**, bei der die Phagen-DNA und die bakterielle DNA durch spezielle Rekombinationsstellen verbunden werden.
![[Pasted image 20241128123422.png#invert|400]]
- **Schlüsselmechanismen**:
    - <mark style="background: #FFB86CA6;">Phagen-Rekombinationsstelle</mark>: Eine spezifische Sequenz auf der Phagen-DNA.
    - <mark style="background: #FFB86CA6;">Bakterielle Rekombinationsstelle</mark>: Die Zielsequenz auf dem bakteriellen Chromosom.
    - Integration führt zur <mark style="background: #FFB86CA6;">Bildung eines Prophagen, bei dem die Phagen-DNA Teil des bakteriellen Genoms wird.</mark>
### Details
- Lambda integriert sich an eine **spezifische Stelle** im _E. coli_-Genom (attB-Stelle) durch **Integrative Rekombination**.
- **Mechanismus**:
    - **attP**: <mark style="background: #FFB86CA6;">Stelle auf dem Phagen-Genom</mark>.
    - **attB**: <mark style="background: #FFB86CA6;">Stelle auf dem bakteriellen Genom</mark>.
    - **Integrase** (ein Enzym): Vermittelt die <mark style="background: #FFB86CA6;">Rekombination zwischen attP und attB</mark>.
#### attP und attB-Stellen:
- **attP** <mark style="background: #FFB86CA6;">enthält Core-Elemente</mark> (z. B. C und C'), die <mark style="background: #FFB86CA6;">Bindungsstellen für die λ-Integrase und andere Proteine wie IHF</mark> (Integration Host Factor) sind
- **attB** ist <mark style="background: #FFB86CA6;">einfacher aufgebaut</mark> und enthält <mark style="background: #FFB86CA6;">nur Core-Elemente</mark> (B und B').
#### Beteiligte Proteine:
- **Integrase (λ-Int)**: <mark style="background: #FFB86CA6;">Katalysiert die Rekombination</mark>.
- **IHF**: <mark style="background: #FFB86CA6;">Krümmt die DNA, sodass attP und attB näher zueinander gebracht werden</mark>.
- **Xis-Protein** (<mark style="background: #FFB86CA6;">für Exzision</mark>): Unterstützt die Entfernung des Prophagen bei der <mark style="background: #FFB86CA6;">Rückkehr in den lytischen Zyklus</mark>.
#### DNA-Krümmung durch IHF
   - Die DNA wird durch IHF stark gekrümmt, was die Bindung der λ-Integrase an die attP- und attB-Stellen erleichtert.
### **Exzision des Phagen-Genoms**
- Bei der Induktion des <mark style="background: #FFB86CA6;">lytischen Zyklus muss das λ-Genom aus dem bakteriellen Chromosom entfernt werden</mark>.
- **Mechanismus**:
    - **attL** und **attR** (<mark style="background: #FFB86CA6;">veränderte Stellen nach der Integration</mark>) werden durch die λ-Integrase und das Xis-Protein rekombiniert.
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