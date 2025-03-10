Die **Genomsequenzierung** ist ein Prozess, bei dem die <mark style="background: #FFB86CA6;">genaue Reihenfolge der Nukleotide in der DNA eines Organismus bestimmt wird</mark>.
## Ablauf
### **a) DNA-Extraktion**
Die <mark style="background: #FFB86CA6;">DNA wird aus Zellen</mark> des Organismus isoliert.
### **b) Fragmentierung**
Die DNA wird in <mark style="background: #FFB86CA6;">kleine Stücke zerlegt</mark>, da lange DNA-Stränge schwer direkt zu analysieren sind.
### **c) Sequenzierung**
Die Fragmente werden mit <mark style="background: #FFB86CA6;">speziellen Technologien</mark> analysiert. Wichtige Methoden:
- **Kettenspaltung-Methode**
- **Sanger-Sequenzierung**: 
- **Next-Generation Sequencing (NGS)**: 
- **Third-Generation Sequencing**: Noch schnellere und präzisere Methoden, die lange DNA-Fragmente in einem Schritt sequenzieren.
### **d) Datenanalyse**
- Die sequenzierten Fragmente werden mithilfe von Computern zu einer vollständigen DNA-Sequenz zusammengesetzt.

## Kettenspaltung-Methode (Maxam-Gilbert Sequenzierung)
1. **Markierung der DNA:**
    - Ein <mark style="background: #FFB86CA6;">DNA-Strang wird am 5'-Ende mit einem radioaktiven Marker</mark> (z. B. Phosphor-32) markiert.
2. **Fragmentierung:**
    - Die markierte DNA wird <mark style="background: #FFB86CA6;">in vier Reaktionsansätze aufgeteilt, die jeweils spezifische Basen (G, A+G, T+C, C) angreifen</mark>.
3. **Chemische Spaltung:**
    - <mark style="background: #FFB86CA6;">Spezifische Chemikalien schneiden die DNA an bestimmten Nukleotiden</mark>:
        - Tube 1: Spaltet nur Guanin (G).
        - Tube 2: Spaltet Adenin (A) und Guanin (G).
        - Tube 3: Spaltet Thymin (T) und Cytosin (C).
        - Tube 4: Spaltet nur Cytosin (C).
4. **Elektrophorese:**
    - Die DNA-Fragmente werden <mark style="background: #FFB86CA6;">nach Größe durch Gelelektrophorese aufgetrennt</mark>.
    - Jedes Band repräsentiert die Position eines spezifischen Basens im DNA-Strang.
5. **Auswertung:**
    - <mark style="background: #FFB86CA6;">Die Reihenfolge der Basen wird durch Ablesen der Bandenmuster bestimmt</mark> (z. B. mithilfe von Autoradiographie).
#### **Vorteile und Nachteile:**
- **Vorteile:**
    - Sehr präzise.
- **Nachteile:**
    - Aufwendig und arbeitsintensiv.
    - Benötigt gefährliche Chemikalien.
    - Wird heute kaum noch verwendet.
## Sanger-Methode (Dideoxy-Methode)
Sie <mark style="background: #FFB86CA6;">basiert auf dem Prinzip des Kettenabbruchs</mark> durch dideoxynukleotide (ddNTPs).
1. **DNA-Vorlage und Primer:**
    - Eine <mark style="background: #FFB86CA6;">einzelsträngige DNA wird mit einem Primer vorbereitet</mark>.
2. **Reaktionsansatz:**
    - <mark style="background: #FFB86CA6;">DNA-Polymerase synthetisiert den komplementären Strang</mark>.
    - <mark style="background: #FFB86CA6;">Neben normalen Nukleotiden (dNTPs) werden ddNTPs hinzugefügt</mark>. Diese ddNTPs <mark style="background: #FFB86CA6;">bewirken einen Kettenabbruch</mark>, da sie <mark style="background: #FFB86CA6;">keine 3'-OH-Gruppe besitzen</mark>.
3. **Farbcodierte ddNTPs:**
    - Jedes <mark style="background: #FFB86CA6;">ddNTP ist mit einem fluoreszierenden Farbstoff</mark> markiert, sodass <mark style="background: #FFB86CA6;">die Abbruchprodukte identifizierbar</mark> sind.
4. **Elektrophorese:**
    - Die entstandenen DNA-Fragmente werden durch Kapillarelektrophorese nach <mark style="background: #FFB86CA6;">Größe aufgetrennt</mark>.
    - <mark style="background: #FFB86CA6;">Ein Laser liest die fluoreszierenden Signale aus, und die Basenreihenfolge wird bestimmt</mark>.
#### **Vorteile und Nachteile:**
- **Vorteile:**
    - Sehr zuverlässig und genau.
    - Ideal für kürzere Sequenzen.
- **Nachteile:**
    - Langsam im Vergleich zu modernen Methoden wie NGS (Next-Generation Sequencing).
    - Teurer für große Genome.
## Pyrosequencing
- <mark style="background: #FFB86CA6;">basiert auf der Detektion von freigesetztem Pyrophosphat</mark> ($PPi_ii$​), das <mark style="background: #FFB86CA6;">während der DNA-Synthese entsteht</mark>.
- Jedes Mal, <mark style="background: #FFB86CA6;">wenn ein dNTP (Desoxynukleotid) eingebaut wird</mark>, wird $PPi_ii$​ freigesetzt.
1. **DNA-Synthese:**
    - Ein <mark style="background: #FFB86CA6;">einzelsträngiger DNA-Abschnitt wird durch eine Polymerase verlängert</mark>.
    - Die Nukleotide (dATP, dTTP, dGTP, dCTP) werden nacheinander in den Reaktionsansatz gegeben.
2. **Freisetzung von Pyrophosphat (PPi_ii​):**
    - <mark style="background: #FFB86CA6;">Beim Einbau</mark> eines Nukleotids durch die DNA-Polymerase <mark style="background: #FFB86CA6;">wird PPi_ii</mark>​ freigesetzt.
3. **ATP- und Lichtproduktion:**
    - <mark style="background: #FFB86CA6;">ATP-Sulfurylase wandelt PPi_ii​ in ATP um</mark>.
    - <mark style="background: #FFB86CA6;">Das ATP wird durch Luciferase genutzt, um Licht zu erzeugen</mark>.
    - Die <mark style="background: #FFB86CA6;">Lichtintensität ist proportional zur Menge an eingebautem Nukleotid</mark>.
4. **Signalaufzeichnung:**
    - Die <mark style="background: #FFB86CA6;">Lichtsignale werden durch einen Detektor erfasst und in eine Nukleotidsequenz umgewandelt</mark>.
**Vorteile:**
- Echtzeit-Erkennung der DNA-Synthese.
- Hohe Genauigkeit.
**Nachteile:**
- Sequenzlänge ist begrenzt (300–500 Basen).
- Relativ teuer im Vergleich zu neueren Technologien.
## “454” Technologie
eine der <mark style="background: #FFB86CA6;">ersten Anwendungen von Pyrosequencing auf eine NGS-Plattform</mark>
**Grundprinzip:**
- <mark style="background: #FFB86CA6;">Verwendet Pyrosequencing auf Mikroperlen (Beads), die in Emulsionen oder Wells platziert sind</mark>.
- Ermöglicht die <mark style="background: #FFB86CA6;">parallele Sequenzierung von Millionen DNA-Fragmente</mark>.
**Ablauf:**
1. **Fragmentierung und Adaption:**
    - <mark style="background: #FFB86CA6;">DNA wird in kleine Fragmente geschnitten</mark>, und <mark style="background: #FFB86CA6;">spezifische Adapter werden an den Enden angebracht</mark>.
2. **Amplifikation (emPCR):**
    - Die <mark style="background: #FFB86CA6;">DNA-Fragmente werden an Beads gebunden</mark>.
    - Die Beads <mark style="background: #FFB86CA6;">werden in einer Wasser-in-Öl-Emulsion amplifiziert</mark>, sodass <mark style="background: #FFB86CA6;">jedes Bead viele Kopien des gleichen DNA-Fragments trägt</mark>.
3. **Sequenzierung in Wells:**
    - <mark style="background: #FFB86CA6;">Die Beads werden in Wells auf einer Platte verteilt</mark>.
    - <mark style="background: #FFB86CA6;">Pyrosequencing wird auf jedem Bead durchgeführt</mark>.
    - Enzyme wie Luciferase und ATP-Sulfurylase unterstützen die Reaktion.
4. **Lichtdetektion:**
    - Lichtsignale von jedem Well werden durch eine CCD-Kamera erfasst.
    - Die Intensität und Reihenfolge der Lichtsignale werden in DNA-Sequenzen übersetzt.
## Next Generation Sequencing (NGS)
beschreibt eine Gruppe <mark style="background: #FFB86CA6;">moderner DNA-Sequenzierungstechnologien</mark>, die <mark style="background: #FFB86CA6;">Millionen von DNA-Fragmenten parallel sequenzieren können</mark>.
### **Grundlegender Ablauf der NGS**
1. **Library Preparation (Bibliotheksvorbereitung):**
    - **Fragmentierung der DNA:** Die DNA wird in <mark style="background: #FFB86CA6;">kleine Fragmente zerlegt</mark>.
    - **Adapter-Ligation:** <mark style="background: #FFB86CA6;">Spezielle Adapter werden an die Fragmente ligiert</mark>, um sie für die Sequenzierung vorzubereiten.
2. **Cluster Amplification:**
    - Die DNA-Bibliothek wird in einer **Flow Cell** verteilt, und <mark style="background: #FFB86CA6;">einzelne Fragmente werden durch Bridge Amplification in Cluster umgewandelt</mark>. <mark style="background: #FFB86CA6;">Jedes Cluster enthält viele Kopien desselben DNA-Fragments</mark>.
3. **Sequenzierung:**
    - Die DNA wird Zyklus für Zyklus <mark style="background: #FFB86CA6;">sequenziert</mark>.
    - <mark style="background: #FFB86CA6;">Fluoreszenzfarbstoffe oder chemische Signale markieren die eingebauten Nukleotide</mark>, die optisch oder chemisch erfasst werden.
4. **Datenanalyse:**
    - Die resultierenden "Reads" werden mithilfe von Bioinformatik-Tools mit einer Referenz-DNA verglichen.
    - Unterschiede, Mutationen oder spezifische Merkmale werden identifiziert.
## Humangenomprojekt (HGP)
### **Ziele des Projekts**
1. **Identifikation aller menschlichen Gene**:
    - Schätzung: 20.000–25.000 Gene.
2. **Bestimmung der Sequenz von 3 Milliarden Basenpaaren**:
    - Vollständige DNA-Sequenz des menschlichen Genoms.
3. **Speicherung der Informationen**:
    - Aufbau von Datenbanken zur Archivierung und Verteilung.
4. **Entwicklung neuer Technologien**:
    - Verbesserung von Werkzeugen zur Datenanalyse.
5. **Behandlung ethischer, rechtlicher und sozialer Fragen**:
    - Umgang mit Datenschutz und genetischen Informationen.
### **Methoden**
1. **Sequenzierung**:
    - <mark style="background: #FFB86CA6;">Basierend auf der Sanger-Methode</mark>.
2. **Automatisierung**:
    - Einsatz automatisierter Sequenzierer (z. B. ABI 3700).
3. **Zusammenstellung des Genoms**:
    - Fragmentierte DNA wurde sequenziert und anschließend zusammengesetzt.
### **Ergebnisse**
1. **Einblicke in die Struktur des menschlichen Genoms**:
    - <mark style="background: #FFB86CA6;">Nur 2 % der DNA kodieren für Proteine</mark>.
    - <mark style="background: #FFB86CA6;">Der Rest besteht aus regulatorischen Elementen, Introns und "nicht-codierender" DNA</mark>.
2. **Genom-Ähnlichkeit**:
    - <mark style="background: #FFB86CA6;">97 % des Genoms sind bei allen Menschen identisch</mark>.
3. **Gene ungleich verteilt**:
    - Chromosom 1: 2.968 Gene.
    - Y-Chromosom: 231 Gene.
4. **Transposons**:
    - <mark style="background: #FFB86CA6;">Etwa 50 % der DNA besteht aus Transposons</mark> (springenden Genen).
5. **Vergleich mit anderen Spezies**:
    - Mensch: 23.000 Gene.
    - Fruchtfliege (_Drosophila melanogaster_): 20.000 Gene.
- **Genomgröße:** Das menschliche Genom besteht aus über drei Milliarden Nukleotiden, aber nur **5% kodieren Proteine.**
- **Transposable Elemente:** Mindestens **50%** des Genoms besteht aus "springenden Genen" (Transposons).
- **Gene ohne Funktion:** Mehr als 40% der identifizierten Gene haben keine bekannte molekulare Funktion.
- **Ungleiche Verteilung:** Gene sind ungleichmäßig auf die 24 Chromosomen verteilt.
- **Vergleich:** Menschliche Gene sind oft größer und enthalten mehr Introns (nicht-kodierende Abschnitte) als Gene von Nicht-Wirbeltieren (z. B. Fruchtfliegen).

## Sequenzierstrategien
#### **1. Hierarchical Shotgun Sequencing (HGP, Francis Collins)**
- **Vorgehen**:
    1. **BAC Library**: Zunächst wird das <mark style="background: #FFB86CA6;">Genom in größere Abschnitte fragmentiert und in einer BAC Bibliothek organisiert</mark>.
    2. **Contig Map**: <mark style="background: #FFB86CA6;">Diese größeren Abschnitte werden kartiert, um überlappende Bereiche zu identifizieren</mark>.
    3. **Shotgun Sequencing**: <mark style="background: #FFB86CA6;">Jeder Contig wird mittels Shotgun-Ansatz sequenziert</mark>.
    4. **Alignment**: Die <mark style="background: #FFB86CA6;">Contigs werden anschließend zusammengefügt, um die vollständige Genomsequenz</mark> zu erhalten.
- **Vorteile**:
    - Strukturierte Methode, die <mark style="background: #FFB86CA6;">weniger redundante Daten</mark> erzeugt.
    - <mark style="background: #FFB86CA6;">Geeignet für komplexe Genome mit vielen repetitiven Sequenzen</mark>.
- **Nachteile**:
    - <mark style="background: #FFB86CA6;">Langsamer</mark> und arbeitsintensiver im Vergleich zu direktem Shotgun Sequencing.
#### **2. Direct Shotgun Sequencing (Celera, Craig Venter)**
- **Vorgehen**:
    1. Das gesamte Genom wird direkt in <mark style="background: #FFB86CA6;">viele kleine Fragmente zerschnitten</mark>.
    2. Diese Fragmente werden sequenziert.
    3. Die Fragmente werden mithilfe von Bioinformatik-Tools <mark style="background: #FFB86CA6;">über ihre Überlappungen zu einer vollständigen Sequenz zusammengesetzt</mark>.
- **Vorteile**:
    - Schnellere Sequenzierung, da keine Contig-Kartierung notwendig ist.
    - Effizienter für kleinere Genome.
- **Nachteile**:
    - Problematisch bei repetitiven Sequenzen, da diese zu Fehlassemblierungen führen können.
    - Erfordert leistungsstarke Bioinformatik-Algorithmen.
#### **3. Hybrid Approach**
- **Vorgehen**:
    - <mark style="background: #FFB86CA6;">Kombination der beiden Ansätze</mark>: Hierarchical Shotgun Sequencing und direktes Shotgun Sequencing.
    - Die BAC-basierten Reads werden mit den Ganz-Genom-Reads kombiniert.
    - <mark style="background: #FFB86CA6;">Überlappende Sequenzen aus beiden Ansätzen werden genutzt, um eine höhere Genauigkeit zu erreichen</mark>.
- **Vorteile**:
    - Vereint die Struktur des hierarchischen Ansatzes mit der Geschwindigkeit des Shotgun Sequencing.
    - Liefert vollständigere und genauere Ergebnisse.
- **Forschung**:
    - Ziel ist es, die optimale Mischung der beiden Methoden für maximale Effizienz und Genauigkeit zu finden.
## Assemblierung der Sequenzen
<mark style="background: #FFB86CA6;">kurze DNA-Fragmente zu einer vollständigen Sequenz zusammengesetzt</mark> werden. Dieser Prozess funktioniert folgendermaßen:
1. **Überlappende Sequenzen erkennen**:
    - Bei der Shotgun-Sequenzierung entstehen zahlreiche zufällige Fragmente der DNA.
    - Die <mark style="background: #FFB86CA6;">Sequenzen dieser Fragmente werden analysiert, um überlappende Regionen zu identifizieren</mark>. Diese Überlappungen helfen, die Fragmente <mark style="background: #FFB86CA6;">in der richtigen Reihenfolge anzuordnen</mark>.
2. **Komplette Sequenz zusammenstellen**:
    - Die <mark style="background: #FFB86CA6;">zusammengefügten Fragmente ergeben schließlich die gesamte DNA-Sequenz</mark>. Spezielle Softwareprogramme wie **PHRAP** analysieren die Qualität und die Übereinstimmung der Sequenzen und fügen die Fragmente zusammen.
    **Schlüsselpunkt**: Die korrekte Assemblierung erfordert eine präzise Analyse beider DNA-Stränge.
## Finishing
Das **Finishing** ist der <mark style="background: #FFB86CA6;">letzte Schritt, um sicherzustellen, dass die komplette DNA-Sequenz korrekt und lückenlos</mark> ist:
1. **Lücken schließen**:
    - Wenn es Bereiche gibt, die während der Assemblierung nicht vollständig abgedeckt wurden, <mark style="background: #FFB86CA6;">werden spezifische Primer entworfen. Mit diesen Primern werden die fehlenden Regionen erneut sequenziert</mark>.
2. **Sicherstellung der Genauigkeit**:
    - <mark style="background: #FFB86CA6;">Ziel: Weniger als 1 Fehler pro 10.000 Basenpaare</mark>.
    - Um dies zu erreichen, wird oft eine <mark style="background: #FFB86CA6;">Coverage von 10x angestrebt</mark>. Das bedeutet, dass <mark style="background: #FFB86CA6;">jede Region mindestens zehnmal sequenziert</mark> wird, um Fehler zu minimieren.
3. **Analyse von Problemzonen**:
    - <mark style="background: #FFB86CA6;">Einige Regionen können "Gaps" oder "Single-stranded" Bereiche enthalten, die gezielt bearbeitet werden müssen</mark>, um eine vollständige Sequenz zu erhalten.
4. **Spezifische Primer für schwierige Regionen**:
    - <mark style="background: #FFB86CA6;">Weniger gut abgedeckte oder fehleranfällige Bereiche werden durch gezielte Sequenzierung mit spezifischen Primern verbessert.</mark>
## Scale-up von existierenden Techniken
- **Verbesserung der Technik:** Die Sequenziertechnologie hat sich stark verbessert, wodurch die Menge an Basen, die eine Person sequenzieren kann, erheblich gestiegen ist:
    - 2000: 500–1.000 Kilobasen (kb) pro Jahr.
- **Sanger-Methode:** Die meisten Hochleistungssequenziermethoden basieren immer noch auf der klassischen Sanger-Kettenabbruchtechnologie.
- **Pyrosequencing (2008):** Ein komplettes diploides Genom (6 Gigabasen) konnte in nur zwei Monaten sequenziert werden.
- **"1000 Dollar Genom" (2014):** Die Illumina-Sequenziertechnologie machte die vollständige Genomanalyse erschwinglich.
- **2015:** Eine vollständige Genomanalyse dauerte nur noch 26 Stunden, was entscheidend für Notfälle war.