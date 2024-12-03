Eine **DNA-Bibliothek** besteht aus einer Sammlung von DNA-Fragmenten, die in Vektoren (z. B. Plasmiden oder Phagen) eingebaut und in Bakterien oder anderen Wirten vermehrt wurden. Diese Bibliothek repräsentiert das gesamte Genom oder cDNA eines Organismus. Ziel ist es, spezifische Klone zu identifizieren, die ein gewünschtes Gen oder Protein tragen.
## Genomgröße versus Anzahl der Gene
![[Pasted image 20241203140520.png|400]]
- Mensch hat ca. 23.000 Gene und insgesammt 3.000.000.000 BP (3 GB)
- Weizen aber z.B. 17 GB
- Bakterien (Prokaryoten) haben kleinere Genome mit weniger Genen, während Eukaryoten größere Genome besitzen.
## Durchsuchen der Bibliothek
- Eine Sonde wird eingesetzt, um ein Ziel-DNA-Stück zu finden.
- Die DNA-Bibliothek enthält viele verschiedene Fragmente, und die Sonde fungiert wie ein Köder.
![[Pasted image 20241203140840.png#invert|200]]
- Antikörper können als Sonden verwendet werden, um spezifische Proteine zu erkennen.
- DNA- oder RNA-Sonden sind mit Markern wie $^{32}P$ oder DIG markiert und helfen, komplementäre DNA/RNA-Sequenzen zu identifizieren.
- Proteine in Bibliotheken müssen exprimiert werden
### Wann benutzt man Antikörper als Sonden
1. **Herstellung der Bibliothek:**
    - DNA-Fragmente werden in einen Vektor (z. B. Phagenvektor) eingefügt, der hinter einem Promotor liegt. Dies ermöglicht die Expression des eingefügten DNA-Stücks als Protein.
2. **Expression und Screening:**
    - Die Phagen werden auf eine Bakterienrasenplatte aufgetragen, wo sie Klone bilden.
    - Ein **Nitrozellulosefilter** wird auf die Platte gelegt, wodurch die exprimierten Proteine an den Filter binden.
3. **Nachweis mit Antikörpern:**
    - Der Filter wird mit einem spezifischen Antikörper inkubiert, der das Zielprotein erkennt.
    - Ein sekundärer Antikörper, markiert mit einem Detektionsmolekül (z. B. radioaktives Isotop oder Enzym), bindet an den primären Antikörper.
    - Durch Autoradiographie oder Farbreaktionen werden die positiven Klone sichtbar gemacht.
### Wann benutzt man DNA- oder RNA-Sonden
- **Klon-Screening:**
    - Jede Kolonie in der DNA-Bibliothek enthält ein Plasmid mit einem spezifischen DNA-Insert.
    - Eine **DNA-Sonde** ist ein markiertes Stück DNA oder RNA, das komplementär zur Zielsequenz ist.
- **Hybridisierung:**
    - Kolonien werden auf einen Filter übertragen, die DNA wird durch Zelllyse freigesetzt und denaturiert (in Einzelstränge zerlegt).
    - Die markierte Sonde wird hinzugefügt und hybridisiert nur mit der komplementären Zielsequenz.
- **Nachweis:**
    - Ungebundene Sonden werden ausgewaschen, und die hybridisierten Bereiche werden durch Autoradiographie oder andere Techniken detektiert.
#### Markierung von DNA-Sonden
##### **Radioaktive Markierung:**
- Hier werden radioaktive Nukleotide (z. B. mit $\alpha^{32}$) eingebaut.
###### a) **T4-Polynukleotidkinase**
- Diese Enzymreaktion katalysiert die Übertragung einer Phosphatgruppe vom $\gamma^{32}P$-ATP auf die 5'-Enden eines DNA-Strangs.
- Es gibt zwei Reaktionen:
    - **Forward Reaction:** Hinzufügen eines neuen Phosphatrests am 5'-Ende.
    - **Exchange Reaction:** Ersetzen eines bestehenden Phosphats durch ein markiertes Phosphat.
###### b) **Nick Translation**
- Doppeltsträngige DNA wird mit DNase I "geschnitten", wodurch sogenannte "Nicks" entstehen.
- DNA-Polymerase I verwendet radioaktive Nukleotide, um die Lücken in der DNA wieder aufzufüllen, was zur Markierung der DNA führt.
###### c) **Klenow-Markierung**
- Hierbei wird die DNA denaturiert, und Hexanukleotide werden als Primer verwendet.
- Mit Klenow-Fragmenten der DNA-Polymerase I und markierten Nukleotiden wird die DNA-Sonde synthetisiert.
###### d) **Labeling mit PCR**
- PCR kann genutzt werden, um DNA-Sonden zu amplifizieren und gleichzeitig mit markierten Nukleotiden (z. B. $\alpha^{32}P$-dATP) zu versehen.
##### **Nicht-radioaktive Markierung:** 
- Statt Radioaktivität werden chemische Labels wie DIG (Digoxigenin) oder Biotin verwendet.
- **DIG (Digoxigenin):** Ein chemisches Label, das an Desoxynukleotide gekoppelt ist. DIG wird mit Antikörpern detektiert, die mit einem Enzym (z. B. Alkalische Phosphatase) gekoppelt sind. Die enzymatische Reaktion erzeugt ein Farbsignal.
- **Biotin:** Biotinylierte Nukleotide können mit Streptavidin nachgewiesen werden, das ebenfalls enzymatisch detektiert wird.
### **Vergleich zwischen homologen und heterologen Sonden:**
- **Homologe Sonden:** Sie hybridisieren spezifischer und bei höheren Temperaturen.
- **Heterologe Sonden:** Können verwandte Gene erkennen, aber die Bindung ist weniger spezifisch.
### Membran-Hybridisierung und Southern Blot
Der **Southern Blot** ist eine spezifische Technik zur Identifikation von DNA-Sequenzen:
1. **DNA-Denaturierung:** Die doppelsträngige DNA wird in Einzelstränge aufgetrennt (z. B. durch Hitze oder chemische Denaturierung).
2. **Transfer auf Membran:** Die DNA wird durch Kapillarkräfte auf eine Nylon- oder Nitrozellulosemembran übertragen.
3. **Hybridisierung:** Die Membran wird mit einer markierten DNA-Sonde inkubiert, die spezifisch an die Ziel-DNA bindet.
4. **Signalnachweis:** Durch Autoradiographie oder chemische Reaktionen wird die gebundene Sonde sichtbar gemacht.
#### Anwendung
- Die DNA wird auf ein Gel aufgetragen, getrennt, und auf eine Membran transferiert.
- Eine markierte Sonde wird hinzugefügt, die hybridisiert und detektiert wird.
- Die Position des Signals zeigt die Größe und Anwesenheit der Zielsequenz.
### Oligonukleotide als Sonden
- Primer
- Oligonukleotide sind kurze DNA- oder RNA-Fragmente (ca. 20 Nukleotide), die gezielt an komplementäre DNA-Sequenzen binden können. Sie werden als Werkzeuge zur Identifikation spezifischer Klone in einer DNA-Bibliothek genutzt.
#### **Warum 20 Nukleotide?**
- Eine spezifische 20-Nukleotid-Sequenz ist in einem Genom sehr selten. Die Wahrscheinlichkeit, dass eine 20er-Sequenz zufällig irgendwo vorkommt, beträgt $4^{20}$ (ca. $10^{12}$).
- Dies ist besonders geeignet für Genome mit Größen von Milliarden Basenpaaren (z. B. Mensch: 3 Milliarden bp).
#### Herkunft der Oligonukleotid-Sequenzen
Die 20 Nukleotide, die für die Sonden benötigt werden, stammen aus verschiedenen Quellen:
1. **Bekannte DNA-Fragmente:**
    - DNA-Sequenzen, die bereits sequenziert wurden, wie **expressed sequence tags (ESTs)**.
    - ESTs sind kurze cDNA-Fragmente, die aus mRNA durch Reverse Transkription hergestellt wurden. Sie repräsentieren exprimierte Gene und können leicht sequenziert werden.
2. **Heterologe Sonden:**
    - Diese stammen von verwandten Genen aus anderen Spezies. Da homologe Gene oft konservierte Sequenzen haben, können diese Sonden an ähnliche Gene binden.
3. **Von Proteinsequenzen:**
    - Falls die Aminosäuresequenz des Zielproteins bekannt ist, kann diese in die entsprechende DNA-Sequenz umgerechnet werden. Dies erfordert die Kenntnis des genetischen Codes.
    - Für 7 spezifische Aminosäuren benötigt man eine DNA-Sonde von 21 Basenpaaren.
#### Herstellung von Oligonukleotid-Sonden aus Proteinsequenzen
Die Sequenz des Zielproteins kann durch biochemische Methoden bestimmt werden:
##### Edman-Abbau
**Prinzip:**
- Beim **Edman-Abbau** wird die **aminoterminale Aminosäure** (die erste Aminosäure eines Proteins) schrittweise abgetrennt, ohne die Peptidbindungen zwischen den anderen Aminosäuren zu zerstören.
- Die Methode ermöglicht es, die Sequenz von Peptiden oder Proteinen schrittweise zu bestimmen.
 **Ablauf:**
1. **Labeling (Markierung):**  
    Die Aminosäure wird mit **Phenylisothiocyanat (PITC)** chemisch modifiziert. Dies erfolgt unter mild alkalischen Bedingungen (pH 8.0).
2. **Release (Abspaltung):**  
    Die modifizierte Aminosäure wird unter mild sauren Bedingungen (z. B. durch Erhitzen) abgespalten.
3. **Identifizierung:**  
    Das abgespaltene Aminosäure-Derivat wird durch **Chromatographie** oder **Elektrophorese** analysiert und identifiziert.
4. **Wiederholung:**  
    Nach der Entfernung der ersten Aminosäure wird der Zyklus mit dem verkürzten Peptid erneut durchgeführt. Dieser Prozess wird so lange wiederholt, bis die Sequenz bestimmt ist.
**Einschränkungen:**
- Maximal können etwa 50–60 Aminosäuren sequenziert werden. Meistens liegt die Grenze jedoch bei 30 Aminosäuren, da der Prozess mit zunehmender Länge an Genauigkeit verliert.
##### Massenspektrometrie (MALDI-TOF)
 **Prinzip:**
- Diese Methode nutzt einen Laser, um Proteine oder Peptide zu ionisieren. Anschließend wird die Flugzeit (Time of Flight, TOF) der Ionen gemessen, um ihr **Masse-zu-Ladungsverhältnis (m/z)** zu bestimmen.
 **Ablauf:**
1. **Probenvorbereitung:**
    - Die Proteine werden mit einer Protease (z. B. Trypsin) verdaut, um kleinere Peptide zu erzeugen.
    - Diese Peptide werden mit einer Matrix vermischt, die die Ionisierung unterstützt.
2. **Ionisierung:**  
    Ein **UV-Laser** ionisiert die Peptide, indem er die Matrix verdampft. Die dabei entstehenden Ionen werden in das Massenspektrometer eingespeist.
3. **Beschleunigung:**  
    Die ionisierten Peptide werden durch ein elektrisches Feld beschleunigt.
4. **Flugzeitmessung:**  
    Die Geschwindigkeit der Ionen hängt von ihrem **m/z-Verhältnis** ab: Leichtere Ionen fliegen schneller als schwerere. Dies ermöglicht die Bestimmung der Masse der Peptide.
5. **Spektralanalyse:**  
    Das Massenspektrum wird analysiert, um die Masse der Peptide zu bestimmen und daraus die Aminosäuresequenz abzuleiten.
 **Vorteile:**
- Geeignet für große Proteine und komplexe Peptidmischungen.
- Sehr schnelle und empfindliche Methode.
 **Einschränkungen:**
- Die Reihenfolge der Aminosäuren in einem Peptid kann nicht direkt abgelesen werden. Zusätzliche bioinformatische Analysen sind erforderlich.
#### **Ableitung von Oligonukleotid-Sonden aus Proteinsequenzen**
Nachdem die Aminosäuresequenz bekannt ist, kann man mit dem **genetischen Code** die entsprechende DNA-Sequenz berechnen. Dabei ist zu beachten, dass:
- Der genetische Code **degeneriert** ist, d. h., eine Aminosäure kann durch mehrere Codons codiert werden.
- Beispielsweise hat **Leucin** sechs verschiedene Codons.
##### **Ansätze zur Verringerung der Degeneration:**
- Einsatz konservierter Codons aus verwandten Spezies.
- Nutzung zusätzlicher Informationen (z. B. mRNA-Sequenzen), um die am wahrscheinlichsten verwendeten Codons zu bestimmen.