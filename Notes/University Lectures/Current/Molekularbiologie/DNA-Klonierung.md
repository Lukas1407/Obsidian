## Erstellung einer DNA-Bibliothek:
   - **Amplifizierte DNA-Fragmente**: Die zu klonierende DNA wird oft zuvor durch PCR vervielfältigt, um sicherzustellen, dass genügend DNA-Material vorhanden ist. Diese Fragmente können dann in eine DNA-Bibliothek integriert werden.
   - **Ligieren der DNA in einen Vektor**: Die amplifizierten DNA-Fragmente werden durch einen Prozess namens Ligation in einen Vektor integriert. Ein Vektor ist ein DNA-Molekül, das als Transportmittel dient (z. B. ein Plasmid), um das DNA-Fragment in eine Wirtszelle einzuführen.
   - **Klonierung durch Replikation**: Die Vektoren, die das DNA-Fragment enthalten, werden in eine Wirtszelle (häufig eine Bakterienzelle) eingebracht. Diese Zellen vermehren sich dann, und das eingebettete DNA-Fragment wird bei jeder Zellteilung kopiert, wodurch eine große Anzahl identischer DNA-Kopien entsteht.
   - **Library Screening**: Die einzelnen DNA-Fragmente werden katalogisiert, um später gezielt nach bestimmten Genen oder Sequenzen, wie z. B. dem Phosphattransporter, suchen zu können.
## DNA-Ligation und Klonierung in Vektoren:
![[Pasted image 20241112100109.png#invert|300]]
   - **Restriktionsverdau**: Die DNA-Fragmente und der Vektor werden mit einem Restriktionsenzym geschnitten, das spezifische Schnittstellen erkennt und schneidet (z. B. BamHI). So entstehen "klebrige Enden", die komplementär zueinander sind und leichter verbunden werden können.
   - **Ligation**: Die geschnittenen Enden des DNA-Fragments und des Vektors werden durch ein Enzym namens [[Ligase]] miteinander verbunden. Dadurch entsteht ein rekombinantes Plasmid, das sowohl das Vektor-DNA als auch das Ziel-DNA-Fragment enthält.
   - **Transformation und Zellvermehrung**: Das rekombinante Plasmid wird in Bakterienzellen eingeführt, die sich anschließend teilen und das Plasmid bei jeder Teilung vervielfältigen. Dies führt zur Bildung vieler Klone, die alle das eingefügte DNA-Fragment enthalten.
## DNA-Klonierungsmethoden und Genbanken:
   - **DNA-Ligation** und **DNA-Klonierung mit Plasmidvektoren** sind grundlegende Schritte, bei denen die Ziel-DNA in einen Vektor integriert und in eine Wirtszelle eingebracht wird.
   - **Transformationsmethoden**: Verschiedene Methoden (z. B. Hitze- oder Elektroschock) werden verwendet, um die DNA in die Zelle zu transportieren.
   - **Selektion der Klone**: Nur die Zellen, die erfolgreich das Plasmid aufgenommen haben, werden ausgewählt. Dies kann durch Marker im Plasmid (z. B. Antibiotikaresistenzgene) erreicht werden.
   - **Genbanken**: Die klonierten Fragmente werden in Vektoren wie Plasmiden, λ-Phagen, Cosmiden oder künstlichen Chromosomen (BACs, YACs) gespeichert, um später darauf zugreifen zu können.
![[Pasted image 20241112100619.png#invert|400]]
### Vektor
- Vektoren sind Werkzeuge in der Molekularbiologie, die als Träger fungieren, um DNA-Fragmente in eine Wirtszelle zu bringen. Sie können die eingefügte DNA nicht nur transportieren, sondern auch replizieren, sodass sie in nachfolgenden Zellgenerationen erhalten bleibt.
- Z.B. Plasmide
#### Plasmide
Plasmide sind eine der am häufigsten verwendeten Vektoren in der Molekularbiologie, weil sie einfach zu manipulieren und zu replizieren sind.
![[Pasted image 20241119121135.png|200]]
- **Struktur**: Plasmide bestehen aus einem zirkulären DNA-Molekül, das von chromosomaler DNA getrennt ist. Sie enthalten spezifische Regionen, die essenziell für ihre Funktion sind:
    - **Origin of Replication (ORI)**: Startpunkt für die Replikation. Dadurch kann das Plasmid eigenständig in der Zelle kopiert werden.
    - **Region für DNA-Insertion**: Bereich, in den fremde DNA (z. B. Gene) eingefügt werden kann. Diese Region enthält oft eine sogenannte Multiple Cloning Site (MCS), die verschiedene Schnittstellen für Restriktionsenzyme bereitstellt.
    - **Selektionsmarker**: Gene, die es ermöglichen, Zellen mit Plasmid von denen ohne Plasmid zu unterscheiden, wie etwa Antibiotikaresistenzgene.
**Eigenschaften**:
- **Replikation**: Plasmide werden vor jeder Zellteilung verdoppelt. Dadurch wird sichergestellt, dass Tochterzellen mindestens eine Kopie erhalten.
- **Größe**: Plasmide können DNA-Fragmentgrößen von etwa 1 kb bis 200 kb aufnehmen.
- **Kopienzahl**: Plasmide existieren in Zellen in unterschiedlichen Kopienzahlen. High-copy-Plasmide (z. B. pUC19) können in einer Zelle Hunderte Kopien haben, während low-copy-Plasmide (z. B. pBR322) nur wenige Kopien besitzen.
#### Anforderungen an Vektoren
Ein effektiver Vektor muss bestimmte Eigenschaften aufweisen, um für die DNA-Klonierung geeignet zu sein:
1. **Origin of Replication (ORI)**: Notwendig, damit das Plasmid eigenständig repliziert werden kann.
![[Pasted image 20241119121246.png#invert|300]]
1. **Region für DNA-Insertion**: Eine MCS (Multiple Cloning Site) mit spezifischen Schnittstellen für Restriktionsenzyme ermöglicht das gezielte Einfügen fremder DNA.
![[Pasted image 20241119121303.png#invert|300]]
1. **Selektionsmarker**: Ermöglicht die Identifikation von Zellen, die das Plasmid enthalten. Beispiel: Antibiotikaresistenzgene wie _amp_ (Ampicillin-Resistenz).
#### Verwendung von Vektoren in der DNA-Klonierung
1. **Einfügen der Fremd-DNA**:
    - Fremde DNA wird mithilfe von Restriktionsenzymen in den Vektor (z. B. ein Plasmid) geschnitten.
    - Die DNA wird durch Ligation (Verknüpfung mit Ligase) stabil in das Plasmid eingefügt.
2. **Transformation**:
    - Das rekombinante Plasmid wird in eine Wirtszelle (z. B. _Escherichia coli_) eingebracht.
3. **Selektion**:
    - Zellen, die das Plasmid aufgenommen haben, werden auf einem Medium mit dem entsprechenden Antibiotikum kultiviert.
4. **Vermehrung und Expression**:
    - Die Wirtszellen vermehren sich, replizieren das Plasmid und exprimieren bei Bedarf die eingefügte DNA.

## Transformation in der DNA-Klonierung
- Transformation ist der Prozess, bei dem freie DNA (wie Plasmid-DNA) in Zellen (z. B. Bakterien, Pflanzen oder tierischen Zellen) eingeführt wird. Es gibt verschiedene Methoden, um dies zu erreichen:
### Hitzeschock
1. Zellen werden in einer Calciumchlorid (CaCl₂)-Lösung inkubiert, wodurch sie kompetent werden, also DNA aufnehmen können.
2. Die Zugabe von Ca²⁺-Ionen neutralisiert die negative Ladung der Zellmembran und der DNA, wodurch die DNA näher an die Membran gebracht wird.
3. Ein Temperaturwechsel (Hitzeschock) wird durchgeführt, indem die Zellen kurz auf 42°C erhitzt werden. Dies führt zu:
        1. **Schwellung** der Zellen.
        2. Bildung kleiner Poren in der Membran.
        3. Unterdruck, der die DNA ins Zellinnere zieht.
4. Nach dem Schock werden die Zellen auf Eis gelegt, um die Membran zu stabilisieren.
- **Effizienz**: Nur etwa 1 von 10.000 Zellen nimmt die DNA auf, daher ist diese Methode relativ ineffizient.
- **Schema**: Nach der Transformation werden die Zellen auf einem Antibiotikum-haltigen Medium ausgestrichen, um transformierte Kolonien zu selektieren.
### Elektroporation
1. Kompetente Zellen werden in einer Suspension mit der DNA in eine Elektroporationsküvette gegeben.
2. Ein elektrisches Feld (z. B. 2000 V für etwa 1 Millisekunde) wird angelegt.
3. Der elektrische Impuls erzeugt eine kurzfristige **Permeabilisierung** der Zellmembran:
        - Es bilden sich temporäre Poren in der Membran.
        - Die Fremd-DNA gelangt durch die Poren ins Zellinnere.
4. Nach dem Impuls schließt sich die Membran wieder und bleibt intakt.
- **Effizienz**: Höher als beim Hitzeschock, da die Zellen gezielt geöffnet werden.
- **Vorteil**: Funktioniert auch für dickere Zellwände, z. B. bei Pflanzenzellen.
## Selektion von transformierten Zellen
Die Selektion ist ein essenzieller Schritt nach der Transformation, um Zellen zu identifizieren, die das Plasmid aufgenommen haben.
1. **Prozess**:
    - Ein rekombinantes Plasmid wird in kompetente _E. coli_-Zellen eingebracht (z. B. durch Hitzeschock oder Elektroporation).
    - Die Zellen werden auf einem Nährboden mit einem Selektionsmittel kultiviert, z. B. einem Antibiotikum wie Ampicillin.
2. **Mechanismus**:
    - **Transformierte Zellen** (Zellen, die das Plasmid aufgenommen haben) überleben, da sie ein Antibiotikaresistenzgen (z. B. AmpR für Ampicillin) tragen.
    - **Nicht-transformierte Zellen** sterben auf dem Antibiotika-haltigen Medium ab, da ihnen die Resistenz fehlt.
3. **Ergebnis**:
    - Kolonien auf dem Agar repräsentieren transformierte Zellen, die das Plasmid erfolgreich aufgenommen haben.
## Selektion rekombinanter Plasmide
Um sicherzustellen, dass die eingefügte Fremd-DNA tatsächlich in das Plasmid integriert wurde (rekombinantes Plasmid), wird eine zweite Selektion durchgeführt.
### **Blue-White-Screening (Blau-Weiß-Selektion)**:
Dieses Verfahren basiert auf der Funktion des lacZ-Gens, das für das Enzym **β-Galactosidase** codiert. Dieses Enzym spaltet das Substrat X-Gal (eine synthetische Verbindung), wodurch ein blauer Farbstoff entsteht.
1. **Funktionsweise**:
    - Das Plasmid enthält eine **Multiple Cloning Site (MCS)** innerhalb des lacZ-Gens.
    - Wenn Fremd-DNA in die MCS eingefügt wird, wird das lacZ-Gen unterbrochen, und die Zelle produziert keine β-Galactosidase.
    - Ohne Unterbrechung des lacZ-Gens bleibt die β-Galactosidase funktional.
2. **Screening**:
    - Die transformierten Zellen werden auf einem Agar mit Ampicillin, IPTG und X-Gal kultiviert.
        - **IPTG** ist ein Induktor, der die Expression des lacZ-Gens aktiviert.
        - **X-Gal** wird von β-Galactosidase gespalten und erzeugt eine blaue Farbe.
    - **Blaue Kolonien**: Nicht-rekombinante Plasmide (lacZ aktiv, keine Fremd-DNA integriert).
    - **Weiße Kolonien**: Rekombinante Plasmide (lacZ inaktiv, Fremd-DNA integriert).
3. **Vorteil**:
    - Dies erlaubt eine schnelle und visuelle Unterscheidung zwischen Zellen mit rekombinanten und nicht-rekombinanten Plasmiden.
### **Ergebnis der Selektion**
- Jede Kolonie repräsentiert einen **Klon**, der von einer einzigen Zelle abstammt. Dadurch kann die eingefügte Fremd-DNA spezifisch analysiert werden.
- Rekombinante Kolonien (weiß) enthalten das gewünschte Plasmid mit der Fremd-DNA, während nicht-rekombinante Kolonien (blau) ausgeschlossen werden können.
### Positive und Negative Selektion von Klonen
Die Selektion wird verwendet, um sicherzustellen, dass die Zellen ein korrekt rekombiniertes Plasmid tragen.
#### **Positivselektion:**
- Ein **Antibiotikaresistenzgen** (z. B. AmpR) wird genutzt, um Zellen zu selektieren, die das Plasmid aufgenommen haben.
- Zellen ohne Plasmid sterben auf Antibiotika-haltigem Medium ab.
#### **Negativselektion:**
- Ein zweites Selektionsgen (z. B. KmR = Resistenz gegen Kanamycin) wird verwendet.
- Wenn die Fremd-DNA in das Plasmid inseriert wird, wird das KmR-Gen unterbrochen.
- Zellen, die noch das funktionelle KmR-Gen haben, tragen kein korrekt rekombiniertes Plasmid.
#### **Ablauf:**
1. Das Plasmid enthält zwei Antibiotikaresistenzgene (z. B. AmpR und KmR).
2. Die Fremd-DNA wird in die **Multiple Cloning Site (MCS)** eingefügt, was das KmR-Gen deaktiviert.
3. Zellen werden auf zwei Medien getestet:
   - **LB+Amp**: Nur Zellen mit einem Plasmid wachsen.
   - **LB+Amp+Km**: Nur Zellen mit einem nicht-rekombinierten Plasmid wachsen (da KmR intakt ist).
#### **Ergebnis:**
- Rekombinante Plasmide wachsen nur auf LB+Amp, nicht auf LB+Amp+Km.
- Nicht-rekombinante Plasmide wachsen auf beiden Medien.
## Restriktionsanalyse von Plasmiden
Die Restriktionsanalyse wird verwendet, um zu bestätigen, dass Fremd-DNA in ein Plasmid integriert wurde und die korrekte Größe hat.
### **Ablauf:**
1. **Restriktionsenzyme** schneiden DNA an spezifischen Stellen.
   - Die genaue Position der Schnittstellen hängt von der Sequenz ab.
   - Das Plasmid wird linearisiert, oder es entstehen Fragmente, deren Länge überprüfbar ist.
2. Die DNA wird mittels **Agarose-Gelelektrophorese** aufgetrennt:
   - Das Gel zeigt die Größen der DNA-Fragmente.
   - Die Position der Banden wird mit einem **Molekulargewichtsmarker** verglichen.
### **Ergebnis:**
![[annotated_Pasted image 20241119123021|400]]
[[Notes/University Lectures/Current/Molekularbiologie/images/Pasted image 20241119123021.png#invert|400]]
  - Plasmid A enthält ein Insert → zeigt zusätzlich eine Band, die der Größe des Inserts entspricht.
  - Plasmide B und C enthalten kein Insert → zeigen nur die Banden des Vektors.
## Genbanken in Plasmiden
Eine Genbank ist eine Sammlung von DNA-Fragmenten, die das gesamte Genom eines Organismus repräsentieren.
### **Herausforderungen bei Plasmiden:**
1. **Effizienz der Transformation:**
   - *E. coli* kann nur begrenzte Mengen an Plasmid-DNA aufnehmen.
   - Die Transformationseffizienz ist niedrig, was bedeutet, dass nur wenige Kolonien pro Platte erhalten werden.
2. **Größe des Genoms:**
   - Das menschliche Genom ist etwa 3 Milliarden Basenpaare groß.
   - Plasmide können nur kleine DNA-Stücke (z. B. 20 kb) aufnehmen.
   - Um das gesamte menschliche Genom zu klonieren, wären ca. 150.000 Klone nötig.
#### **Berechnung der Anzahl benötigter Klone:**
- **Formel:**  
  $$N = \frac{\ln(1-P)}{\ln(1-a/b)}$$
  
  - \( P \): Wahrscheinlichkeit, dass ein Gen in der Genbank enthalten ist (meist 95 %).
  - \( a \): Durchschnittliche Größe der DNA-Fragmente in der Bibliothek.
  - \( b \): Gesamtgröße des Genoms.
#### **Beispiel aus der Tabelle:**
- Für das menschliche Genom (\( b = 3.000.000 \) kb) sind bei 20 kb Fragmentgröße etwa **150.000 Klone** notwendig.
## DNA-Genbanken in λ-Phagen
λ-Phagen sind nützlich, um große DNA-Fragmentbibliotheken zu erstellen, da sie bis zu 20 kb Fremd-DNA aufnehmen können.
### Ersetzung der "Replaceable Region":
- Der λ-Phage hat eine entbehrliche Region in seinem Genom, die entfernt werden kann.
- Diese Region wird durch Fremd-DNA ersetzt, die in die Phagenpartikel verpackt wird.
- Das resultierende rekombinante Phagen-DNA-Molekül wird bei Infektion in Bakterienzellen exprimiert.

λ-Phagen können in Bakterienzellen (z. B. _E. coli_) entweder den lytischen oder den lysogenen Zyklus durchlaufen:
### **Lytischer Zyklus**:
- **Prozess**:
    1. Der Phage bindet an die Wirtszelle und injiziert seine DNA.
    2. Die Phagen-DNA übernimmt die Kontrolle über die Zellmaschinerie.
    3. Neue Phagen-DNA und -Proteine werden synthetisiert und zu funktionsfähigen Virionen zusammengesetzt.
    4. Die Wirtszelle lysiert, setzt mehr als 100 neue Phagen frei und stirbt.
- **Ergebnis**: Schnelle Vermehrung und Freisetzung von Phagen.
### Lysogener Zyklus:
- **Prozess**:
    1. Nach der Injektion integriert sich die Phagen-DNA in das bakterielle Chromosom und wird zu einem **Prophagen**.
    2. Die Phagen-DNA wird bei jeder Zellteilung passiv repliziert.
    3. Unter bestimmten Bedingungen (z. B. Stress) kann der Prophage den lysogenen Zyklus verlassen und in den lytischen Zyklus übergehen.
- **Ergebnis**: Langfristige Erhaltung der Phagen-DNA ohne Zelltod.
### Erstellen von DNA-Genbanken mit λ-Phage
DNA-Genbanken mit λ-Phagen eignen sich für die Klonierung großer DNA-Fragmente, z. B. für genomische Bibliotheken.
#### **Ablauf**:
1. **Fragmentierung der DNA**:
    - Human-DNA (z. B. 3x10⁹ Basenpaare) wird mit Restriktionsenzymen (z. B. Sau3A) in Fragmente von ca. 20 kb zerlegt.
    - Diese Fragmente besitzen "klebrige Enden" (Sticky Ends), die mit λ-DNA kompatibel sind.
2. **Rekombination**:
    - Die 20-kb-Fragmente werden mit den Armen der λ-DNA (enthalten COS-Sites) kombiniert und mit DNA-Ligase verbunden.
3. **Verpackung in Phagenpartikel**:
    - Die rekombinante DNA wird in vitro in Phagenköpfe verpackt und bildet funktionsfähige Phagen.
4. **Infektion und Vermehrung**:
    - Die Phagen infizieren Bakterienzellen und bilden eine hohe Anzahl identischer Phagen (ca. 10⁶ pro Mikrogramm DNA).
    - Jede Phagenplaque entspricht einem spezifischen DNA-Fragment.
#### **Vorteile gegenüber Plasmiden**:
- **Effizienz**:
    - λ-Phagen können größere DNA-Fragmente (ca. 20 kb) aufnehmen als Plasmide (max. ca. 10 kb).
    - Sie erzeugen eine höhere Anzahl an Klonen (Plaques) pro Mikrogramm DNA.
- **Platzbedarf**:
    - Für die Klonierung des menschlichen Genoms benötigt man nur 20-30 Petri-Schalen mit λ-Phagen, aber tausende Platten bei der Verwendung von Plasmiden.
### Überlappende Klone und Genomabdeckung
Die Erstellung einer DNA-Genbank erfordert eine ausreichende Anzahl von Klonen, um das gesamte Genom abzudecken:
- **Überlappende Klone**:
    - DNA-Fragmente überlappen, um sicherzustellen, dass alle Regionen des Genoms repräsentiert sind.
    - Überlappende Klone ermöglichen eine lückenlose Kartierung des Genoms durch "chromosome walking".
- **Berechnung der Anzahl Klone**:
    - Beispiel: Um 95 % des menschlichen Genoms zu repräsentieren, sind ca. 150.000 verschiedene rekombinante λ-Phagen nötig.
## DNA-Genbanken in Cosmiden
**Cosmide** sind Hybridvektoren, die Eigenschaften von Plasmiden und Phagen kombinieren. Sie können größere DNA-Inserts als Plasmide aufnehmen, bis zu **45 kb**.
#### **Aufbau und Funktion von Cosmiden:**
1. **COS-Sites**:
    - Sie stammen von λ-Phagen und ermöglichen die Verpackung der DNA in Phagenhüllen.
    - Erlauben die effiziente Infektion von Wirtszellen (_E. coli_).
2. **Plasmid-Eigenschaften**:
    - Cosmide enthalten ein **ORI (Origin of Replication)**, das die Replikation in Bakterienzellen ermöglicht.
    - Sie tragen ein Antibiotikaresistenzgen (z. B. Ampicillin), um transformierte Zellen zu selektieren.
3. **Polylinker**:
    - Enthält Schnittstellen für Restriktionsenzyme, an denen die Fremd-DNA eingefügt wird.
#### **Prozess der Klonierung in Cosmiden:**
1. **Schritt 1: Schneiden und Ligieren**:
    - Die Cosmide und die DNA-Fragmente (35-45 kb) werden mit Restriktionsenzymen geschnitten.
    - Die Fragmente werden mit DNA-Ligase in das Cosmid-Vektor-DNA integriert.
2. **Schritt 2: Verpackung in Phagenhüllen**:
    - Die rekombinante Cosmid-DNA wird in Phagenhüllen verpackt.
3. **Schritt 3: Infektion und Selektion**:
    - Die Phagen infizieren _E. coli_-Zellen.
    - Transformierte Zellen werden auf Ampicillin-haltigem Medium selektiert.
#### **Vorteile von Cosmiden:**
- Größere Insertkapazität (bis zu 45 kb) im Vergleich zu Plasmiden.
- Einfacher zu handhaben als Phagenvektoren.
- Effiziente Klonierung von großen Genen oder Genomfragmenten.
## **DNA-Genbanken in Bakteriellen Künstlichen Chromosomen (BACs)**
**BACs (Bacterial Artificial Chromosomes)** sind Plasmid-basierte Vektoren, die für die Klonierung sehr großer DNA-Fragmente (bis zu **300 kb**) verwendet werden.
### **Eigenschaften von BACs:**
1. **Abgeleitet vom F-Faktor-Plasmid**:
    - Der **F-Faktor** ist ein großes natürliches Plasmid in Bakterien, das für die Konjugation (horizontaler Gentransfer) verantwortlich ist.
    - BACs enthalten den Replikationsursprung (ORI) und die F-Faktor-Proteine, die die stabile Replikation und Verteilung des Plasmids gewährleisten.
2. **Kapazität**:
    - BACs können DNA-Inserts von 100-300 kb aufnehmen, ideal für die Klonierung großer Genome.
3. **Niedrige Kopienzahl**:
    - BACs liegen in Zellen in niedriger Kopienzahl vor, was die Stabilität großer Inserts erhöht.
### **Klonierungsprozess in BACs:**
1. **Fragmentierung der DNA**:
    - Große DNA-Stücke werden mit Restriktionsenzymen geschnitten, z. B. in 300 kb Fragmente.
2. **Einfügen in den BAC-Vektor**:
    - Die DNA-Fragmente werden an spezifischen Schnittstellen im BAC-Vektor ligiert.
3. **Transformation in _E. coli_**:
    - Die rekombinanten BACs werden in _E. coli_-Zellen eingebracht.
    - Transformierte Zellen werden durch Selektion (z. B. Antibiotikaresistenz) identifiziert.
### **Vorteile von BACs:**
- **Sehr hohe Kapazität**: Ideal für die Klonierung großer Genome wie dem menschlichen Genom.
- **Hohe Stabilität**: Große DNA-Fragmente bleiben während der Zellteilung stabil.
- **Effiziente Verarbeitung**: Weniger Klone sind erforderlich als bei Plasmiden oder Cosmiden.
### **Konjugation und der F-Faktor**:
- Der F-Faktor ermöglicht den horizontalen Gentransfer zwischen Bakterien durch Konjugation:
    1. Eine F⁺-Zelle (trägt den F-Faktor) bildet eine Konjugationstube zu einer F⁻-Zelle.
    2. Der F-Faktor wird repliziert und in die F⁻-Zelle übertragen.
    3. Beide Zellen werden F⁺ und können den F-Faktor weitergeben.
- BACs nutzen diese Mechanismen zur Stabilisierung und Replikation des Vektors in Wirtszellen.
## DNA-Genbanken in Yeast Artificial Chromosomes (YACs)
### **Eigenschaften von YACs**
**Yeast Artificial Chromosomes (YACs)** sind künstliche Chromosomen, die in Hefe (_Saccharomyces cerevisiae_) verwendet werden, um sehr große DNA-Fragmente zu klonieren, bis zu **3000 kb**.
- **Komponenten eines YAC-Vektors**:
    - **Telomere (TEL)**: Schützen die Enden des Chromosoms und verhindern Degradierung.
    - **Centromere (CEN)**: Sichern die korrekte Verteilung während der Zellteilung.
    - **Origin of Replication (ORI)**: Ermöglicht die Replikation in Hefe.
    - **Selektionsmarker**:
        - Für Hefen: Marker wie _trp1_ oder _ura3_ (ermöglichen das Wachstum nur transformierter Hefezellen).
        - Für Bakterien: Antibiotikaresistenzgene (z. B. _AmpR_ für Ampicillin).
#### **Besonderheiten**:
- YACs sind große, lineare DNA-Moleküle.
- Sie sind weniger stabil als BACs, da sie anfälliger für Rekombination sind.
- Wurden erstmals 1983 von **Murray und Szostak** beschrieben.
###  Klonierung mit YACs
**Prozess der DNA-Klonierung in YACs**:
1. **Vorbereitung des YAC-Vektors**:
    - Der Vektor wird mit Restriktionsenzymen (z. B. _BamHI_ und _EcoRI_) geschnitten, um lineare Fragmente mit offenen Enden zu erzeugen.
    - Die Telomere schützen die Enden des Vektors.
2. **Einfügen von Fremd-DNA**:
    - Große DNA-Fragmente (oft in der Größenordnung von Megabasen) werden ebenfalls mit Restriktionsenzymen geschnitten.
    - Die Fremd-DNA wird mit DNA-Ligase in den YAC-Vektor integriert.
3. **Transformation in Hefezellen**:
    - Der rekombinante YAC wird in Hefezellen eingebracht.
    - Transformierte Zellen werden anhand von Selektionsmarkern identifiziert (z. B. durch fehlende Aminosäuren in Wachstumsmedien).
4. **Stabilisierung und Klonierung**:
    - Der YAC wird in der Hefe repliziert und verteilt sich während der Zellteilung wie ein normales Chromosom.
### **3. Vorteile von YACs**
- **Klonierung sehr großer Fragmente**: Mit bis zu **3000 kb** sind YACs der leistungsfähigste Vektortyp für die Klonierung großer Genome (z. B. für menschliche Chromosomen).
- **Chromosomenähnliche Eigenschaften**:
    - Telomere und Centromere machen sie besonders geeignet für eukaryotische DNA.
- **Selektionsmöglichkeiten**:
    - Marker erlauben die einfache Identifizierung transformierter Zellen.
## **Nachteile von YACs**
- **Instabilität**:
    - Häufige Rekombination kann zu Verlust oder Umlagerung von DNA-Fragmenten führen.
- **Komplexität**:
    - Herstellung und Handhabung sind schwieriger als bei BACs.