****- Stellt eine vollständige Kopie der [[Desoxyribonukleinsäure|DNA]] her
- Bei der Zellteilung aktiv
- Extrem hohe Genauigkeit durch Reparaturmechanismen (1 Fehler pro 1 Mrd. [[Nukleotide]])
- Semi-konservative Verdopplung:
	- jeder der beiden neuen DNA-Doppelstränge aus einem **alten** (ursprünglichen) und einem **neuen** Strang besteht
## Ablauf
### 1. Trennung der Stränge:
- Zu Beginn der Replikation werden die beiden Stränge der DNA-Doppelhelix voneinander getrennt. Enzyme wie **[[Helikase]]** brechen die Wasserstoffbrücken zwischen den komplementären Basenpaaren auf, sodass zwei Einzelstränge entstehen.
### 2. Anlagerung neuer Nukleotide:
- An jeden der Einzelstränge wird ein neuer, komplementärer Strang synthetisiert. Die **DNA-Polymerase** fügt dazu die passenden Nukleotide (Adenin zu Thymin und Guanin zu Cytosin) an den offenen Einzelstrang an.
### 3. Verknüpfen der neuen Nukleotide
- Indem die neuen Nukleotide an das Zucker-Phosphat-Rückgrad verknüpft werden entstehen 2 identische DNA-Doppelhelices 
![[Pasted image 20241031095537.jpg#invert|500]]
## Zuständige Proteine
- [[Helikase]]: Entwindung der DNA-Doppelhelix
- [[SSB-Proteine]]: Stabilisieren die geöffnete DNA-Doppelhelix
- [[Desoxyribonukleinsäure#Topoisomerase Typ I|Topoisomerase]]: Abbau der Superhelikalität der DNA
- [[Primase]]: Synthese eines kurzen [[RNA-Primer]] für die [[DNA-Synthese]]
- [[DNA-Polymerase]]: [[DNA-Synthese]] und Korrekturlesen
- [[Ligase]]: Verknüpft die DNA-Einzelstränge
## Replikationsgabel
![[Pasted image 20241031102217.png#invert|600]]
- [[DNA-Polymerase]] läuft immer in 5'-3' Richtung ab
- DNA wird stückweise synthetisiert -> es entstehen [[Okazaki-Fragmente]] welche durch [[Ligase]] verknüpft werden
### Unterschiedliche Replikation von Leit- und Folgestrang
![[Pasted image 20241031102500.png#invert|500]]
## Start der DNA-Replikation
### Bei Prokaryoten
- [[DNA-Polymerase]] kann nicht einfach an einem Einzelstrang binden, es benötigt einen [[RNA-Primer]]
![[Pasted image 20241031102830.png#invert|500]]
#### Entfernen der Primer
- Sobald diese Fragmente synthetisiert wurden, müssen die RNA-Primer durch DNA ersetzt und die Fragmente verbunden werden.
- **Erkennung und Entfernung der RNA-Primer durch RNase H**:
    - **RNase H** ist ein Enzym, das spezifisch RNA in einem **RNA-DNA-Hybrid** abbauen kann. Es erkennt die RNA-Primer und schneidet sie in kleine Fragmente.
    - Dies bedeutet, dass RNase H die RNA nur dann abbauen kann, wenn sie an DNA gebunden ist (z. B. in den RNA-Primer-Bereichen der Okazaki-Fragmente).
- **Exonuklease-Aktivität der DNA-Polymerase I**:
    - Nach dem teilweisen Abbau der RNA-Primer durch RNase H übernimmt die **DNA-Polymerase I**. Diese besitzt eine **5'→3' Exonuklease-Aktivität**, mit der sie die restlichen RNA-Nukleotide am 5'-Ende des Primers entfernen kann.
    - Gleichzeitig ersetzt die DNA-Polymerase I die entfernte RNA mit **neuer DNA**, indem sie die Lücke, die durch die entfernte RNA entstanden ist, auffüllt.
- **Synthese der fehlenden DNA und Schließen der Lücken**:
    - Nachdem die RNA vollständig entfernt und die DNA-Polymerase I die Lücken mit DNA gefüllt hat, bleibt am Ende der Okazaki-Fragmente eine **kleine Lücke** (auch als „Nick“ bezeichnet) zwischen den DNA-Fragmenten.
    - Diese Lücke kann die DNA-Polymerase I nicht schließen, da sie die Phosphodiesterbindung zwischen den benachbarten Nukleotiden nicht knüpfen kann.
- **DNA-Ligase schließt die Lücken**:
    - Die **DNA-Ligase** schließt die Lücken, indem sie eine **Phosphodiesterbindung** zwischen den benachbarten DNA-Fragmenten herstellt. Dadurch werden die Okazaki-Fragmente miteinander verbunden, sodass der Folgestrang ein kontinuierlicher DNA-Strang wird.
![[Pasted image 20241031103442.png#invert|400]]
### Bei Eukaryoten
- Bei Eukaryoten wird der Primer von einer speziellen **Primase** synthetisiert, die eng mit der **DNA-Polymerase α (Pol α)** assoziiert ist.
- Sobald der Primer und der erste DNA-Abschnitt synthetisiert sind, wird die DNA-Polymerase α durch andere Polymerasen ersetzt:
    - **DNA-Polymerase ε (Epsilon)** für den **Leitstrang** (leading strand).
    - **DNA-Polymerase δ (Delta)** für den **Folgestrang** (lagging strand).
- Diese Polymerasen übernehmen die eigentliche Synthese der DNA und setzen die Replikation fort.
![[Pasted image 20241031103935.png#invert|200]]
#### Entfernen des RNA-Primers
- **Verdrängungssynthese durch DNA-Polymerase δ**:
    - Auf dem Folgestrang wird die **DNA-Polymerase δ (Pol δ)** aktiv, wenn sie auf einen RNA-Primer trifft. Sie verdrängt den RNA-Primer, indem sie DNA synthetisiert und dabei den RNA-Abschnitt quasi „überschreibt“.
- **Bindung von Replication Protein A (RPA)**:
    - Das **Replication Protein A (RPA)** bindet an den verdrängten Einzelstrang (den RNA-Überhang), um die Struktur zu stabilisieren und zu verhindern, dass sich die Einzelstrangbereiche wieder zurückfalten.
- **Rekrutierung der Endonuklease Fen1**:
    - Die Endonuklease **Fen1** (Flap Endonuclease 1) wird rekrutiert, um den verbleibenden RNA-Überhang abzutrennen.
    - **Fen1** schneidet den überstehenden RNA-Teil, sodass die RNA vollständig entfernt wird und eine saubere „Nick“ (Einzelstrangbruch) zwischen den DNA-Fragmenten verbleibt.
- **Entfernung des Überhangs und Verknüpfung**:
    - In einigen Fällen wird eine zusätzliche Endonuklease benötigt, um den Überhang vollständig zu entfernen, falls er länger ist und Fen1 ihn nicht alleine abtrennen kann.
    - Schließlich wird die **DNA-Ligase** aktiv und schließt den Nick, indem sie eine **Phosphodiesterbindung** zwischen den benachbarten DNA-Fragmenten herstellt. So entsteht ein durchgehender DNA-Strang.
![[Pasted image 20241031104011.png#invert|200]]
## Regulation der Replikation bei Prokaryoten
- In Prokaryoten gibt es spezielle DNA-Sequenzen, die als **Replikator** bezeichnet werden und als Startpunkt der Replikation dienen.
- Ein **Initiator-Protein** bindet an diese Replikator-DNA, wodurch die Replikation eingeleitet wird und die DNA sich zu verdoppeln beginnt.
![[Pasted image 20241107100557.png#invert|200]]
### Struktur von Replikatoren
![[Pasted image 20241107100650.png#invert|300]]
- Das Bild zeigt verschiedene Replikator-Regionen für Prokaryoten wie _E. coli_, wo der Replikator aus spezifischen Sequenzen wie AT-reichen Bereichen (die leicht entwunden werden können) und Bindestellen für Initiator-Proteine besteht.
- Diese Bindestellen sind entscheidend, da sie den Startpunkt der DNA-Synthese markieren.
### Funktion der Initiatorproteine
![[Pasted image 20241107100749.png#invert|300]]
- Sie binden an die DNA, um die Replikation zu starten.
- Sie trennen die DNA-Stränge durch eine leichte Schmelzung der AT-reichen Region, wodurch die Replikationsmaschinerie Zugang erhält.
- Sie rekrutieren weitere Proteine, die für die Replikation notwendig sind.
### Initiation der DNA-Replikation in E. coli
![[Pasted image 20241107100856.png#invert|200]]
![[Pasted image 20241107100915.png#invert|200]]

- **DnaA-ATP** bindet an spezifische 9-mer Motive in der OriC-Region (dem Replikationsursprung).
- Diese Bindung ermöglicht die Entwindung der DNA in der benachbarten AT-reichen Region, was die Stränge trennt und den Replikationsstart erleichtert.
- **DnaA-ATP** rekrutiert die **DnaB-Helicase** und den **Helicase-Loader (DnaC)**, die für die Entwindung der DNA weiter sorgen.
- Die Konversion von DnaA-ATP zu DnaA-ADP reguliert die Initiation, da dieser Schritt durch den Austausch von ATP langsam ist und verhindert, dass die Replikation unkontrolliert gestartet wird.
- Nachdem die Helicase die DNA-Stränge entwindet, wird die **Primase** rekrutiert, die kurze RNA-Primer für den Start der DNA-Synthese synthetisiert.
- Die verbleibenden DnaA-Proteine werden entfernt, und der Priming-Komplex zieht die DNA-Polymerase III an, die die bidirektionale Synthese des Leit- und Folgestrangs beginnt.
### Methylierung
- Die DNA-Replikation muss eng mit der Zellteilung koordiniert werden, um sicherzustellen, dass die Chromosomenanzahl in jeder Zelle konstant bleibt.
- In _E. coli_ wird dies durch das Zusammenspiel der **Dam-Methylase** und des **SeqA-Proteins** in Verbindung mit dem Initiatorprotein **DnaA-ATP** kontrolliert.
- Die **Dam-Methylase** erkennt bestimmte DNA-Sequenzen (GATC) und methyliert die Adenin-Basen in diesen Bereichen.
- Die Methylierung dient als Markierung dafür, dass ein DNA-Strang bereits „alt“ ist, d.h., dass er bereits einmal repliziert wurde.
- Da der neue Strang un- oder nur halb-methyliert ist, kann das Reparatursystem der Zelle (Mismatch-Reparatursystem) diesen neuen Strang von dem alten unterscheiden.
- Bei Fehlern während der Replikation überprüft das System beide Stränge. Da der alte Strang methyliert ist, wird der nicht-methylierte (neue) Strang als fehlerhaft erkannt und repariert, um Mutationen zu vermeiden.
## DNA-Replikation bei Eukaryoten
Die DNA-Replikation in Eukaryoten ist ein komplexer Prozess, der sicherstellt, dass die gesamte DNA während der S-Phase des Zellzyklus einmal und nur einmal verdoppelt wird. Hier sind die wichtigsten Schritte und Mechanismen, die zur Regulation und Durchführung der Replikation beitragen:

1. **Einmalige Replikation pro Zellzyklus**:
    
    - Während der S-Phase muss jede Region der DNA genau einmal repliziert werden, um sicherzustellen, dass die Chromosomenzahl konstant bleibt. Unreplizierte Bereiche könnten während der Zellteilung zu Chromosomenbrüchen führen, und doppelte Replikation verursacht Kopieanzahlanomalien.
2. **Replikationsursprünge (Origins of Replication)**:
    
    - Die eukaryotische DNA hat viele Replikationsursprünge, an denen die Replikation beginnen kann. Die Replikation startet jedoch nicht an allen Ursprüngen gleichzeitig. Nur ausgewählte Ursprünge werden aktiviert, während andere passiv repliziert werden können, wenn die Replikationsgabel sie erreicht.
3. **Inaktivierung der Replikationsursprünge nach Initiation**:
    
    - Sobald ein Replikationsursprung aktiviert wird, wird er inaktiviert und kann während des laufenden Zellzyklus nicht erneut aktiviert werden. Dies verhindert die mehrfache Replikation derselben DNA-Region.
    - Wie im Bild gezeigt, beginnen einige Ursprünge mit der Replikation, während andere Ursprünge „passiv“ repliziert werden, indem die Replikationsgabel diese Bereiche erreicht und ohne eine eigenständige Initiation repliziert.
4. **Koordination der Replikation**:
    
    - Aufgrund der großen Anzahl an Replikationsursprüngen ist eine strenge Kontrolle erforderlich, um sicherzustellen, dass jede DNA-Region genau einmal repliziert wird. Dies wird durch die zeitlich abgestimmte Aktivierung der Ursprünge und die Inaktivierung nach Beginn der Replikation gewährleistet.
    - So werden z. B. in der Abbildung die Ursprünge 3 und 5 zuerst aktiviert, dann Ursprung 1, wodurch die übrigen Ursprünge passiv repliziert werden.
5. **Prä-Replikationskomplexe (Pre-RC) und Lizenzierung**:
    
    - Vor der S-Phase wird an jedem Replikationsursprung ein Prä-Replikationskomplex (Pre-RC) gebildet, was als „Lizenzierung“ bezeichnet wird. Dieser Komplex markiert die Ursprünge für die spätere Aktivierung während der S-Phase.
    - Nach der Aktivierung werden die Pre-RCs abgebaut oder inaktiviert, was eine erneute Replikation verhindert.
## Inaktivierung der Replikationsursprünge durch Replikation
![[Pasted image 20241114094919.png#invert|300]]
- In dieser Abbildung wird gezeigt, dass die Replikationsursprünge (in Grün markiert) nur einmal pro Zellzyklus aktiviert werden dürfen.
- Die Replikation startet an bestimmten Ursprüngen, wie an den Ursprüngen 3 und 5, wodurch die DNA in diesen Bereichen repliziert wird.
- Einmal aktiviert, werden diese Replikationsursprünge "inaktiviert", sodass sie während desselben Zellzyklus nicht erneut aktiviert werden können. Dies ist wichtig, um eine erneute Replikation und mögliche Kopienanzahlfehler zu vermeiden.
- Die anderen Ursprünge werden passiv repliziert, wenn die Replikationsgabel diese Bereiche erreicht. Dies stellt sicher, dass alle Regionen der DNA einmal, aber nicht mehrfach, repliziert werden.
## Terminierung der DNA-Replikation in _E. coli_
![[Pasted image 20241114095338.png#invert |100]]
In _E. coli_ trifft die Replikation auf eine spezielle Terminierungsregion, die **ter-Sequenzen** enthält. Diese Sequenzen dienen als "Stoppsignal" für die Replikationsgabeln, wenn sie sich aufeinander zubewegen.
1. **ter-Sequenzen und Tus-Protein**:
    - Die **ter-Sequenzen** binden das **Tus-Protein**, das die Weiterführung der Replikation blockiert, indem es die Helicase (das Enzym, das die DNA-Stränge trennt) hemmt.
    - Dadurch laufen die Replikationsgabeln zusammen und beenden die Replikation an den Terminationsstellen, ohne dass sie aneinander vorbeilaufen.
2. **Catenane und Topoisomerase II**:
    - Nach der Replikation entstehen zwei ringförmige DNA-Moleküle, die ineinander verschlungen sind, sogenannte **Catenane**.
    - Die **Topoisomerase II** entwirrt diese ineinander verschlungenen Moleküle durch einen Prozess namens **Decatenation** und ermöglicht so, dass die beiden DNA-Ringe voneinander getrennt werden können, um korrekt an die Tochterzellen weitergegeben zu werden.
## Problem bei der Replikation der Telomere in Eukaryoten
Eukaryotische Chromosomen sind linear, und bei der Replikation entsteht ein Problem an den Enden (Telomeren) der DNA, speziell auf dem 3'-5'-Strang (Leitstrang).
1. **Problem des fehlenden Primers am 3'-Ende**:
![[Pasted image 20241114095458.png#invert|300]]
    - Die DNA-Polymerase benötigt einen Primer, um die DNA-Synthese zu starten. Am Ende des 3'-5'-Strangs fehlt jedoch ein Platz für die Bindung eines Primers, was zur Verkürzung des Chromosoms bei jeder Zellteilung führt, da das letzte Stück des Folgestrangs (Problemstrang) nicht repliziert werden kann.
    - -> Der Strang würde immer kürzer werden je öfter er repliziert wird
1. **Lösung durch die Telomerase**:
    - **Telomerase** ist ein spezielles Enzym, das die Telomere (Enden der Chromosomen) verlängert. Es trägt eine RNA-Matrize, die komplementär zu den Telomer-Sequenzen ist und als Vorlage dient, um neue Telomer-DNA-Sektionen zu synthetisieren.
    - **Verlängerung durch Telomerase**:
        - Die Telomerase bindet an das 3'-Ende und synthetisiert zusätzliche DNA-Sequenzen, wodurch das Chromosomenende verlängert wird.
        - Danach wird der neu synthetisierte Abschnitt als Vorlage verwendet, um die fehlende DNA des Problemstrangs zu ergänzen.
3. **t-Loop-Bildung und Stabilisierung**:
![[Pasted image 20241114095616.png#invert|300]]
    - Die verlängerten Telomere bilden eine **t-Loop-Struktur**, bei der das 3'-Ende sich selbst überlappt und in die Telomerregion faltet, was zur Stabilität und zum Schutz des Chromosomenendes beiträgt.
    - Diese t-Loop-Struktur verhindert, dass die Chromosomenenden als DNA-Schäden erkannt und abgebaut werden.
![[Telomere_Replication.mov]]
