- Stellt eine vollständige Kopie der [[Desoxyribonukleinsäure|DNA]] her
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
