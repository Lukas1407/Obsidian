## Arten von Bioinformatik-Datenbanken
1. **Sequenzdatenbanken**
    - Hier werden <mark style="background: #FFB86CA6;">Sequenzinformationen zu Nukleotiden (DNA, RNA) und Proteinen</mark> gesammelt.
    - Beispiele: <mark style="background: #FFB86CA6;">GenBank</mark> (NIH, Bethesda, USA), <mark style="background: #FFB86CA6;">EMBL</mark> (European Molecular Biology Laboratory), DDBJ (Japan).
    - Inhalt: Genom- und Transkript-Sequenzen, Protein-Codierung, Annotationen (z. B. Genfunktionen, Organismusherkunft).
2. **Strukturdatenbanken**
    - Speichern <mark style="background: #FFB86CA6;">3D-Strukturen von Proteinen oder anderen Molekülen</mark>.
    - Einsatz vor allem in der <mark style="background: #FFB86CA6;">Strukturanalyse und beim Protein-Design</mark>.
    - Beispiel: Protein Data Bank (PDB).
3. **Pathway-Datenbanken**
    - <mark style="background: #FFB86CA6;">Beschreiben biochemische Stoffwechselwege und regulatorische Pfade</mark> (<mark style="background: #FFB86CA6;">z. B. Genexpression</mark>, Signaltransduktion).
    - Beispiel: KEGG (Kyoto Encyclopedia of Genes and Genomes), Reactome.
    - Man <mark style="background: #FFB86CA6;">kann hier erkennen, welche Enzyme in einem bestimmten Stoffwechselweg aktiv sind, wie Gene reguliert werden usw</mark>.
4. **Wissensdatenbanken**
    - <mark style="background: #FFB86CA6;">Fassen strukturelle und funktionale Informationen</mark> aus vielen Quellen zusammen.
    - Beispiel: **PubMed** als Literaturdatenbank (zugleich aber auch Wissensbasis mit Metadaten), UniProt als kombinierte Protein-Wissensdatenbank.
    - Dienen zur **Recherche** und zum **Verknüpfen** von Daten aus unterschiedlichen Studien.
### Typische Probleme in solchen Datenbanken
- **Redundanz**
    - Es gibt <mark style="background: #FFB86CA6;">oft viele Varianten oder Kopien derselben Sequenz</mark> (z. B. wenn <mark style="background: #FFB86CA6;">Gene mehrfach in verschiedenen Stämmen oder Organismen</mark> sequenziert wurden).
    - Das erschwert manchmal die Datenanalyse, weil man redundante Einträge filtern muss.
- **Qualität**
    - <mark style="background: #FFB86CA6;">Manche Einträge stammen aus unbekannter oder unsicherer Quelle</mark>, andere sind **fehlerhaft** oder von **minderer Qualität** (z. B. unvollständige Sequenzen).
    - Viele Datenbanken erlauben es, dass Forschende direkt Rohdaten hochladen, ohne vollständige Validierung – das kann zu Fehlern führen.
- **Automatische Annotation**
    - <mark style="background: #FFB86CA6;">Um große Datenmengen schnell zu beschreiben, wird oft eine automatische Annotationspipeline verwendet</mark>.
    - Diese ist <mark style="background: #FFB86CA6;">nicht immer zuverlässig</mark>: Fehler in einer Referenz-Annotation können sich „vererben“ und zu vielen fehlerhaften Einträgen führen.
### Meist benutze Datanbanken
- GenBank
- EMBL
## Sequenzanalyse
- <mark style="background: #FFB86CA6;">Homologe Sequenzen sind ähnliche Sequenzen, die in verschiedenen Organismen oder Geweben vorkommen</mark> und für ein <mark style="background: #FFB86CA6;">Protein mit ähnlicher Funktion kodieren</mark>.
- **Orthologe Sequenzen:** <mark style="background: #FFB86CA6;">Entstehen durch Artbildung</mark>. Sie finden sich <mark style="background: #FFB86CA6;">in unterschiedlichen Spezies und haben oft ähnliche Funktionen</mark>
- **Paraloge Sequenzen:** <mark style="background: #FFB86CA6;">Entstehen durch Genverdopplung innerhalb derselben Spezies</mark>. Diese Gene können sich <mark style="background: #FFB86CA6;">im Laufe der Evolution spezialisieren und verschiedene Funktionen übernehmen</mark>.
![[Pasted image 20250113125932.png#invert|600]]
### **Beispiel: Homologiesuche beim Mausgenom**
- **Ergebnisse einer Suche:**
    - <mark style="background: #FFB86CA6;">Nur 1% der Gene sind spezifisch für Nagetiere</mark>.
    - 27% der Gene kommen in anderen Metazoa (vielzelligen Tieren) vor.
    - 29% der Gene existieren in anderen Eukaryoten.
    - Restliche Anteile zeigen Verbindungen zu anderen Wirbeltieren oder Säugetieren.
### Alignment
- Alignment" bedeutet, <mark style="background: #FFB86CA6;">Sequenzen so auszurichten</mark>, dass <mark style="background: #FFB86CA6;">möglichst viele Positionen übereinstimmen</mark>. <mark style="background: #FFB86CA6;">Ziel ist es, Homologien zu identifizieren</mark>.
- **Vorgehen:** Sequenzen werden nebeneinander geschrieben und optimiert, sodass **Matches**, **Mismatches** und **Lücken (Gaps)** sichtbar werden.
#### Match
![[Pasted image 20250113124326.png#invert|50]]
#### Missmatch
![[Pasted image 20250113124340.png#invert|50]]
#### Gap
![[Pasted image 20250113124352.png#invert|50]]

#### Verschiedene Alignment-Typen
- **Lokal:** <mark style="background: #FFB86CA6;">Nur Teilstücke der Sequenz werden verglichen</mark>, um Regionen mit hoher Ähnlichkeit zu finden.  
    → Beispiel: Ein kleines konserviertes Motiv in einem Protein.
- **Global:** <mark style="background: #FFB86CA6;">Die gesamte Sequenz wird ausgerichtet</mark>, von Anfang bis Ende.  
    → Beispiel: Vergleich zweier Gene ähnlicher Länge.
- **Multiple:** <mark style="background: #FFB86CA6;">Mehrere Sequenzen werden gleichzeitig ausgerichtet</mark>, <mark style="background: #FFB86CA6;">um konservierte Regionen in vielen Organismen zu identifizieren</mark>.
#### Wie gut ist ein Alignment?
- **Kostenfunktion:** Bewertet, wie „gut“ ein Alignment ist.
    - **Match:** Kosten = 0 (keine Strafe).
    - **Mismatch:** Kosten = 1 (leichte Strafe).
    - **Gap:** Kosten = 2 (höhere Strafe, <mark style="background: #FFB86CA6;">weil Gaps oft biologisch unwahrscheinlicher sind</mark>).
- **Berechnung:** Die Gesamtkosten werden aus der Summe der Strafen für Mismatches und Gaps berechnet. Ziel ist es, ein Alignment mit minimalen Kosten zu finden.
![[Pasted image 20250113124430.png#invert|400]]
### BLAST – Basic Local Alignment Search Tool
Ein Algorithmus, der lokale Ähnlichkeiten zwischen Sequenzen (DNA, RNA, Proteine) findet. <mark style="background: #FFB86CA6;">Er wird oft verwendet, um eine Abfrage-Sequenz mit Datenbank-Sequenzen zu vergleichen</mark>.
- **Eigenschaften:**
    - **Keine optimalen Ergebnisse:** BLAST verwendet <mark style="background: #FFB86CA6;">keine dynamische Programmierung</mark> (wie z. B. Needleman-Wunsch), <mark style="background: #FFB86CA6;">sondern Heuristiken (Schätzmethoden), um schneller zu sein</mark>.
    - **Sehr schnell:** Weil es <mark style="background: #FFB86CA6;">keine optimale Lösung sucht</mark>, ist es viel schneller als andere Alignment-Methoden.
    - **Weit verbreitet:** Wird in fast allen bioinformatischen Anwendungen verwendet.
#### Schritte
- **Seeding:**
    - <mark style="background: #FFB86CA6;">Sucht gemeinsame Subwörter</mark> (kleine Teilsequenzen, sogenannte Seeds) <mark style="background: #FFB86CA6;">zwischen der Abfrage-Sequenz und den Sequenzen in der Datenbank</mark>.
    - Diese Seeds dienen als Startpunkte für das Alignment.
- **Extension:**
    - Die Seeds werden in **beide Richtungen erweitert**, um längere Segmente zu finden, die gut zusammenpassen.
    - Diese Segmente werden **HSPs (High-Scoring Segment Pairs)** genannt.
- **Evaluation:**
    - Es wird bewertet, wie **signifikant** die gefundenen HSPs sind.
    - Statistische Methoden (z. B. der **E-Value**) helfen zu bestimmen, ob die Übereinstimmungen biologisch relevant sind oder zufällig auftreten.
#### Beispiel
1. Seeding
	- Die <mark style="background: #FFB86CA6;">Abfrage-Sequenz wird in kleine Stücke (Subwörter) zerlegt</mark>
![[Pasted image 20250113124905.png#invert|600]]
1. Alignment der Stücke
	- Die <mark style="background: #FFB86CA6;">gefundenen Seeds (Subwörter) werden verglichen</mark>.
	- Andere Teile der Sequenz, die nicht zugeordnet werden konnten (z. B. „GL“), bleiben übrig und werden später behandelt.
![[Pasted image 20250113125132.png#invert|600]]
1. Alignment der Lücken
	- <mark style="background: #FFB86CA6;">Nicht zugeordnete Teile der Sequenz (z. B. „GL“ und „RQL“) werden durch Lücken (Gaps) eingefügt</mark>, um ein möglichst gutes Alignment zu erzeugen.
![[Pasted image 20250113125050.png#invert|600]]
![[Pasted image 20250113125148.png#invert|600]]
### Warum ist BLAST nicht optimal
- BLAST verwendet **Heuristiken** (Schätzmethoden), um Zeit zu sparen.
- Es kann von der optimalen Lösung abweichen, da es keine vollständige dynamische Programmierung verwendet.
- Das Ergebnis ist eine **empirische Annäherung** und keine exakte Wissenschaft.
#### BLAST Evaluation - E-Value
**Was ist der E-Value?**
- <mark style="background: #FFB86CA6;">Ein statistischer Wert, der angibt, wie oft eine Übereinstimmung (Score) zufällig auftreten würde, basierend auf der Datenbankgröße</mark>.
- Formel:  
    $$E ≈ 1 - e^{-p(S>x) \cdot d} $$
    (Hierbei berücksichtigt „d“ die Datenbankgröße.)
- Ein **kleiner E-Value** (nahe 0) bedeutet, <mark style="background: #FFB86CA6;">dass die Übereinstimmung sehr wahrscheinlich biologisch relevant ist</mark>.
- Ein **hoher E-Value** deutet darauf hin, <mark style="background: #FFB86CA6;">dass die Übereinstimmung zufällig sein könnte</mark>.

## Darstellung der Suchergebnisse in der Datenbank
1. **FASTA-Format:**
    - **Definition:** Das FASTA-Format ist ein Standardformat, um Sequenzdaten darzustellen.
    - **Aufbau:**
        - **Kopfzeile (Header):** Beginnt mit einem `>`, enthält Informationen wie Sequenz-ID, Organismus und Beschreibung.  
            Beispiel: `>AAW51149.1 inorganic phosphate transporter PT4 [Solanum tuberosum]`
        - **Sequenz:** Die eigentliche DNA-, RNA- oder Proteinsequenz (in einer Zeile oder über mehrere Zeilen verteilt).
    - **Anwendung:** Dieses Format wird häufig für Datenbankabfragen oder Sequenzvergleiche verwendet.
2. **Graphische Darstellung:**
    - **Definition:** Eine visuelle Darstellung der Sequenzmerkmale (Features).
    - **Beispiele für Features:**
        - **Proteindomänen** (z. B. CDD – Conserved Domain Database).
        - Spezifische Regionen oder funktionale Abschnitte der Sequenz.
    - Diese Darstellung erleichtert die Analyse und Identifikation von Schlüsselbereichen.
3. **Flat File:**
    - **Definition:** Textbasierte Darstellung von Sequenzinformationen und Annotationen.
    - **Inhalte:**
        - Organismus-Informationen.
        - Quellen (Referenzen) und Annotationen.
        - Spezifische Sequenzregionen oder Domänen.

## Phylogenetische Bäume
1. **Stammbäume:**
![[Pasted image 20250113125747.png#invert|400]]
    - Zeigen die <mark style="background: #FFB86CA6;">evolutionären Beziehungen zwischen verschiedenen Organismen</mark> (Bakterien, Archaea, Eukaryoten).
    - **Gene als „Blätter“:** Die Verzweigungen repräsentieren evolutionäre Linien.
1. **Was sagen phylogenetische Bäume aus?**
    - **Funktion eines unbekannten Gens:** <mark style="background: #FFB86CA6;">Bäume können Hinweise auf die Funktion geben, basierend auf den nächsten Verwandten</mark>.
    - **Evolution:** Sie zeigen den <mark style="background: #FFB86CA6;">Ablauf der Evolution</mark> und die verwandtschaftlichen Beziehungen zwischen Organismen.
    - **Neofunktionalisierung:** Identifizieren, <mark style="background: #FFB86CA6;">wie Gene nach einer Genverdopplung neue Funktionen annehmen</mark>.
2. **Interpretation von phylogenetischen Bäumen:**
    - **Horizontale Linien:** Repräsentieren evolutionäre Veränderungen über die Zeit.
    - **Länge der Äste:** Zeigt die **evolutionäre Distanz** (z. B. Anzahl von Substitutionen pro Sequenzposition).
    - Einheit: **Substitutionsrate** (Änderungen pro Position).

## Vergleichende Genomik
- Vergleichende Genomik <mark style="background: #FFB86CA6;">untersucht die Genomsequenzen von zwei oder mehr Arten, um Gemeinsamkeiten oder Unterschiede zu finden.</mark>
- Ziel: <mark style="background: #FFB86CA6;">Verständnis von Genomsequenzen</mark> und <mark style="background: #FFB86CA6;">Identifizierung von ähnlichen Genen zwischen verschiedenen Spezies.</mark>
### **Worin gleichen und unterscheiden sich Arten?**
1. **Ähnlichkeiten:**
    - <mark style="background: #FFB86CA6;">Gene für grundsätzliche Prozesse</mark> wie <mark style="background: #FFB86CA6;">Zellteilung oder Energiegewinnung</mark> sind zwischen vielen Arten ähnlich.
2. **Unterschiede:**
    - <mark style="background: #FFB86CA6;">Gene für spezielle Anpassungen</mark>, wie z. B. Hitzeresistenz in Wüstenpflanzen oder Kälteresistenz in Polartieren.
3. **Evolution:** <mark style="background: #FFB86CA6;">Vergleichende Genomik hilft dabei, Evolutionsmechanismen und -prozesse besser zu verstehen</mark>.
### **Warum vergleichende Genomik?**
- **Potentielle Funktion eines unbekannten Gens**:
    - Wenn ein <mark style="background: #FFB86CA6;">unbekanntes Gen</mark> einer Art ähnlichen Genen in anderen Organismen entspricht, <mark style="background: #FFB86CA6;">kann seine Funktion vorhergesagt werden</mark>.
- **Konservierte Genabschnitte**:
    - <mark style="background: #FFB86CA6;">Manche Gene bleiben in allen Nachkommen gleich und deuten auf essenzielle Funktionen hin</mark>.
### **Konservierte Gene und Uhren-Beispiel**
- **Konservierte Gene = Essenzielle Bestandteile**:
    - Vergleichbar mit den Zahnrädern einer Uhr – sie sind in jeder Uhrart (z. B. Pendeluhr, Armbanduhr) vorhanden, weil sie essenziell sind.
- **Vergleich spezifischer Teile**:
    - Spezielle Bestandteile wie das Pendel (nur in Pendeluhren) zeigen Anpassungen und Unterschiede.

## Syntenie
- **Definition:** <mark style="background: #FFB86CA6;">Gene, die in der gleichen relativen Position auf zwei Chromosomen liegen</mark> (z. B. in Mensch und Maus).
- **Nutzen:**
    - Syntenie hilft, Gene anhand ihrer Position zu identifizieren.
    - Vergleich zeigt, dass verwandte Arten oft ähnliche Genanordnungen haben.
### **Syntenie beim Vergleich Mensch und Maus**
- **Beobachtung:** Viele Gene auf einem Chromosom des Menschen finden sich in einer ähnlichen Anordnung auf einem Maus-Chromosom.
- **Bedeutung:** Dies deutet darauf hin, dass sich große Bereiche des Genoms im Verlauf der Evolution nicht verändert haben.