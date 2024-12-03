### **1. Degenerierte Oligonukleotide**

#### **Degeneration des genetischen Codes:**

- Viele Aminosäuren werden durch mehrere Codons kodiert. Zum Beispiel hat Leucin sechs verschiedene Codons.
- Um eine spezifische DNA-Sequenz für ein Peptid zu designen, müssen alle möglichen Codon-Varianten berücksichtigt werden, was zu **degenerierten Oligonukleotiden** führt.

#### **Beispiel für ein Peptid:**

1. Die Aminosäuresequenz lautet: **Cys - Ile - Tyr - Met - His - Gln - Asp**.
2. Für jede Aminosäure werden die Codons mit den jeweiligen Varianten aufgelistet:
    - Cys: **TGT/TGC** → 2 Möglichkeiten
    - Ile: **ATT/ATC/ATA** → 3 Möglichkeiten
    - Tyr: **TAT/TAC** → 2 Möglichkeiten
    - usw.
3. Die Gesamtanzahl der möglichen DNA-Sequenzen ergibt sich durch Multiplikation der Codon-Varianten: 2×3×2×...2 \times 3 \times 2 \times ....

#### **Reduktion der Degeneration:**

- Die Degeneration kann durch die Auswahl konservierter Sequenzen oder durch den Einsatz von Peptiden mit weniger degenerierten Codons (z. B. Methionin oder Tryptophan mit nur einem Codon) reduziert werden.

### **2. Klausurfrage: Geeignetes Peptid für Oligonukleotid-Sonden**

#### **Frage a): Welches Peptid ist besser geeignet?**

- Die beiden Peptide unterscheiden sich in ihrer Degeneration:
    - Peptid 1: **Asp - Phe - Gly - Lys - Asp - Tyr - Tyr - Ala**
    - Peptid 2: **Phe - Tyr - Ile - Asp - Lys - Phe - Gln - Val**
- Durch Berechnung der Codon-Degeneration ergibt sich:
    - Peptid 1 hat 210=10242^{10} = 1024 mögliche Sequenzen.
    - Peptid 2 hat 28=2562^8 = 256 mögliche Sequenzen.
- **Ergebnis:** Peptid 2 ist besser geeignet, da es eine geringere Degeneration aufweist.


### **3. Primer-Design**

Primers sind kurze DNA-Sequenzen, die an spezifische Regionen der DNA binden, um die DNA-Amplifikation durch Polymerase-Kettenreaktion (PCR) zu ermöglichen.

#### **Vorgehen beim Primer-Design:**

1. **Forward-Primer:**
    - Wird aus der angegebenen DNA-Sequenz in 5'-3'-Richtung abgeleitet.
    - Beispiel für Peptid 1: Forward-Primer ist 5′5' **ATT CTT ATT AAT TGT CAA 3'**.
2. **Reverse-Primer:**
    - Ist die komplementäre und in 5'-3'-Richtung umgekehrte Sequenz des antisense-Strangs.
    - Beispiel für Peptid 1: Reverse-Primer ist 5′5' **TTG ACA ATT AAT AAG AAT 3'**.

### **4. Auswahl und Optimierung von Primern**

#### **Warum A mit D und C mit B?**

- Die Primer müssen spezifisch sein und keine unspezifischen Bindungen eingehen.
- Forward- und Reverse-Primer müssen in entgegengesetzten Richtungen liegen, um die richtige Region der DNA zu amplifizieren.
- Primer sollten eine ähnliche Schmelztemperatur (Tm) haben, damit sie während der PCR gleichzeitig binden können.

#### **Fehlervermeidung:**

- Vermeide zu viele degenerierte Basen.
- Berücksichtige Sekundärstrukturen, die die Bindungseffizienz der Primer beeinflussen könnten.
