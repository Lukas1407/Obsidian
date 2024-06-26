- **Theorem**:
  - Sei $\Sigma$ eine Signatur der Prädikatenlogik erster Stufe (PL1).
  - Dann ist $H$ über $\Sigma$ sowohl korrekt als auch vollständig.
  - Formal: Für alle $M \subseteq \text{For}_\Sigma$ und $A \in \text{For}_\Sigma$ gilt:
    $$
    M \models A \iff M \vdash_H A
    $$
- **Bedeutung**:
  - $M \models A$ bedeutet, dass $A$ in allen Modellen von $M$ wahr ist (semantische Folgerung).
  - $M \vdash_H A$ bedeutet, dass $A$ aus $M$ im formalen System $H$ abgeleitet werden kann (syntaktische Folgerung).
- **Korrektheit**:
  - Wenn eine Formel $A$ aus einer Menge von Formeln $M$ im formalen System $H$ abgeleitet werden kann ($M \vdash_H A$), dann ist $A$ auch in allen Modellen von $M$ wahr ($M \models A$).
- **Vollständigkeit**:
  - Wenn eine Formel $A$ in allen Modellen von $M$ wahr ist ($M \models A$), dann kann $A$ auch aus $M$ im formalen System $H$ abgeleitet werden ($M \vdash_H A$).
