Lassen wir uns die Konzepte der relativen Allgemeingültigkeit und die verschiedenen Eigenschaften der Relationen $R$ in Kripke-Strukturen erklären.

### Relative Allgemeingültigkeit

Eine Formel kann unter bestimmten Bedingungen allgemeingültig sein, auch wenn sie nicht in allen Kripke-Strukturen allgemeingültig ist. Das bedeutet, dass die Formel in allen Kripke-Strukturen gilt, die eine bestimmte Eigenschaft haben.

#### Beispiel: $\square A \to A$

- Die Formel $\square A \to A$ ist nicht allgemeingültig in allen Kripke-Strukturen.
- Aber in allen Kripke-Strukturen $K = (S, R, I)$, bei denen $R$ reflexiv ist (d.h., jeder Zustand ist zugänglich von sich selbst), gilt $\square A \to A$.

### Allgemeingültige Formeln und Eigenschaften von $R$

Verschiedene logische Formeln sind unter bestimmten Bedingungen allgemeingültig, die auf die Eigenschaften der Relation $R$ in Kripke-Strukturen zurückzuführen sind. Hier sind einige wichtige Beispiele:

#### Reflexive Relation

- **$\square P \to P$:** Wenn $R$ reflexiv ist, dann gilt $\square P \to P$. Das bedeutet, wenn etwas in allen zugänglichen Zuständen (einschließlich des Zustands selbst) wahr ist, dann ist es auch im aktuellen Zustand wahr.
- **$\square P \to \square \square P$:** Wenn $R$ transitiv ist, dann gilt $\square P \to \square \square P$. Das bedeutet, wenn etwas notwendig wahr ist, dann ist es auch notwendig notwendig wahr.
- **$P \to \square \Diamond P$:** Wenn $R$ symmetrisch ist, dann gilt $P \to \square \Diamond P$. Das bedeutet, wenn etwas wahr ist, dann ist es möglich, dass es notwendig wahr ist.

#### Transitive Relation

- **$\square P \to \square \square P$:** Wenn $R$ transitiv ist, dann gilt $\square P \to \square \square P$. Das bedeutet, wenn etwas notwendig wahr ist, dann ist es auch notwendig notwendig wahr.
- **$\Diamond \Diamond P \to \Diamond P$:** Wenn $R$ transitiv ist, dann gilt $\Diamond \Diamond P \to \Diamond P$. Das bedeutet, wenn es möglich ist, dass es möglich ist, dann ist es einfach möglich.

#### Symmetrische Relation

- **$P \to \square \Diamond P$:** Wenn $R$ symmetrisch ist, dann gilt $P \to \square \Diamond P$. Das bedeutet, wenn etwas wahr ist, dann ist es möglich, dass es notwendig wahr ist.

#### Dichte Relation

- **$\square \square P \to \square P$:** Wenn $R$ dicht ist, dann gilt $\square \square P \to \square P$. Das bedeutet, wenn etwas notwendig notwendig wahr ist, dann ist es notwendig wahr.
- **Für alle $t_1, t_2 \in S$ mit $R(t_1, t2)$ existiert $t3 \in S$ mit $R(t1, t3)$ und $R(t3, t2)$.** Das bedeutet, dass zwischen jedem Paar von Zuständen immer ein weiterer Zustand existiert, der beide verbindet.

#### Partielle Funktionalität

- **$\Diamond P \to \square P$:** Wenn $R$ partiell funktional ist, dann gilt $\Diamond P \to \square P$. Das bedeutet, dass wenn es möglich ist, dass $P$ wahr ist, dann ist $P$ notwendig wahr.

#### Endliche Relation

- **$\square P \to \Diamond P$:** Wenn $R$ endlos ist, dann gilt $\square P \to \Diamond P$. Das bedeutet, dass wenn etwas notwendig wahr ist, dann ist es möglich wahr.

### Zusammenfassung der Eigenschaften

| Eigenschaft von $R$ | Allgemeingültige Formel |
|-------------------------|-------------------------|
| Reflexiv                | $\square P \to P$     |
| Reflexiv                | $P \to \Diamond P$    |
| Reflexiv                | $\square \square P \to \square P$ |
| Reflexiv                | $\square \Diamond P \to \Diamond P$ |
| Reflexiv                | $\square P \to \Diamond \square P$ |
| Transitiv               | $\Diamond \Diamond P \to \Diamond P$ |
| Transitiv               | $\square P \to \square \square P$ |
| Symmetrisch             | $P \to \square \Diamond P$ |
| Reflexiv und transitiv  | $\square \square P \leftrightarrow \square P$ |
| Reflexiv und transitiv  | $\Diamond \Diamond P \leftrightarrow \Diamond P$ |
| Äquivalenzrelation      | $\Diamond \square P \leftrightarrow \square P$ |
| Äquivalenzrelation      | $\square \Diamond P \leftrightarrow \Diamond P$ |

Diese Formeln zeigen, wie die Struktur der Kripke-Relation $R$ die Gültigkeit von modalen Formeln beeinflusst.

