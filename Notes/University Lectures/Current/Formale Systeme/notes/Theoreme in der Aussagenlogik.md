- Diese Theoreme liefern wichtige Verbindungen zwischen den Konzepten der Erfüllbarkeit, Allgemeingültigkeit, logischen Folgerung und logischer Äquivalenz.
## Erfüllbarkeit und Allgemeingültigkeit
- **Theorem:** $A$ ist erfüllbar genau dann, wenn $\neg A$ nicht allgemeingültig ist.
- **Formel:** $A$ erfüllbar $A\leftrightarrow \neg A$ nicht allgemeingültig.
## Allgemeingültigkeit und Logische Folgerung
- **Theorem:** $\models A$ genau dann, wenn $A$ allgemeingültig ist.
- **Formel:** $\models A \leftrightarrow A$ ist allgemeingültig.
- **Bedeutung:** $A$ ist allgemeingültig, wenn $A$ unter jeder Interpretation wahr ist. Das bedeutet auch, dass $A$ aus der leeren Menge folgt, also immer wahr ist.
## Unerfüllbarkeit und logische Folgerung der Negation
- **Theorem:** $\models \neg A$ genau dann, wenn $A$ unerfüllbar ist.
- **Formel:** $\models \neg A \leftrightarrow A$ ist unerfüllbar.
- **Bedeutung:** Eine Aussage $A$ ist unerfüllbar, wenn keine Interpretation $A$ wahr macht. Das bedeutet, dass $\neg A$ in jeder Interpretation wahr ist, also allgemeingültig ist.
## Logische Folgerung und Implikation
- **Theorem:** $A⊨B$ genau dann, wenn $\models A \rightarrow B$
- **Formel:** $A \models B \leftrightarrow \models A\rightarrow B$
- **Bedeutung:** $B$ folgt aus $A$, wenn die Implikation $A \rightarrow B$ unter allen Interpretationen wahr ist, also allgemeingültig ist.
## Erweiterte logische Folgerung
- **Theorem:** $M \cup \{A\} \models B$ genau dann, wenn$M \models A \rightarrow B$
- **Formel:** $M \cup \{A\} \models B \leftrightarrow M \models A \rightarrow B$
- **Bedeutung:** $B$ folgt aus der Menge $M$ und der Aussage $A$, wenn die Implikation $A \rightarrow B$ unter jeder Interpretation, die $M$ erfüllt, wahr ist.
## Logische Äquivalenz
- **Theorem:** $A$ und $B$ sind logisch äquivalent genau dann, wenn $A \leftrightarrow B$ allgemeingültig ist.
- **Formel:** $A \equiv B \leftrightarrow A \leftrightarrow B$ ist allgemeingültig.
- **Bedeutung:** Zwei Aussagen AAA und BBB sind logisch äquivalent, wenn ihre Äquivalenz A↔BA \leftrightarrow BA↔B unter allen Interpretationen wahr ist.
## Ersetzung logischer Äquivalenzen
- **Theorem:** Wenn $A$ und $B$ logisch äquivalent sind und $A$ eine Unterformel von $C$ ist, dann ist die Ersetzung von $AAA$ durch $B$ in $C$ logisch äquivalent zu $C$.
- **Formel:** Wenn $A \equiv B$, $A$ eine Unterformel von $C$ ist und $C'$ aus $C$ durch Ersetzung von $A$ durch $B$ entsteht, dann sind $C$ und $C'$ logisch äquivalent.
- **Bedeutung:** Wenn $A$ und $B$ logisch äquivalent sind, können sie in jeder Formel gegeneinander ausgetauscht werden, ohne die logische Äquivalenz der Formel zu ändern.