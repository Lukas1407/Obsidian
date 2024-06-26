- [[Gödel's Vollständigkeitssatz (1931)]]
## Konsequenzen der Korrektheit und Vollständigkeit
### Kompaktheitssatz
- **Theorem**:
  - Für beliebige $M \subseteq \text{For}_\Sigma$ und $A \in \text{For}_\Sigma$ gilt:$$
    M \models A \iff E \models A \text{ für eine endliche Teilmenge } E \subseteq M
    $$
- **Bedeutung**:
  - Eine Formel $A$ ist genau dann in allen Modellen von $M$ wahr, wenn es eine endliche Teilmenge $E$ von $M$ gibt, in deren Modellen $A$ ebenfalls wahr ist.
  - Dieser Satz zeigt, dass, um zu überprüfen, ob eine Formel $A$ aus einer (möglicherweise unendlichen) Menge von Formeln $M$ folgt, es genügt, eine endliche Teilmenge $E$ von $M$ zu betrachten.
### Endlichkeitssatz
- **Theorem**:
  - Eine Menge $M \subseteq \text{For}_\Sigma$ hat genau dann ein Modell, wenn jede endliche Teilmenge von $M$ ein Modell hat.
- **Bedeutung**:
  - Eine (möglicherweise unendliche) Menge von Formeln ist genau dann erfüllbar, wenn jede endliche Teilmenge dieser Formeln erfüllbar ist.
- **Zusammenhang mit dem Kompaktheitssatz**:
  - Der Endlichkeitssatz ist ein Spezialfall des Kompaktheitssatzes, wobei man $A$ als die leere Formel (also keine spezielle Formel) betrachtet.
### Beweis des Kompaktheitssatzes
Der Beweis des Kompaktheitssatzes basiert auf der Korrektheit und Vollständigkeit der Prädikatenlogik erster Stufe und der Eigenschaft, dass Ableitungen endlich sind.
#### Schritte des Beweises
1. **Semantische Folgerung**:
    $$
    M \models A
    $$
    - $A$ ist in allen Modellen von $M$ wahr.
2. **Vollständigkeit**:
    $$
    M \models A \implies M \vdash A
    $$
    - Wenn $A$ in allen Modellen von $M$ wahr ist, kann $A$ aus $M$ abgeleitet werden.
3. **Endliche Ableitungen**:
    $$
    M \vdash A \implies E \vdash A \text{ für ein endliches } E \subseteq M
    $$
    - Jede Ableitung ist endlich, also kann $A$ aus einer endlichen Teilmenge $E \subseteq M$ abgeleitet werden.
4. **Korrektheit**:
    $$
    E \vdash A \implies E \models A
    $$
    - Wenn $A$ aus $E$ abgeleitet werden kann, ist $A$ in allen Modellen von $E$ wahr.
5. **Endliche Teilmenge**:
    $$
    E \models A \text{ für ein endliches } E \subseteq M
    $$
    - $A$ ist in allen Modellen von $E$, einer endlichen Teilmenge von $M$, wahr.
#### Zusammengefasst
- $M \models A \iff E \models A \text{ für ein endliches } E \subseteq M$
Das bedeutet, dass wir, um zu überprüfen, ob $A$ aus $M$ folgt, nur eine endliche Teilmenge von $M$ betrachten müssen.

### 