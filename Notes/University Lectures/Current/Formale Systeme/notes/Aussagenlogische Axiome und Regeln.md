Klar, ich werde jede der auf den Bildern dargestellten Regeln genauer erklären. Wir fangen mit den Regeln für die Aussagenlogik an, gehen dann zu den Regeln für die Prädikatenlogik über und schließen mit den Regeln für Gleichheit ab.

### Aussagenlogische Axiome und Regeln

#### 1. **Axiom**

$$ \frac{}{\Gamma, F \Rightarrow F, \Delta} $$

- **Beschreibung:** Dieses Axiom sagt aus, dass wenn eine Formel$F$ sowohl auf der linken (Antezedent) als auch auf der rechten Seite (Sukzedent) der Sequenz vorkommt, die Sequenz immer wahr ist. 
- **Beispiel:** Angenommen,$F$ ist$P$, dann wäre die Sequenz$\Gamma, P \Rightarrow P, \Delta$ immer erfüllt, weil$P \rightarrow P$ eine Tautologie ist.

#### 2. **Negation Links (not-left)**

$$ \frac{\Gamma, \Rightarrow F, \Delta}{\Gamma, \neg F \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel erlaubt es, eine Negation auf die andere Seite der Sequenz zu bewegen. Wenn$F$ auf der rechten Seite steht, dann kann$\neg F$ auf die linke Seite verschoben werden.
- **Beispiel:** Wenn$F$$Q$ ist, dann transformiert diese Regel die Sequenz$\Gamma \Rightarrow Q, \Delta$ in$\Gamma, \neg Q \Rightarrow \Delta$.

#### 3. **Negation Rechts (not-right)**

$$ \frac{\Gamma, F \Rightarrow \Delta}{\Gamma \Rightarrow \neg F, \Delta} $$

- **Beschreibung:** Diese Regel erlaubt es, eine Negation auf die andere Seite der Sequenz zu bewegen. Wenn$F$ auf der linken Seite steht, dann kann$\neg F$ auf die rechte Seite verschoben werden.
- **Beispiel:** Wenn$F$$R$ ist, dann transformiert diese Regel die Sequenz$\Gamma, R \Rightarrow \Delta$ in$\Gamma \Rightarrow \neg R, \Delta$.

#### 4. **Implikation Links (impl-left)**

$$ \frac{\Gamma \Rightarrow F, \Delta \quad \Gamma, G \Rightarrow \Delta}{\Gamma, F \rightarrow G \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Implikation auf der linken Seite. Um$F \rightarrow G$ zu beweisen, muss man zeigen, dass entweder$F$ falsch oder$G$ wahr ist.
- **Beispiel:** Für$F \rightarrow G$, wenn$F$$P$ ist und$G$$Q$, transformiert diese Regel$\Gamma, P \rightarrow Q \Rightarrow \Delta$ in zwei Sequenzen:$\Gamma \Rightarrow P, \Delta$ und$\Gamma, Q \Rightarrow \Delta$.

#### 5. **Implikation Rechts (impl-right)**

$$ \frac{\Gamma, F \Rightarrow G, \Delta}{\Gamma \Rightarrow F \rightarrow G, \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Implikation auf der rechten Seite. Um$F \rightarrow G$ zu beweisen, muss man zeigen, dass wenn$F$ wahr ist,$G$ ebenfalls wahr ist.
- **Beispiel:** Wenn$F$$P$ ist und$G$$R$, dann transformiert diese Regel$\Gamma \Rightarrow P \rightarrow R, \Delta$ in$\Gamma, P \Rightarrow R, \Delta$.

#### 6. **Und Links (and-left)**

$$ \frac{\Gamma, F, G \Rightarrow \Delta}{\Gamma, F \land G \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Konjunktion (und) auf der linken Seite. Wenn$F \land G$ auf der linken Seite steht, müssen beide Formeln$F$ und$G \ auf der linken Seite stehen.
- **Beispiel:** Für$F \land G$, wenn$F$$P$ ist und$G$$Q$, transformiert diese Regel$\Gamma, P \land Q \Rightarrow \Delta$ in$\Gamma, P, Q \Rightarrow \Delta$.

#### 7. **Und Rechts (and-right)**

$$ \frac{\Gamma \Rightarrow F, \Delta \quad \Gamma \Rightarrow G, \Delta}{\Gamma \Rightarrow F \land G, \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Konjunktion auf der rechten Seite. Um$F \land G$ zu beweisen, muss man zeigen, dass sowohl$F$ als auch$G$ wahr sind.
- **Beispiel:** Wenn$F$$P$ ist und$G$$R$, transformiert diese Regel$\Gamma \Rightarrow P \land R, \Delta$ in zwei Sequenzen:$\Gamma \Rightarrow P, \Delta$ und$\Gamma \Rightarrow R, \Delta$.

#### 8. **Oder Links (or-left)**

$$ \frac{\Gamma, F \Rightarrow \Delta \quad \Gamma, G \Rightarrow \Delta}{\Gamma, F \lor G \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Disjunktion (oder) auf der linken Seite. Um$F \lor G$ zu beweisen, muss man zeigen, dass entweder$F$ oder$G$ wahr ist.
- **Beispiel:** Für$F \lor G$, wenn$F$$Q$ ist und$G$$S$, transformiert diese Regel$\Gamma, Q \lor S \Rightarrow \Delta$ in zwei Sequenzen:$\Gamma, Q \Rightarrow \Delta$ und$\Gamma, S \Rightarrow \Delta$.

#### 9. **Oder Rechts (or-right)**

$$ \frac{\Gamma \Rightarrow F, G, \Delta}{\Gamma \Rightarrow F \lor G, \Delta} $$

- **Beschreibung:** Diese Regel behandelt eine Disjunktion auf der rechten Seite. Um$F \lor G$ zu beweisen, muss man zeigen, dass mindestens eine der Formeln$F$ oder$G$ wahr ist.
- **Beispiel:** Wenn$F$$P$ ist und$G$$Q$, transformiert diese Regel$\Gamma \Rightarrow P \lor Q, \Delta$ in$\Gamma \Rightarrow P, Q, \Delta$.

### Prädikatenlogische Regeln

#### 1. **Allquantor Links (all-left)**

$$ \frac{\Gamma, \forall x F, \{x/X\} F \Rightarrow \Delta}{\Gamma, \forall x F \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel behandelt den Allquantor ($ \forall$) auf der linken Seite. Eine neue Variable$X$ wird eingeführt und ersetzt$x$ in$F$. 
- **Beispiel:** Wenn$F(x)$$P(x)$ ist, transformiert diese Regel$\Gamma, \forall x P(x) \Rightarrow \Delta$ in$\Gamma, P(X) \Rightarrow \Delta$, wobei$X$ eine neue Variable ist.

#### 2. **Allquantor Rechts (all-right)**

$$ \frac{\Gamma \Rightarrow \{x/f(\overline{x})\} F, \Delta}{\Gamma \Rightarrow \forall x F, \Delta} $$

- **Beschreibung:** Diese Regel behandelt den Allquantor auf der rechten Seite. Ein neues Funktionssymbol$f$ wird eingeführt, um alle möglichen Werte von$x$ zu erfassen.
- **Beispiel:** Wenn$F(x)$$Q(x)$ ist, transformiert diese Regel$\Gamma \Rightarrow \forall x Q(x), \Delta$ in$\Gamma \Rightarrow Q(f(\overline{x})), \Delta$, wobei$f$ ein neues Funktionssymbol ist.

#### 3. **Existenzquantor Rechts (ex-right)**

$$ \frac{\Gamma \Rightarrow \exists x F, \{x/X\} F, \Delta}{\Gamma \Rightarrow \exists x F, \Delta} $$

- **Beschreibung:** Diese Regel behandelt den Existenzquantor ($ \exists$) auf der rechten Seite. Eine neue Variable$X$ wird eingeführt, die ein bestimmtes$x$ repräsentiert.
- **Beispiel:** Wenn$F(x)$$R(x)$ ist, transformiert diese Regel$\Gamma \Rightarrow \exists x R(x), \Delta$ in$\Gamma \Rightarrow R(X), \Delta$, wobei$X$ eine neue Variable ist.

#### 4. **Existenzquantor Links (ex-left)**

$$ \frac{\Gamma, \{x/f(\overline{x})\} F \Rightarrow \Delta}{\Gamma, \exists x F \Rightarrow \Delta} $$

- **Beschreibung:** Diese Regel behandelt den Exist

enzquantor auf der linken Seite. Ein neues Funktionssymbol$f$ wird eingeführt, um den Existenzquantor zu eliminieren.
- **Beispiel:** Wenn$F(x)$$S(x)$ ist, transformiert diese Regel$\Gamma, \exists x S(x) \Rightarrow \Delta$ in$\Gamma, S(f(\overline{x})) \Rightarrow \Delta$, wobei$f$ ein neues Funktionssymbol ist.

### Axiome und Regeln für Gleichheit

#### 1. **Identität Rechts (identity-right)**

$$ \frac{}{\Gamma \Rightarrow s \doteq s, \Delta}
$$

- **Beschreibung:** Diese Regel besagt, dass jedes Objekt$s$ mit sich selbst identisch ist. Es ist immer wahr, dass$s \doteq s$.
- **Beispiel:** Für jeden Term$s$, wie$a$, ist$a \doteq a$ immer wahr.

#### 2. **Symmetrie Rechts (symmetry-right)**

$$ \frac{\Gamma \Rightarrow s \doteq t, \Delta}{\Gamma \Rightarrow t \doteq s, \Delta}
$$

- **Beschreibung:** Diese Regel besagt, dass wenn$s$ gleich$t$ ist, dann ist$t$ auch gleich$s$.
- **Beispiel:** Wenn$a \doteq b$ wahr ist, dann muss auch$b \doteq a$ wahr sein.

#### 3. **Symmetrie Links (symmetry-left)**

$$ \frac{\Gamma, s \doteq t \Rightarrow \Delta}{\Gamma, t \doteq s \Rightarrow \Delta}
$$

- **Beschreibung:** Diese Regel besagt, dass wenn$s \doteq t$ auf der linken Seite steht, kann es durch$t \doteq s$ ersetzt werden.
- **Beispiel:** Wenn$a \doteq b$ auf der linken Seite einer Sequenz steht, kann es durch$b \doteq a$ ersetzt werden.

#### 4. **Gleichheits-Substitution Rechts (eq-subst-right)**

$$ \frac{\Gamma, s \doteq t \Rightarrow F(t), \Delta}{\Gamma, s \doteq t \Rightarrow F(s), \Delta}
$$

- **Beschreibung:** Diese Regel erlaubt die Ersetzung eines Terms durch einen gleichwertigen Term auf der rechten Seite. Wenn$s \doteq t$ wahr ist, kann$t$ durch$s$ ersetzt werden.
- **Beispiel:** Wenn$a \doteq b$ wahr ist und$F(t)$ eine Formel wie$P(b)$ ist, kann$P(b)$ durch$P(a)$ ersetzt werden.

#### 5. **Gleichheits-Substitution Links (eq-subst-left)**

$$ \frac{\Gamma, F(t), s \doteq t \Rightarrow \Delta}{\Gamma, F(s), s \doteq t \Rightarrow \Delta}
$$

- **Beschreibung:** Diese Regel erlaubt die Ersetzung eines Terms durch einen gleichwertigen Term auf der linken Seite. Wenn$s \doteq t$ wahr ist, kann$t$ durch$s$ ersetzt werden.
- **Beispiel:** Wenn$a \doteq b$ wahr ist und$F(t)$ eine Formel wie$Q(b)$ ist, kann$Q(b)$ durch$Q(a)$ ersetzt werden.


