### Vollständige endliche Automaten

#### Definition

Ein vollständiger endlicher Automat ist ein Automat, bei dem die Übergangsfunktion $\delta$ für jedes Paar $(s, a)$ definiert ist, wobei $s$ ein Zustand aus der Zustandsmenge $S$ und $a$ ein Symbol aus dem Alphabet $V$ ist. Das bedeutet, dass für jede Kombination aus Zustand und Eingabesymbol ein Übergang existiert.

#### Warum Vollständigkeit wichtig ist

Wenn während der Verarbeitung eines Wortes $w$ eine Situation $(s, a)$ erreicht wird, für die $\delta(s, a)$ nicht definiert ist, gilt das Wort $w$ als nicht akzeptiert. Um dies zu vermeiden, stellt ein vollständiger endlicher Automat sicher, dass für jede Kombination ein Übergang definiert ist.

### Beispielautomat $N_{bba}$
![[Pasted image 20240626105802.png#invert|400]]
#### Komponenten des Automaten

- **Zustandsmenge $S$**: $\{s_0, s_1, s_2, s_3\}$
- **Alphabet $V$**: $\{a, b, c\}$
- **Endzustände $S_1$**: $\{s_0, s_1, s_2\}$
- **Übergangsfunktion $\delta$**: Definiert durch die Pfeile im Zustandsdiagramm

#### Zustandsdiagramm

Das Bild zeigt den Beispielautomaten $N_{bba}$. Die Pfeile repräsentieren die Übergangsfunktion $\delta$:
- Vom Zustand $s_0$:
  - Bei Eingabe $a$ oder $c$ bleibt der Automat in $s_0$.
  - Bei Eingabe $b$ wechselt der Automat zu $s_1$.
- Vom Zustand $s_1$:
  - Bei Eingabe $a$ oder $c$ wechselt der Automat zurück zu $s_0$.
  - Bei Eingabe $b$ wechselt der Automat zu $s_2$.
- Vom Zustand $s_2$:
  - Bei Eingabe $a$ wechselt der Automat zu $s_3$.
  - Bei Eingabe $b$ bleibt der Automat in $s_2$.
  - Bei Eingabe $c$ wechselt der Automat zu $s_0$.
- Vom Zustand $s_3$:
  - Bei jeder Eingabe ($a$, $b$, oder $c$) bleibt der Automat in $s_3$.

### Akzeptierte Sprache $L(N_{bba})$

Der Automat $N_{bba}$ akzeptiert alle Wörter $w$, die kein Teilwort "bba" enthalten. Das bedeutet, sobald das Teilwort "bba" in einem Wort erscheint, wird der Automat in den Zustand $s_3$ wechseln, der kein Endzustand ist. Wörter, die "bba" nicht enthalten, können in den Endzuständen $s_0, s_1$ oder $s_2$ enden, und werden daher akzeptiert.

### Vollständigkeit des Automaten $N_{bba}$

Der Automat $N_{bba}$ ist vollständig, weil für jedes Paar $(s, a)$ ein Übergang definiert ist:
- Für jeden Zustand $s$ ($s_0, s_1, s_2, s_3$) und jedes Symbol $a$ ($a, b, c$) existiert ein definierter Übergang gemäß der Übergangsfunktion $\delta$.

Zusammenfassend:

- **Vollständiger Automat**: Jeder Zustand hat für jedes Eingabesymbol einen definierten Übergang.
- **Nicht vollständiger Automat**: Einige Paare $(s, a)$ haben keine definierten Übergänge.
- **Akzeptierte Sprache**: Der Automat $N_{bba}$ akzeptiert alle Wörter, die das Teilwort "bba" nicht enthalten.
- **Beispielautomat $N_{bba}$**: Zeigt, wie die Übergangsfunktion und die Zustände verwendet werden, um die akzeptierte Sprache zu definieren. 

Wenn du noch weitere Fragen hast oder etwas genauer erklärt haben möchtest, lass es mich wissen!

