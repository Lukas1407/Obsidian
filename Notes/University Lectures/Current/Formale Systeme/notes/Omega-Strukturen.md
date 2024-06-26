### Omega-Strukturen

#### Definition

Eine Omega-Struktur $R$ für eine aussagenlogische Signatur $P$ besteht aus folgenden Komponenten:

1. **Geordnete Menge der natürlichen Zahlen $(N, \leq)$**:
   - Diese Menge wird interpretiert als die Menge abstrakter Zeitpunkte.

2. **Funktion $\xi : N \rightarrow 2^P$**:
   - Diese Funktion ordnet jedem Zeitpunkt $n \in N$ eine Teilmenge von $P$ zu.
   - Die Intention dieser Funktion ist, dass $p \in \xi(n)$ genau dann gilt, wenn $p$ zum Zeitpunkt $n$ in $R$ wahr ist.

#### Notation

- **$\xi_n$** steht für das bei $n$ beginnende Endstück von $\xi$:
  $$
  \xi_n(m) = \xi(n + m)
  $$
  Das bedeutet, dass $\xi_n$ die Funktion $\xi$ ist, verschoben um $n$ Zeitpunkte. Insbesondere ist $\xi_0 = \xi$.

#### Erläuterung der Komponenten

1. **Geordnete Menge $(N, \leq)$**:
   - $N$ repräsentiert die natürlichen Zahlen.
   - $\leq$ ist die übliche Ordnungsrelation auf den natürlichen Zahlen.
   - Diese geordnete Menge wird als eine Abfolge von Zeitpunkten interpretiert, wobei jeder Zeitpunkt ein natürlicher Zahl ist.

2. **Funktion $\xi$**:
   - Die Funktion $\xi$ weist jedem Zeitpunkt eine Menge von wahren Aussagen zu.
   - $\xi(n)$ ist eine Teilmenge von $P$, die alle Aussagen enthält, die zum Zeitpunkt $n$ wahr sind.

#### Beispiel zur Veranschaulichung

Betrachten wir ein einfaches Beispiel:

- Nehmen wir $P = \{p, q\}$ als die Signatur, wobei $p$ und $q$ Aussagen sind.
- Die Omega-Struktur $R$ könnte folgendermaßen definiert sein:
  $$
  \xi(n) = 
  \begin{cases} 
  \{p\} & \text{wenn } n \text{ gerade ist} \\
  \{q\} & \text{wenn } n \text{ ungerade ist}
  \end{cases}
  $$

Hier bedeutet $\xi(n)$:
- Zum Zeitpunkt $n = 0$ ist $p$ wahr.
- Zum Zeitpunkt $n = 1$ ist $q$ wahr.
- Zum Zeitpunkt $n = 2$ ist $p$ wieder wahr, und so weiter.

Das Endstück $\xi_2$ wäre dann:
- $\xi_2(0) = \xi(2) = \{p\}$
- $\xi_2(1) = \xi(3) = \{q\}$
- $\xi_2(2) = \xi(4) = \{p\}$, und so weiter.

#### Zusammenfassung

Eine Omega-Struktur ist ein Modell für die zeitliche Entwicklung von Wahrheitswerten von Aussagen. Sie besteht aus:
- Einer geordneten Menge natürlicher Zahlen als Zeitpunkte.
- Einer Funktion, die jedem Zeitpunkt eine Menge von wahren Aussagen zuordnet.

Die Omega-Struktur ermöglicht es, zeitabhängige Wahrheitswerte in formalen Systemen zu modellieren und zu analysieren.

