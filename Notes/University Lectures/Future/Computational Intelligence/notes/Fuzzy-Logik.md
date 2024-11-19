> [!summary] Definition
>  Fuzzy-Logik ist eine Form der Logik, die es ermöglicht, mit Unschärfen zu arbeiten. Anstatt nur die Werte “wahr” oder “falsch” zu verwenden, wie es in der klassischen Logik der Fall ist, können Werte zwischen 0 und 1 annehmen, um verschiedene Grade der Wahrheit darzustellen
## Erweiterung der Mengenlehre
- Zulassen von Zugehörigkeitswerten zwischen Null und Eins
- -> [[Fuzzifizierung]]: Dies ist der Prozess, bei dem klare, präzise Eingabewerte in unscharfe Mengen umgewandelt werden, die durch Zugehörigkeitsgrade zwischen 0 und 1 gekennzeichnet sind
## Erweiterung der Aussagenlogik
- Verallgemeinerung der Operatoren (Konjunktion, Disjunktion, Implikation, Negation) für Wahrheitswerte zwischen Null und Eins
- -> [[Inferenz bei Fuzzy-Systemen|Inferenz]]: In diesem Schritt werden die unscharfen Eingabewerte mithilfe von Fuzzy-Regeln verarbeitet, um zu Schlussfolgerungen zu gelangen
## Defuzzifizierung
- Nachdem die Inferenz abgeschlossen ist, wird der unscharfe Ausgabewert in einen klaren, präzisen Wert umgewandelt, der als Entscheidung oder Aktion verwendet werden kann
## Operatoren in klassischer Logik vs Fuzzy-Logik
In der klassischen Logik sind die Operatoren streng definiert und erlauben nur absolute Wahrheitswerte. In der Fuzzy-Logik hingegen sind die Operatoren so gestaltet, dass sie mit einem Kontinuum von Wahrheitswerten arbeiten können, was eine flexiblere und realitätsnähere Modellierung von Situationen ermöglicht.
### Klassische Logik
- **Wahrheitswerte**: Nur 0 (falsch) oder 1 (wahr).
- **UND (Konjunktion)**: Das Ergebnis ist 1, wenn beide Aussagen wahr sind (A und B sind beide 1).
- **ODER (Disjunktion)**: Das Ergebnis ist 1, wenn mindestens eine der Aussagen wahr ist.
- **Implikation**: Das Ergebnis ist falsch nur dann, wenn aus einer wahren Voraussetzung eine falsche Schlussfolgerung folgt.
- **Negation**: Das Ergebnis ist das Gegenteil des Wahrheitswertes der Aussage (aus 1 wird 0 und umgekehrt).
### Fuzzy-Logik
- **Wahrheitswerte**: Zwischen 0 und 1, um verschiedene Grade der Wahrheit darzustellen.
- **UND (Konjunktion)**: Verwendet das Minimum oder das Produkt der Zugehörigkeitswerte von A und B.
    - Minimum:$$
        min(A,B)$$
        
    - Produkt:$$
        A×B$$
- **ODER (Disjunktion)**: Verwendet das Maximum oder die beschränkte Summe der Zugehörigkeitswerte von A und B.
    - Maximum:$$
        max(A,B)$$
    - Beschränkte Summe:$$
        
        min(A+B,1)$$
- **Implikation**: Der Wert der Schlussfolgerung B ist gleich dem Wert der Voraussetzung A.
- **Negation**: Der Zugehörigkeitswert wird subtrahiert von 1.
    - Negation:$$
        NEG(A)=1−A$$