### DPLL(T): Erweiterung auf beliebige QF-Formeln

Das DPLL(T)-Verfahren ist eine Erweiterung des Nelson-Oppen-Verfahrens, die es ermöglicht, die Erfüllbarkeit von beliebigen quantorenfreien Formeln zu prüfen. 

#### Idee des DPLL(T)-Verfahrens

1. **DPLL-Basisschritt**: Suche nach Konjunktionen von T-Literalen, die das boolesche Skelett der Formel erfüllen.
   - Das bedeutet, dass die formale Struktur der Formel untersucht wird, um potenziell erfüllbare Konjunktionen von Literalen zu finden.

2. **Erfüllbarkeitsprüfung**:
   - **Falls keine erfüllbare Konjunktion gefunden wird**: Die Formel ist unerfüllbar, und der Prozess endet.
   - **Falls eine Konjunktion gefunden wird**: Prüfe, ob diese Konjunktion in der Theorie $T$ erfüllbar ist.

3. **Prüfung der Erfüllbarkeit**:
   - **Falls die Konjunktion erfüllbar ist**: Die Formel ist erfüllbar, und der Prozess endet.
   - **Falls die Konjunktion nicht erfüllbar ist**: Diese Konjunktion wird ausgeschlossen, und der Prozess wird erneut gestartet (GOTO 1).

#### Schritt-für-Schritt Beispiel

1. **Boolesches Skelett analysieren**:
   - Betrachte die Formel als eine Menge von Booleschen Variablen und prüfe, welche Kombinationen dieser Variablen die Formel erfüllen könnten.

2. **Konjunktionen finden**:
   - Suche nach möglichen Kombinationen von Literalen, die zusammen das boolesche Skelett erfüllen.

3. **T-Erfüllbarkeit prüfen**:
   - Prüfe, ob diese Kombinationen von Literalen in der jeweiligen Theorie $T$ erfüllbar sind.

4. **Ergebnisse auswerten**:
   - Erfüllbare Kombinationen bestätigen die Erfüllbarkeit der Formel.
   - Nicht erfüllbare Kombinationen werden ausgeschlossen, und der Prozess wird wiederholt.
