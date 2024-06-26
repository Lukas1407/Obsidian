
### Erklärung der wichtigsten SMT-LIB-Konstrukte

- **define-fun**: Definiert eine Funktion oder Konstante.
  ```smt-lib
  (define-fun N () Int 5)
  ```

- **declare-fun**: Deklariert eine Funktion, die als Variable verwendet wird.
  ```smt-lib
  (declare-fun v (Int Int) Bool)
  ```

- **forall**: Universeller Quantor, der für alle Elemente im angegebenen Bereich gilt.
  ```smt-lib
  (forall ((col Int))
    ...)
  ```

- **exists**: Existenzieller Quantor, der für mindestens ein Element im angegebenen Bereich gilt.
  ```smt-lib
  (exists ((row Int))
    ...)
  ```

- **assert**: Drückt eine Bedingung aus, die erfüllt sein muss.
  ```smt-lib
  (assert
    ...)
  ```

- **check-sat**: Überprüft die Erfüllbarkeit der angegebenen Bedingungen.

### Schritte zur Formulierung des n-Damen-Problems in SMT-LIB

#### 1. Signatur

Wir definieren die Signatur der Variablen und Funktionen, die wir verwenden werden.

- **Konstante $N$**: Die Größe des Schachbretts.
- **Schachbrett-Funktion $v$**: Eine Funktion, die angibt, ob eine Dame an einer bestimmten Position steht.

```smt-lib
(define-fun N () Int 5) ; N ist die Größe des Schachbretts, z.B. 5
(declare-fun v (Int Int) Bool) ; v gibt an, ob eine Dame an (row, col) steht
```

#### 2. Formalisierung der Bedingungen

**Jede Spalte muss mindestens eine Dame haben:**

Für jede Spalte $c$ gibt es mindestens eine Zeile $r$, in der eine Dame steht.

```smt-lib
(assert
  (forall ((col Int))
    (=> (and (>= col 1) (<= col N))
        (exists ((row Int))
          (and (>= row 1) (<= row N)
               (v row col))))))
```

**Eine Dame pro Reihe, Spalte und Diagonale:**

1. **Pro Reihe nur eine Dame**:

Für jede Reihe $r$ und jede Spalte $c$, wenn eine Dame in $v(r, c)$ steht, darf keine weitere Dame in derselben Spalte $c$ oder in derselben Reihe $r$ stehen.

```smt-lib
(assert
  (forall ((r Int) (c Int))
    (=> (v r c)
        (and
          ;; Nur eine Dame pro Reihe
          (forall ((s Int))
            (=> (distinct r s) (not (v s c))))

          ;; Nur eine Dame pro Spalte
          (forall ((d Int))
            (=> (distinct c d) (not (v r d))))))))
```

2. **Keine Dame auf derselben Diagonale**:

Für jede Position $(r, c)$ darf keine andere Dame auf derselben Diagonale stehen.

```smt-lib
(assert
  (forall ((r Int) (c Int))
    (=> (v r c)
        (and
          ;; Diagonale von links oben nach rechts unten
          (forall ((s Int) (d Int))
            (=> (and (distinct r s) (distinct c d)
                     (= (- r s) (- c d)))
                (not (v s d))))

          ;; Diagonale von rechts oben nach links unten
          (forall ((s Int) (d Int))
            (=> (and (distinct r s) (distinct c d)
                     (= (+ r s) (+ c d)))
                (not (v s d))))))))
```

### Schritt 3: Erfüllbarkeit überprüfen

Nun überprüfen wir die Erfüllbarkeit der Bedingungen und erhalten das Modell:

```smt-lib
(check-sat)
(get-model)
```

### Schritt 4: Ausführen des Programms

Um das Programm auszuführen, speichern wir den SMT-LIB-Code in einer Datei, z.B. `queens.smt2`, und führen dann das SMT-Solver-Tool (wie Z3) aus.

```shell
$ z3 queens.smt2
```

Falls die Formel erfüllbar ist, erhalten wir eine Ausgabe wie folgt:

```smt-lib
sat
(model
  (define-fun v ((x!0 Int) (x!1 Int)) Bool
    (ite (and (= x!0 1) (= x!1 1)) false
    (ite (and (= x!0 2) (= x!1 1)) false
    ...
    (ite (and (= x!0 6) (= x!1 1)) true
    ...
    (ite (and (= x!0 8) (= x!1 8)) false
    false)))))))))
```

Um die spezifischen Werte der Damenpositionen zu erhalten:

```smt-lib
(get-value ((v 1 1) (v 2 1) (v 3 1) (v 4 1)
            (v 5 1) (v 6 1) (v 7 1) (v 8 1)
            (v 9 1) (v 10 1)))
```

Die Ausgabe könnte wie folgt aussehen:

```smt-lib
((v 1 1) false)
((v 2 1) false)
((v 3 1) false)
((v 4 1) false)
((v 5 1) false)
((v 6 1) true)
((v 7 1) false)
((v 8 1) false)
```

### Kompletter SMT-LIB-Code

Hier ist der vollständige SMT-LIB-Code für das n-Damen-Problem für ein 8x8-Schachbrett:

```smt-lib
(define-fun N () Int 8)

(declare-fun v (Int Int) Bool)

(assert
  (forall ((col Int))
    (=> (and (>= col 1) (<= col N))
        (exists ((row Int))
          (and (>= row 1) (<= row N)
               (v row col))))))

(assert
  (forall ((r Int) (c Int))
    (=> (v r c)
        (and
          (forall ((s Int))
            (=> (distinct r s) (not (v s c))))
          (forall ((d Int))
            (=> (distinct c d) (not (v r d))))
          (forall ((s Int) (d Int))
            (=> (and (distinct r s) (distinct c d)
                     (= (- r s) (- c d)))
                (not (v s d))))
          (forall ((s Int) (d Int))
            (=> (and (distinct r s) (distinct c d)
                     (= (+ r s) (+ c d)))
                (not (v s d))))))))

(check-sat)
(get-model)
```

Dieser Code kann direkt in einen SMT-Solver wie Z3 eingegeben werden, um die Lösung für das n-Damen-Problem zu finden.

Falls du weitere Fragen hast oder mehr Details benötigst, lass es mich wissen!
