### Quantoren gegen Funktionszeichen

#### Darstellung mit Existenzquantoren
Die Existenz eines bestimmten Wertes wird durch Existenzquantoren angezeigt.

1. **$\forall x \exists y (y = x + x)$**:
   - Diese Formel besagt, dass es für jedes $x$ ein $y$ gibt, das gleich $x + x$ ist.

2. **$\forall x \exists y (x < y)$**:
   - Diese Formel besagt, dass es für jedes $x$ ein $y$ gibt, das größer als $x$ ist.

3. **$\forall x \forall y \exists z (x < y \rightarrow x + z = y)$**:
   - Diese Formel besagt, dass es für jedes $x$ und $y$ ein $z$ gibt, so dass, wenn $x < y$, $x + z = y$ ist.

#### Darstellung mit Funktionszeichen
Funktionszeichen ersetzen Existenzquantoren und geben explizit den Wert an, der die Bedingung erfüllt.

1. **$\forall x (do(x) = x + x)$**:
   - Hier wird $do(x)$ als $x + x$ definiert.

2. **$\forall x (x < gr(x))$**:
   - $gr(x)$ ist eine Funktion, die einen Wert liefert, der größer ist als $x$.

3. **$\forall x \forall y (x < y \rightarrow x + diff(x, y) = y)$**:
   - $diff(x, y)$ ist eine Funktion, die die Differenz $y - x$ liefert, wenn $x < y$.

### Interpretationen der Funktionszeichen

1. **$do_{N1}(d) = d + d$**:
   - $do(x)$ ist die Funktion, die das Doppelte von $x$ zurückgibt.

2. **$gr_{N2}(d) = d + 1$**:
   - $gr(x)$ liefert den Nachfolger von $x$, also $x + 1$.

3. **$diff_{N3}(d_1, d_2) = d_2 - d_1$**, falls $d_1 < d_2$, sonst $0$:
   - $diff(x, y)$ gibt $y - x$ zurück, wenn $x < y$. Andernfalls wird $0$ zurückgegeben, wobei dieser Wert willkürlich gewählt wurde.