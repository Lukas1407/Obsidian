## Systematische Analyse des Denkens
Die moderne Logik ist aus dem Bedürfnis entstanden, das menschliche Denken systematisch zu analysieren.
Dabei geht es darum, wie wir Informationen verarbeiten, Schlüsse ziehen und Probleme lösen.
Die Logik untersucht also die Struktur des Denkens und die Regeln, nach denen wir vernünftig argumentieren.
## Fokus auf logisches Schließen
In der Logik konzentriert man sich auf eine spezielle Form des Denkens: das logische Schließen.
Das bedeutet, dass wir aus gegebenen Annahmen (Prämissen) auf Schlussfolgerungen kommen, die logisch zwingend sind.

## Beispiel aus der Alltagslogik: Kann mein Bruder mein Schwager sein?
### Erklärung der Argumentation
1. **Annahme: Bruder als Schwager**:
   - Zuerst stellen wir die Hypothese auf, dass Ihr Bruder Ihr Schwager sein könnte.
   - Diese Annahme werden wir überprüfen.
2. **Definition von „Schwager“**:
   - Laut allgemeinem Sprachgebrauch ist ein Schwager der Bruder des Ehepartners.
   - Es gibt auch die Definition, dass ein Schwager der Ehepartner eines Geschwisters sein kann, aber diese lassen wir in diesem Beispiel außen vor.
3. **Bruder ist Bruder des Ehepartners**:
   - Wenn Ihr Bruder Ihr Schwager ist, müsste er der Bruder Ihres Ehepartners sein.
   - Das würde bedeuten, dass Ihr Ehepartner auch ein Geschwisterteil von Ihnen ist.
4. **Deutsches Eherecht**:
   - In Deutschland ist es gesetzlich verboten, Geschwister zu heiraten.
   - Das ist aus biologischen und sozialen Gründen so festgelegt.
5. **Schlussfolgerung**:
   - Da es nicht erlaubt ist, Geschwister zu heiraten, kann Ihr Bruder nicht Ihr Schwager sein.
   - Die ursprüngliche Annahme, dass Ihr Bruder Ihr Schwager sein könnte, wird dadurch widerlegt.
### Formalisierung
#### 1. Bruno und ich sind Geschwister:
- **Aussage**: $geschwister(Bruno, i)$
- **Bedeutung**: Bruno und „ich“ (i) sind Geschwister.
- Dies ist ein Faktum, das im Kontext gilt. Bruno und „ich“ (i) sind Geschwister.
#### 2. Bruno ist mein Schwager:
- **Aussage**: $schwager(Bruno, i)$
- **Bedeutung**: Bruno ist der Schwager von „mir“ (i).
- Dies ist eine Annahme für die momentane Argumentation. Bruno ist der Schwager von „mir“ (i).
#### 3. Definition von „Schwager“:
- **Aussage**: $∀x(schwager(x, i) → geschwister(x, ehe(i)))$
- **Bedeutung**: Für jede Person $x$ gilt: Wenn $x$ mein Schwager ist, dann sind $x$ und mein Ehepartner $ehe(i)$ Geschwister.
  - Das heißt, wenn $x$ mein Schwager ist, dann ist $x$ der Bruder meines Ehepartners.
- Dies ist eine allgemeine Regel. Für jede Person xxx gilt: Wenn xxx mein Schwager ist, dann sind xxx und mein Ehepartner (ehe(i)) Geschwister.
#### 4. Gesetzliches Verbot:
- **Aussage**: $∀x(¬geschwister(ehe(x), x))$
- **Bedeutung**: Für jede Person $x$ gilt: Es ist nicht erlaubt, dass $x$ und ihr Ehepartner Geschwister sind.
  - Man darf also nicht seinen eigenen Bruder oder seine eigene Schwester heiraten.
- Dies ist ebenfalls eine allgemeine Regel. Für jede Person xxx gilt: xxx und ihr Ehepartner dürfen keine Geschwister sein.
#### 5. Bruno und mein Ehepartner sind Geschwister:
- **Aussage**: $geschwister(ehe(i), Bruno)$
- **Bedeutung**: Mein Ehepartner ($ehe(i)$) und Bruno sind Geschwister.
  - Dies folgt aus der Definition von „Schwager“ (Aussage 3) und der Annahme, dass Bruno mein Schwager ist (Aussage 2).
- Dies ist eine Folgerung aus den Aussagen 2 und 3. Bruno ist der Schwager von „mir“ $i$, daher ist Bruno der Bruder meines Ehepartners ($ehe(i)$). Die Schlussfolgerung ergibt sich aus dem Modus Ponens: Wenn $schwager(Bruno,i)schwager(Bruno, i)schwager(Bruno,i)$ wahr ist und $schwager(x,i)→geschwister(ehe(i),x)schwager(x, i) → geschwister(ehe(i), x)schwager(x,i)→geschwister(ehe(i),x)$ gilt, dann muss auch $geschwister(ehe(i),Bruno)geschwister(ehe(i), Bruno)geschwister(ehe(i),Bruno)$ wahr sein.
#### 6. Mein Ehepartner und ich sind Geschwister:
- **Aussage**: $geschwister(ehe(i), i)$
- **Bedeutung**: Mein Ehepartner und ich $i$ sind Geschwister.
  - Dies folgt aus der Tatsache, dass Bruno mein Bruder ist (Aussage 1) und wir bereits festgestellt haben, dass mein Ehepartner und Bruno Geschwister sind (Aussage 5).
- Dies ist eine weitere Folgerung. Da mein Ehepartner ($ehe(i)$) und ich ($i$) Geschwister sind (aus der Aussage $geschwister(ehe(i),Bruno)geschwister(ehe(i), Bruno)geschwister(ehe(i),Bruno)$ und dem Faktum $geschwister(Bruno,i)geschwister(Bruno, i)geschwister(Bruno,i))$, folgt, dass mein Ehepartner und ich Geschwister sein müssen.
#### 7. Widerspruch:
- **Aussage**: Nach dem deutschen Eherecht (Aussage 4) darf niemand mit seinen Geschwistern verheiratet sein, was aber hier der Fall wäre (Aussage 6).
- Aus der allgemeinen Regel 4 $∀x(¬geschwister(ehe(x),x))∀x(¬geschwister(ehe(x), x))∀x(¬geschwister(ehe(x),x))$ folgt, dass mein Ehepartner und ich nicht Geschwister sein dürfen. Dies steht im Widerspruch zu Aussage 6 $geschwister(ehe(i),i)geschwister(ehe(i), i)geschwister(ehe(i),i)$. Der Widerspruch entsteht, weil die Aussagen 4a $¬geschwister(ehe(i),i)¬geschwister(ehe(i), i)¬geschwister(ehe(i),i)$ und 6 $geschwister(ehe(i),i)geschwister(ehe(i), i)geschwister(ehe(i),i)$ sich gegenseitig ausschließen.
