Gerne! Hier ist eine detaillierte Erklärung der verschiedenen **Arten von Lichtquellen**, die auf der Folie dargestellt sind.

### 1. Punktlichtquelle
- Eine **Punktlichtquelle** ist eine Lichtquelle, die Licht in alle Richtungen gleichmäßig abstrahlt, als käme es von einem einzigen Punkt.
- Sie wird definiert durch:
  - **Position `p`**: Den Ort, an dem sich die Lichtquelle befindet.
  - **Intensität `I_L`**: Die Lichtstärke in Watt pro Steradiant \((\text{W/sr})\), was die Menge an Licht beschreibt, die pro Raumwinkel abgestrahlt wird.
- **Abstandsgesetz**: Die Intensität einer Punktlichtquelle nimmt mit dem **Abstand zum Quadrat** ab. Das bedeutet, dass ein Objekt doppelt so weit entfernt von der Lichtquelle nur ein Viertel der Intensität erhält. Dieser Effekt simuliert realistische Lichtabnahme über größere Distanzen.
![[Pasted image 20241113092459.png#invert|200]]
### 2. Paralleles Licht / Direktionale Lichtquelle
- Eine **parallele oder gerichtete Lichtquelle** strahlt Lichtstrahlen in einer festen Richtung und mit parallelen Strahlen ab.
- Sie wird definiert durch:
  - **Richtung `d`**: Die Richtung, in die das Licht strahlt.
  - **Flussdichte `E`**: Gemessen in Watt pro Quadratmeter \((\text{W/m}^2)\), beschreibt die Energiemenge, die pro Flächeneinheit in einer bestimmten Richtung strahlt.
- **Beispiel**: Sonnenlicht kann als annähernd parallel angesehen werden, da die Sonne so weit entfernt ist, dass ihre Strahlen auf der Erde nahezu parallel erscheinen. Diese Art der Beleuchtung erzeugt scharfe, parallele Schatten.
![[Pasted image 20241113092511.png#invert|200]]
### 3. Weitere Lichtquellentypen
Es gibt auch weitere Arten von Lichtquellen mit spezifischen Eigenschaften:

- **Strahler (Spot-Lights)**:
  - Ein Spot-Light strahlt Licht in einem **Lichtkegel** und fokussiert es in eine bestimmte Richtung. Dies ist nützlich, um bestimmte Bereiche hervorzuheben, ähnlich wie bei einer Taschenlampe oder einem Bühnenstrahler.
  - Die **Abstrahlungscharakteristik** eines Spot-Lights wird oft durch den Ausdruck \( \cos^n(\theta) \) beschrieben, wobei:
    - \( \theta \) der Winkel relativ zur Achse des Lichtkegels ist.
    - Der Exponent \( n \) steuert die Breite des Lichtkegels: Ein höherer Wert von \( n \) bedeutet einen engeren Lichtkegel und konzentrierteres Licht.

- **Punktlichtquellen mit Richtungscharakteristik**:
  - Im Gegensatz zur normalen Punktlichtquelle, die gleichmäßig in alle Richtungen strahlt, kann diese Variante bevorzugt in bestimmten Richtungen leuchten. So lässt sich das Licht gezielt steuern.

- **Flächenlichtquellen**:
  - Eine Flächenlichtquelle, wie zum Beispiel eine Leuchtstoffröhre oder ein Fenster, strahlt Licht von einer definierten Fläche ab und nicht nur von einem Punkt. Dadurch erzeugt sie weiche Schatten und gleichmäßige Beleuchtung.
  - Lichtstrahlen breiten sich von verschiedenen Punkten auf der Fläche aus, was eine sanftere und gleichmäßigere Ausleuchtung zur Folge hat. Flächenlichtquellen sind daher realistischer als Punktlichtquellen, erfordern jedoch mehr Berechnung, da die Lichtverteilung komplexer ist.

### Zusammenfassung
Jede Lichtquelle hat spezifische Eigenschaften und Anwendungen:
- **Punktlichtquellen** eignen sich für Lichtquellen, die gleichmäßig in alle Richtungen abstrahlen, wie Glühbirnen.
- **Direktionale Lichtquellen** simulieren Licht, das von weit entfernten Objekten wie der Sonne kommt.
- **Spot-Lights** und **Flächenlichtquellen** bieten mehr Kontrolle über die Lichtverteilung und werden für gezielte Beleuchtung und weichere Schatten verwendet. 

Diese unterschiedlichen Lichtquellen ermöglichen eine realistische Beleuchtung in der Computergrafik, indem sie verschiedene natürliche und künstliche Lichtsituationen nachbilden.