### 1. **Probleme des klassischen Whitted-Style Raytracing-Verfahrens**
   - Das Whitted-Style Raytracing erzeugt oft „perfekte“ Bilder, die zu künstlich wirken. Die Effekte umfassen perfekte Spiegelung, scharfe Schattenkanten und unendliche Schärfentiefe.
   - **Lösung:** Um realistischere Effekte zu erzielen, werden Schattenstrahlen zu zufällig gewählten Punkten innerhalb der Lichtquelle geschossen. Dadurch entsteht ein weicher Schatten, der realistischer aussieht.
![[Pasted image 20241113121820.png|500]]
### 2. **Weiche Schatten**
   - Da reale Lichtquellen eine endliche Ausdehnung besitzen, werden durch Distributed Raytracing weiche Schatten erzeugt.
   - Der Schatten wird mit zunehmendem Abstand weicher, nahe am Objekt bleibt der Schatten jedoch scharf.

### 3. **Bewegungsunschärfe (Motion Blur)**
   - Bewegungsunschärfe wird durch Verteilung der Strahlen in der Zeit simuliert. Dies entspricht der Belichtungszeit bei realen Kameras.
   - Objekte, die sich während eines bestimmten Zeitintervalls bewegen, werden unscharf abgebildet.
   - Für jeden Pixel werden mehrere Strahlen in unterschiedlichen Zeitpunkten ausgesandt, um die Bewegung des Objekts zu erfassen und die Farbwerte entsprechend zu mitteln.

### 4. **Tiefenschärfe**
   - Durch das Modell der „dünnen Linse“ werden zusätzliche Strahlen durch die Linsenfläche projiziert. Für jeden Punkt auf der Bildebene wird ein Punkt auf der Linse gewählt, und Strahlen werden durch das Zentrum der Linse zum Fokuspunkt geführt.
   - Der Effekt sorgt dafür, dass Objekte außerhalb des Fokusbereichs unscharf erscheinen, was eine realistische Tiefenschärfe erzeugt.

### 5. **Code-Beispiel für Bewegungs- und Tiefenunschärfe**
   - Der Pseudocode zeigt, wie Bewegungsunschärfe und Tiefenschärfe durch mehrere Proben für jeden Pixel erzielt werden.
   - `nSuperSample` bestimmt die Anzahl der Strahlen pro Pixel (Supersampling), `nLensSample` ist die Anzahl der Linsenproben für Tiefenschärfe und `nTimeSamples` die Probenanzahl für die Bewegungsunschärfe.
   - Zufallspunkte werden in jedem Durchlauf generiert, um unterschiedliche Strahlenrichtungen und Zeiten zu simulieren.

### **Zusammenfassung: Distributed Raytracing**
   - Distributed Raytracing erweitert das klassische Raytracing um realistische Effekte durch die Verwendung mehrerer Strahlen pro Pixel.
   - Bewegungsunschärfe und Tiefenschärfe werden durch eine Kombination von zeit- und raumbasiertem Supersampling erzeugt, was dem Bild eine realistischere Darstellung verleiht.