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
### Grundidee von Distributed Raytracing
Um Licht realistischer zu modellieren, wird bei jedem Schnittpunkt eines Lichtstrahls mit einer Oberfläche nicht nur ein einziger, sondern mehrere sekundäre Lichtstrahlen (Sekundärstrahlen) in zufällige Richtungen weiterverfolgt. Dies berücksichtigt Effekte wie:
![[Pasted image 20241126094046.png#invert|200]]
1. **Reflexion**: Licht, das von einer Oberfläche reflektiert wird.
2. **Transmission**: Licht, das durch eine Oberfläche hindurchgeht (z. B. bei Glas oder Wasser).
Die Richtung und Stärke dieser Strahlen werden durch die **BRDF** (Bidirectional Reflectance Distribution Function) und **BTDF** (Bidirectional Transmission Distribution Function) bestimmt. Diese Funktionen modellieren, wie Licht abhängig von der Oberfläche gestreut oder gebrochen wird.
### Eigenschaften von Distributed Raytracing
- **Sekundärstrahlen**: Es werden viele Strahlen benötigt, um alle Lichtpfade zu simulieren. Dies kann zu einem exponentiellen Anstieg der Berechnungen führen, da sich der "Strahlenbaum" durch die Rekursion verzweigt.
- **Realismus**: Durch die zufälligen Richtungen der Strahlen werden Effekte wie unscharfe Reflexionen, Tiefenschärfe und Bewegungsunschärfe realistisch dargestellt.
- **Physikalische Basis**: Das Verfahren basiert auf physikalischen Prinzipien des Lichttransports und der Radiometrie.
### Zusammenhang mit Radiometrie
Radiometrie beschreibt, wie Licht in einem physikalischen Raum verteilt ist. Wichtige Konzepte hierbei sind:
- **Flussdichte (Irradiance)**: Die Menge an Lichtleistung, die eine Fläche erreicht.
- **Strahldichte (Radiance)**: Die Lichtleistung in eine bestimmte Richtung pro Fläche und Raumwinkel.
Die Integration über alle einfallenden Strahlen ermöglicht die Berechnung der gesamten Beleuchtung eines Punktes.
### Herausforderungen
- **Rechenaufwand**: Distributed Raytracing erfordert erheblich mehr Rechenleistung als einfachere Raytracing-Methoden wie Whitted-Style Raytracing.
- **Effizienz**: Optimierungen wie adaptive Sampling oder stochastische Ansätze können den Rechenaufwand reduzieren.
### **Reflexion an einer Oberfläche**
Die BRDF beschreibt, wie einfallendes Licht $\omega_i$ (aus einer bestimmten Richtung) von einer Oberfläche reflektiert wird und in eine andere Richtung $\omega_r$ weitergeleitet wird. Die mathematische Definition lautet:

$$f_r(\omega_i, x, \omega_r) = \frac{dL_r(x, \omega_r)}{L_i(x, \omega_i) \cos\theta_i d\omega_i}$$

- **$L_r(x, \omega_r)$**: Die reflektierte Strahldichte in Richtung $\omega_r$.
- **$L_i(x, \omega_i)$**: Die einfallende Strahldichte in Richtung $\omega_i$.
- **$\cos \theta_i$**: Der Winkel zwischen der einfallenden Lichtstrahlrichtung $\omega_i$ und der Oberflächennormalen $n$.
- **$\omega_i$**: Ein infinitesimaler Raumwinkel, aus dem das Licht einfällt.
Die BRDF wird verwendet, um die Beziehung zwischen einfallendem und reflektiertem Licht zu quantifizieren. Durch Integration über alle einfallenden Strahlen (alle $\omega_i$) kann die gesamte reflektierte Strahldichte berechnet werden:
$$L_r(x, \omega_r) = \int_{\Omega^+} f_r(\omega_i, x, \omega_r) L_i(x, \omega_i) \cos\theta_i d\omega_i$$

### **Eigenschaften der BRDF**
Eine physikalisch plausible BRDF erfüllt folgende Bedingungen:
1. **Nicht-Negativität**:
    - Die BRDF kann nicht negativ sein: fr(ωi,x,ωr)≥0f_r(\omega_i, x, \omega_r) \geq 0.
    - fr=0f_r = 0 beschreibt vollständige Absorption, fr→∞f_r \to \infty beschreibt perfekte Spiegelung (z. B. wie ein idealer Spiegel).
2. **Helmholtz-Reziprozität**:
    
    - Die BRDF ist symmetrisch in Bezug auf Ein- und Ausfallsrichtung: fr(ωi,x,ωr)=fr(ωr,x,ωi)f_r(\omega_i, x, \omega_r) = f_r(\omega_r, x, \omega_i)
    - Das bedeutet, dass der Lichtweg umkehrbar ist, eine Eigenschaft, die aus den Maxwell-Gleichungen folgt.
3. **Energieerhaltung**:
    
    - Die reflektierte Energie kann nicht größer sein als die einfallende Energie: ∫Ω+fr(ωi,x,ωr)cos⁡θrdωr≤1\int_{\Omega^+} f_r(\omega_i, x, \omega_r) \cos\theta_r d\omega_r \leq 1
    - Diese Bedingung stellt sicher, dass keine physikalisch unmögliche Lichtverstärkung auftritt.

### **Zusammenhang mit Radiometrie**

- Die BRDF verbindet die **Strahldichte** (LL) und die **Flussdichte** (EE).
- Strahldichte (LL) berücksichtigt die Richtung des Lichts, während Flussdichte (EE) nur die Menge an Lichtleistung auf einer Fläche beschreibt.
### Fazit
Die BRDF ist ein zentraler Bestandteil in der Computergrafik, da sie die Wechselwirkung zwischen Licht und Oberflächen beschreibt. Sie ermöglicht es, realistische Reflexionseffekte zu simulieren, indem sie die Verteilung der reflektierten Strahldichte basierend auf einfallendem Licht berechnet.


Distributed Raytracing ist eine Erweiterung des klassischen Raytracing:

1. **Unterschied zu Whitted-Style Raytracing**:
    
    - Beim klassischen Raytracing werden nur einzelne reflektierte und gebrochene Strahlen verfolgt.
    - Distributed Raytracing verfolgt mehrere Sekundärstrahlen, die zufällig verteilt sind, um realistische Effekte zu erzeugen.
2. **Effekte**:
    
    - Tiefenschärfe, Bewegungsunschärfe, weiche Schatten, unscharfe Reflexionen.
3. **Rekursion**:
    
    - Für jeden Sekundärstrahl wird die Rendering-Gleichung erneut ausgewertet, bis ein Abbruchkriterium (z. B. Pfadlänge oder Energie) erreicht ist.
    - Beispiel: "Russisches Roulette" zur stochastischen Bestimmung, ob ein Strahl weiter verfolgt wird.
4. **Phong-Modell als BRDF**:
    
    - Die BRDF kann durch ein Phong-Reflexionsmodell vereinfacht werden: fr(ωi,x,ω)=kd+ks⋅(Phong-Term)f_r(\omega_i, x, \omega) = k_d + k_s \cdot (\text{Phong-Term})