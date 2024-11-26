### **Rendering-Gleichung**

Die Rendering-Gleichung beschreibt die Lichtverteilung in einer Szene und wurde von James Kajiya 1986 eingeführt. Sie lautet:

L(x,ω)=Le(x,ω)+∫Ω+fr(ωi,x,ω)Li(x,ωi)cos⁡θidωiL(x, \omega) = L_e(x, \omega) + \int_{\Omega^+} f_r(\omega_i, x, \omega) L_i(x, \omega_i) \cos \theta_i d\omega_i

#### Bedeutung der Terme

1. **L(x,ω)L(x, \omega)**: Strahldichte in Richtung ω\omega von Punkt xx.
    
    - Beschreibt das Licht, das von einem Punkt xx in eine bestimmte Richtung ω\omega ausgesendet wird.
2. **Le(x,ω)L_e(x, \omega)**: Emissionsterm.
    
    - Licht, das die Oberfläche xx direkt emittiert, wie z. B. von Lichtquellen.
3. **∫Ω+…dωi\int_{\Omega^+} \ldots d\omega_i**: Reflexionsterm.
    
    - Integration über alle einfallenden Richtungen (ωi\omega_i) der positiven Hemisphäre (Ω+\Omega^+).
    - Beschreibt, wie viel einfallendes Licht (LiL_i) in die Richtung ω\omega reflektiert wird.
4. **fr(ωi,x,ω)f_r(\omega_i, x, \omega)**: Bidirectional Reflectance Distribution Function (BRDF).
    
    - Bestimmt, wie einfallendes Licht in verschiedene Richtungen reflektiert wird.
5. **cos⁡θi\cos \theta_i**: Winkelabhängigkeit.
    
    - Gewichtet das einfallende Licht basierend auf dem Winkel θi\theta_i zwischen der einfallenden Richtung ωi\omega_i und der Normalen nn der Oberfläche.

---

### **Monte Carlo-Integration**

Die Integration in der Rendering-Gleichung ist oft schwer analytisch lösbar. Stattdessen wird sie mit der Monte Carlo-Methode angenähert:

L(x,ω)≈Le(x,ω)+2πN∑i=1Nfr(ωi,x,ω)Li(x,ωi)cos⁡θiL(x, \omega) \approx L_e(x, \omega) + \frac{2\pi}{N} \sum_{i=1}^N f_r(\omega_i, x, \omega) L_i(x, \omega_i) \cos \theta_i

- **Monte Carlo-Methode**:
    
    - Zufällige Probepunkte (ωi\omega_i) werden aus der positiven Hemisphäre gezogen.
    - Die Beiträge der BRDF, einfallenden Strahldichte und Winkelabhängigkeit werden gemittelt.
    - Der Faktor 2π2\pi entspricht dem Raumwinkel der Hemisphäre.
- **Vorteile**:
    
    - Effizient für komplexe Szenen.
    - Unterstützt weiche Schatten, unscharfe Reflexionen und andere Effekte.
