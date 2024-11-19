> [!summary] 
> Surrogatfunktionen sind in der [[Optimierungsprobleme|Optimierung]] besonders nützlich, wenn die eigentliche Zielfunktion $Q(p)$ nicht explizit gegeben ist und einzelne Werte erst aufwändig ermittelt werden müssen. Z.B. durch Experimente oder aufwendige Simulationen ([[Finite Element Method]])

- **Surrogatmodell**: Ein Surrogatmodell wird entwickelt, um den Verlauf von $Q(p)$ zu schätzen. Dieses Modell ist einfacher zu bewerten als die tatsächliche Zielfunktion. Z.B in Form eines kNN
- **Trainingsdaten**: Vorhandene Datenpunkte, die durch Experimente oder Simulationen gewonnen wurden, dienen als Trainingsdaten für das Surrogatmodell.
- **Optimierungsläufe**: Mit dem Surrogatmodell werden viele Optimierungsläufe durchgeführt, um Bereiche von Interesse zu identifizieren.
- **Validierung**: Die Ergebnisse des Surrogatmodells werden dann durch weitere Experimente oder Simulationen in der Nähe des vermuteten Optimums überprüft und validiert.d