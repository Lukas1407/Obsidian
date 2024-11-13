- Die DNA-Ligase ist ein Enzym, das DNA-Stränge durch die Bildung einer Phosphodiesterbindung zwischen den 3'-Hydroxyl- (OH) und 5'-Phosphat- (P) Gruppen der DNA verknüpft.
-  Es gibt zwei Arten von Enden, die DNA-Fragmente haben können:
        - **Sticky Ends (klebrige Enden)**: Diese Enden haben komplementäre Überhänge, die sich spezifisch paaren können, wodurch die Ligase leichter eine stabile Verbindung herstellt.
        - **Blunt Ends (glatte Enden)**: Hier gibt es keine Überhänge, und die Enden sind flach. Die Verknüpfung von blunt ends ist schwieriger, da sie weniger stabil sind und keine komplementären Basen zur Orientierung haben.
## T4-DNA-Ligase:
- Diese Ligase stammt aus dem T4-Bakteriophagen und verwendet **ATP** als Energiequelle.
- Die T4-DNA-Ligase kann sowohl **sticky ends** als auch **blunt ends** verknüpfen. Sie ist besonders vielseitig und wird deshalb häufig in der Molekularbiologie verwendet, insbesondere bei DNA-Klonierungsexperimenten, bei denen verschiedene Endtypen vorkommen können.
- **Anwendung**: T4-DNA-Ligase wird fast immer für Klonierungsverfahren eingesetzt, da sie in der Lage ist, jede Art von DNA-Enden zu verknüpfen.
## E. coli-DNA-Ligase:
- Diese Ligase wird aus dem Bakterium E. coli gewonnen und verwendet **NAD+** als Energiequelle (anstelle von ATP).
- Die E. coli-DNA-Ligase kann nur **sticky ends** von doppelsträngiger DNA verknüpfen. Sie ist nicht in der Lage, blunt ends zu verbinden, was ihre Anwendung auf bestimmte Situationen einschränkt.
- **Anwendung**: Aufgrund ihrer eingeschränkten Funktion wird sie seltener verwendet und ist spezifisch für Anwendungen geeignet, bei denen nur sticky ends vorliegen.
## Verknüpfungsprozess
- In den Bildern ist der Mechanismus der Ligation dargestellt, wobei ATP oder NAD+ verwendet wird, um AMP bereitzustellen, das die Phosphatgruppe aktiviert und die Bildung der Phosphodiesterbindung ermöglicht.
- Die **T4-DNA-Ligase** verbindet blunt und sticky ends durch die Bereitstellung von AMP (Aktivator), was besonders hilfreich ist, wenn DNA-Fragmente ohne klebrige Enden vorliegen.
- Die **E. coli-DNA-Ligase** hingegen ist spezialisiert auf die Verknüpfung von sticky ends und nutzt NAD+ für denselben Aktivierungsprozess.

Gerne erkläre ich die gezeigten Schritte zur DNA-Ligation und die Probleme sowie Lösungen, die auftreten können:

## Typische DNA-Ligationsprobleme
1. **Ineffiziente Reaktion**: Manchmal ist die Ligation nicht effizient, was bedeutet, dass die DNA-Fragmente nicht gut zusammengefügt werden. Dies kann aufgrund von ungünstigen Bedingungen wie pH-Wert oder Konzentrationen von Enzymen und Substraten geschehen.

2. **Selbstligation**: Ein häufiges Problem bei der Ligation ist, dass der Vektor ohne das gewünschte DNA-Insert zirkularisiert, also eine Selbstligation durchführt. Dies führt dazu, dass der Vektor in einer geschlossenen Kreisform vorliegt, jedoch ohne das gewünschte Fremd-DNA-Fragment.
### Lösung für Selbstligation: Dephosphorylierung
Um die Selbstligation zu verhindern, kann der Vektor mit einer **Alkalischen Phosphatase** behandelt werden. Dieses Enzym entfernt die Phosphatgruppen an den Enden des Vektors, sodass diese Enden nicht mehr durch die Ligase verbunden werden können. Dadurch wird verhindert, dass der Vektor ohne das Insert zirkularisiert. Es ist wichtig, dass nur **eins** der beiden DNA-Fragmente (entweder der Vektor oder das Insert) dephosphoryliert wird, damit die Ligation immer noch stattfinden kann, wenn das Insert eingefügt wird.

# Inkompatible Enden und Adapter/Linker
Manchmal gibt es Situationen, in denen die Enden von zwei DNA-Fragmenten nicht kompatibel sind, also keine komplementären sticky ends aufweisen, die sich aneinanderlagern könnten. Hier gibt es zwei Lösungsansätze:

1. **Adapter**: Ein Adapter ist ein kurzer DNA-Abschnitt, der an das DNA-Fragment angebaut wird, um ein gewünschtes sticky end zu erzeugen. In der Abbildung wird beispielsweise ein Adapter verwendet, um ein EcoRI-spezifisches sticky end an ein DNA-Fragment anzufügen. Die Adapterstücke können dann durch T4-DNA-Ligase mit dem Vektor verbunden werden.
![[Pasted image 20241112101347.png#invert|400]]
2. **Linker**: Ein Linker ist ebenfalls ein kurzer Doppelstrang, der an ein blunt end (glattes Ende) des DNA-Fragments angebaut wird, aber zusätzlich eine spezifische Restriktionsschnittstelle enthält. Nach der Anfügung des Linkers und einer Verdauung mit dem entsprechenden Restriktionsenzym entsteht ein sticky end. Dadurch kann ein blunt end in ein sticky end umgewandelt werden, um eine Ligation mit dem Vektor zu ermöglichen.
![[Pasted image 20241112101406.png#invert|400]]
