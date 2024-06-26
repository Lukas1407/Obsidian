### Einführung in den Bakery-Algorithmus

Der **Bakery-Algorithmus** ist ein klassisches Synchronisationsverfahren in der Informatik, das von Leslie Lamport entwickelt wurde, um das Problem des gegenseitigen Ausschlusses in verteilten Systemen zu lösen. Er stellt sicher, dass mehrere Prozesse auf sichere Weise auf eine gemeinsame Ressource zugreifen können, ohne dass es zu Konflikten kommt.

#### Motivation und Anwendungsbeispiel

Der Name "Bakery-Algorithmus" leitet sich von der in vielen Bäckereien und Behörden üblichen Praxis ab, bei der Kunden beim Betreten eine Nummer ziehen und in der Reihenfolge der Nummern bedient werden. Diese Methode stellt sicher, dass jeder Kunde bedient wird und es keine Streitigkeiten darüber gibt, wer als nächstes an die Reihe kommt.

### Funktionsweise des Bakery-Algorithmus

Der Algorithmus verwendet numerische Tickets, um die Reihenfolge zu bestimmen, in der die Prozesse die Ressource betreten dürfen. Die Prozesse lassen sich als Instanzen der Klasse `Customer` vorstellen, wobei jeder `Customer` eine Ticketnummer und einen Status hat, der seinen aktuellen Zustand angibt.

#### Zustand eines Prozesses (Customer)

- **Ticket**: Eine numerische Kennzeichnung, die die Reihenfolge angibt.
- **Phase**: Der aktuelle Zustand des Prozesses. Mögliche Zustände sind:
  - **idle**: Der Prozess ist inaktiv und wartet auf den Zugriff.
  - **trying**: Der Prozess versucht, ein Ticket zu ziehen und in die kritische Phase einzutreten.
  - **critical**: Der Prozess befindet sich in der kritischen Phase und hat Zugriff auf die Ressource.

### Beschreibung des Bakery-Algorithmus

1. **Ziehen eines Tickets (trying)**:
   - Wenn ein Prozess in die kritische Phase eintreten möchte, zieht er ein Ticket, das größer ist als alle aktuell gezogenen Tickets der anderen Prozesse.
   - Jedes Ticket wird mit einer eindeutigen Prozessnummer kombiniert, um Kollisionen zu vermeiden.

2. **Warten auf den Zugriff (idle)**:
   - Der Prozess wartet, bis sein Ticket die kleinste Nummer unter den wartenden Prozessen ist.
   - Falls zwei Prozesse dieselbe Ticketnummer haben, wird die Prozessnummer als Sekundärkriterium verwendet, um die Reihenfolge zu bestimmen.

3. **Eintritt in die kritische Phase (critical)**:
   - Sobald der Prozess feststellt, dass sein Ticket das kleinste ist, betritt er die kritische Phase und erhält exklusiven Zugriff auf die Ressource.

4. **Verlassen der kritischen Phase**:
   - Nachdem der Prozess seine Operationen in der kritischen Phase abgeschlossen hat, gibt er sein Ticket zurück und wechselt in den `idle`-Zustand, um anderen Prozessen den Zugriff zu ermöglichen.

### Formale Beschreibung in Pseudocode

Hier ist eine einfache Darstellung des Bakery-Algorithmus in Pseudocode:

```pseudocode
shared int tickets[N] = [0, 0, ..., 0]  // Array für die Ticketnummern der N Prozesse
shared boolean choosing[N] = [false, false, ..., false]  // Array für die Auswahl der Tickets

// Funktion zum Betreten der kritischen Phase
void enter_critical_section(int i) {
    choosing[i] = true
    tickets[i] = 1 + max(tickets[0], tickets[1], ..., tickets[N-1])  // Höchstes Ticket + 1
    choosing[i] = false

    for (int j = 0; j < N; j++) {
        if (i != j) {
            // Warte, bis Prozess j sein Ticket gezogen hat
            while (choosing[j]) {}
            
            // Warte, bis Ticket von Prozess j kleiner ist oder das Ticket gleich ist und i < j
            while (tickets[j] != 0 && 
                   (tickets[j] < tickets[i] || 
                   (tickets[j] == tickets[i] && j < i))) {}
        }
    }
    // Jetzt in der kritischen Phase
}

// Funktion zum Verlassen der kritischen Phase
void leave_critical_section(int i) {
    tickets[i] = 0
}
```

### Eigenschaften des Bakery-Algorithmus

- **Fairness**: Jeder Prozess wird irgendwann die kritische Phase betreten, wenn er darauf wartet. Es gibt keine Priorisierung von Prozessen, sodass alle Prozesse fair behandelt werden.
- **Deadlock-Freiheit**: Der Algorithmus verhindert Deadlocks, da jeder Prozess irgendwann seine Ressource freigeben wird und kein Prozess unendlich lange auf den Zugang zur kritischen Phase warten muss.
- **Vermeidung von Starvation**: Kein Prozess wird unendlich lange warten müssen, da die Ticketnummern stetig erhöht werden und jedes Ticket letztendlich an der Reihe ist.
- **Gegenseitiger Ausschluss**: Zu jedem Zeitpunkt ist sichergestellt, dass nur ein Prozess in der kritischen Phase ist.

### Anwendung und Relevanz

Der Bakery-Algorithmus wird in verteilten Systemen und Mehrprozessorsystemen eingesetzt, um den gegenseitigen Ausschluss von Prozessen zu gewährleisten. Er ist besonders nützlich in Szenarien, in denen keine zentrale Instanz zur Verwaltung der Ressourcen vorhanden ist und die Prozesse unabhängig voneinander arbeiten.

### Fazit

Der Bakery-Algorithmus ist ein einfaches, aber effektives Synchronisationsverfahren, das Fairness und gegenseitigen Ausschluss gewährleistet, ohne dass es zu Deadlocks oder Starvation kommt. Er ist ein grundlegender Algorithmus in der Theorie der verteilten Systeme und ein wichtiges Konzept für die Synchronisation in parallelen und verteilten Umgebungen.

Falls du weitere Fragen oder spezifische Aspekte des Bakery-Algorithmus vertiefen möchtest, lass es mich wissen!

