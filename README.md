Integration & Hardware-Test (Checkliste)
Damit diese Steuerung auf deiner Hardware sicher läuft, prüfe vor dem Flashen folgende Punkte:

Hardware Abstraction Layer (HAL) implementieren: Du musst die drei extern-Funktionen (Sensorik lesen, Relais schalten, PWM setzen) für deine spezifische Hardware (z. B. I2C-Sensoren, GPIO-Pins) in einer separaten Datei implementieren.

Non-Blocking Treiber: Stelle sicher, dass deine implementierten I2C-/Hardware-Treiber bei Kommunikationsfehlern nicht in Endlosschleifen hängen bleiben, sondern mit false oder 0 zurückkehren. Nur so kann unsere Zustandsmaschine greifen.

Chaos-Test am lebenden Objekt: Führe einen abschließenden Test auf der echten Hardware durch. Simuliere einen Bus-Ausfall (z. B. durch Abziehen der I2C-SDA/SCL Leitungen im laufenden Betrieb). Das System MUSS den Fehlerzähler hochtreiben und nach exakt 1 Sekunde (10 Fehlversuchen) den Livelock-Schutz aktivieren, wodurch der Watchdog (WDT) nach weiteren 2 Sekunden einen harten Kaltstart erzwingt.
