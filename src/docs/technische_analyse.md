AUSFÜHRLICHE ERLÄUTERUNG DES WALLBOX-STEUERUNGS-CODES (VERSION 2.1)

ÜBERBLICK
Der Code realisiert eine PV-Überschussladesteuerung für eine Wallbox auf einem AVR-Mikrocontroller.
Aufgabe: Den verfügbaren PV-Überschuss (Leistung in Watt) messen, filtern, daraus die optimale Phasenzahl (1 oder 3 Phasen) und den Ladestrom pro Phase berechnen und die Hardware (PWM-Steller, Relais) entsprechend ansteuern.
Das System ist sicherheitsorientiert: Plausibilitätsprüfungen, Fehlerzähler, Watchdog, eine Zustandsmaschine für stromloses Umschalten (Zero Current Switching) und ein Self‑Healing‑Mechanismus bei Hardwarefehlern.

Die Software ist modular aufgebaut:
- Konfigurations-Makros (alle einstellbaren Parameter)
- Statische Compile‑Zeit‑Prüfungen (_Static_assert)
- DSP‑Koeffizienten für vier verschiedene Fensterfunktionen (wahlweise)
- Datentypen für die Zustände
- Hardware‑Abstraktions‑Schnittstelle (externe Funktionen)
- Logikbausteine (Filter, Hysterese, Stromberechnung, Hardware‑Controller)
- Hauptschleife mit 10‑Hz‑Takt

Im Folgenden werden alle Teile detailliert beschrieben.

1. KONFIGURATION (Abschnitt 1)
Über #define‑Konstanten werden alle anpassbaren Größen festgelegt:
- FILTER_WINDOW_SELECTION: Auswahl der Fensterfunktion für den Sinc‑Filter (1=Boxcar, 2=Hamming, 3=Blackman‑Harris, 4=Kaiser).
- NUM_TAPS (32): Anzahl der Filterkoeffizienten (muss Zweierpotenz sein, damit der Ringpuffer per Bitmaske arbeiten kann).
- DECIMATION_M (4): Dezimationsfaktor – nur jeder 4. Eingangswert erzeugt einen neuen Filterausgang (reduziert Rechenlast).
- PV_LIMIT_LOW / HIGH: Plausibilitätsgrenzen des PV‑Sensors in Watt.
- PV_MAX_ERRORS (50): Maximale Anzahl aufeinanderfolgender Sensorfehler, bevor das System abschaltet (entspricht 5 s bei 100 ms Takt).
- THRESHOLD_1_TO_3_W (4500) und THRESHOLD_3_TO_1_W (3500): Schwellwerte für die Hysterese (Umschalten zwischen 1 und 3 Phasen).
- DELAY_UP_SECONDS (60) / DELAY_DOWN_SECONDS (120): Verzögerungszeiten für die Hysterese (umschalten erst nach anhaltender Überschreitung).
- CAR_RAMPDOWN_TICKS (50): Wartezeit nach Abschalten des Stroms, bis die Relais umgeschaltet werden dürfen (5 s).
- RELAY_SETTLE_TICKS (5): Wartezeit nach dem Relais‑Umschalten, bis der neue Strom gesetzt wird (0,5 s).
- MAX_HW_RETRIES (3): Maximale Wiederholungen bei fehlgeschlagenen Hardware‑Befehlen (PWM/Relais).
- VOLTAGE_LN_V (230): Netzspannung Phase‑Neutralleiter in Volt.
- MIN_CHARGE_CURRENT_DA (60) / MAX_CHARGE_CURRENT_DA (160): Minimaler und maximaler Ladestrom in Dezi‑Ampere (6,0 A … 16,0 A).

2. STATISCHE SICHERHEITS‑WÄCHTER (Abschnitt 2)
_Static_assert‑Prüfungen stellen zur Compile‑Zeit sicher, dass die Konfiguration keine offensichtlichen Fehler enthält (z.B. NUM_TAPS als Zweierpotenz, Bereichsgrenzen für Zähler‑Typen). Das verhindert Laufzeitfehler durch Überläufe.

3. DSP‑KOEFFIZIENTEN (Abschnitt 3)
Je nach gewählter Fensterfunktion wird ein Array von 32 int16_t‑Koeffizienten definiert. Diese Werte sind bereits skaliert (Q15‑Format), sodass die Summe etwa 32768 beträgt (Gleichsignal‑Durchlassfaktor 1). Die Koeffizienten liegen im Flash‑Speicher (PROGMEM), um RAM zu sparen. Ein ausführlicher Kommentar erläutert die Filterauslegung (angenommene Abtastfrequenz 10 Hz, Grenzfrequenz ca. 2 Hz) und den Skalierungsfaktor.

4. DATENTYPEN (Abschnitt 4)
- PhaseMode: Enum für mögliche Phasenzustände (OFF, 1, 3). Die Werte 0,1,3 sind bewusst so gewählt, dass sie direkt als Anzahl der Phasen verwendet werden können.
- ChargeCommand: Struktur, die den Befehl für die Hardware enthält: gewünschte Phasenzahl und Zielstrom in Dezi‑Ampere.
- FIR_FilterState: Zustand des FIR‑Filters (Ringpuffer delay_line, Schreibindex, Dezimationszähler).
- PhaseHysteresis: Zustand der Hysterese (aktueller Zustand, Timer für Hoch‑ und Runterzählen).
- ZCS_State: Enum für die Zustände der Hardware‑Zustandsmaschine (IDLE, WAIT_CAR_STOP, WAIT_RELAY_SETTLE).
- HardwareController: Struktur mit aktuellem Hardware‑Zustand, aktuell eingestellter Phasenzahl, anstehender Phasenzahl, Timer, Fehlerflag und Wiederholungszähler.

5. HARDWARE‑ABSTRAKTION (Abschnitt 5)
Drei externe Funktionen werden vorausgesetzt:
- int16_t Read_PV_Surplus_Watts(void): Liest den aktuellen PV‑Überschuss in Watt.
- bool Set_PWM_Output_Amps(int16_t deci_amps): Stellt den Ladestrom ein (Rückgabe true bei Erfolg).
- bool Set_Relays(PhaseMode phases): Schaltet die Relais für die gewünschte Phasenzahl (Rückgabe true bei Erfolg).
Diese Funktionen müssen hardwarenah implementiert werden (z.B. über ADC, PWM‑Peripherie, GPIOs).

6. LOGIKBAUSTEINE (Abschnitt 6)

6.1 Validate_PV_Value
Prüft den Rohwert des PV‑Sensors gegen die Plausibilitätsgrenzen. Bei einem gültigen Wert wird der Fehlerzähler zurückgesetzt; bei ungültigem Wert wird der Zähler inkrementiert (bis PV_MAX_ERRORS). Die Funktion gibt true zurück, wenn der Wert gültig ist, sonst false.

6.2 FIR_Init und FIR_Process
- FIR_Init: Setzt den Ringpuffer (delay_line) auf Null und initialisiert Schreibindex und Dezimationszähler.
- FIR_Process:
  - Das aktuelle Sample wird in den Ringpuffer an Position write_idx geschrieben.
  - Der Dezimationszähler wird erhöht. Nur wenn er DECIMATION_M erreicht, wird ein neuer Ausgangswert berechnet:
    * Faltung (MAC): Über alle 32 Koeffizienten wird acc = Σ (coeff[i] * delay_line[read_idx]) aufgebaut, wobei read_idx rückwärts durch den Ringpuffer läuft (wegen der Bitmaske TAP_MASK automatischer Überlauf).
    * Die Koeffizienten werden mit pgm_read_word aus dem Flash gelesen.
    * Nach der Faltung wird acc um 16384 erhöht (entspricht 2^14) und um 15 Bit nach rechts geshiftet. Das ist eine Rundungsoperation: Durch die Addition von 0,5 (in der Q15‑Skalierung) und den Rechtsshift wird das Ergebnis gerundet und vom Q15‑Format in eine Ganzzahl mit 16‑Bit‑Bereich überführt.
    * Sättigungsarithmetik: Falls das Ergebnis außerhalb des int16_t‑Bereichs liegt, wird es auf 32767 oder -32768 begrenzt. Das verhindert Überläufe bei extremen Eingangssignalen.
    * Das Ergebnis wird in *out_sample gespeichert, und die Funktion liefert 1 zurück (neuer Wert verfügbar).
  - Unabhängig davon, ob ein neuer Ausgang berechnet wurde, wird der Schreibindex weitergeschaltet (write_idx = (write_idx + 1) & TAP_MASK).
Der Filter arbeitet also mit jedem Sample (10 Hz), führt die rechenintensive Faltung aber nur jedes 4. Sample aus (Dezimation). Das spart CPU‑Zeit.

6.3 Hysteresis_Init und Hysteresis_Process
- Hysteresis_Init: Setzt den aktuellen Phasenzustand auf PHASES_1 und beide Timer auf 0.
- Hysteresis_Process:
  - Wenn aktuell 1‑phasig geladen wird, prüft die Funktion, ob der geglättete PV‑Wert (pv_surplus_w) den oberen Schwellwert (THRESHOLD_1_TO_3_W) erreicht oder überschreitet.
    * Falls ja, wird timer_up erhöht. Sobald timer_up DELAY_UP_SECONDS erreicht, wird auf 3‑phasig umgeschaltet und beide Timer werden zurückgesetzt.
    * Falls nein, wird timer_up zurückgesetzt (damit eine kurzzeitige Überschreitung nicht zählt).
  - Analog für den 3‑phasigen Zustand: Unterschreitung des unteren Schwellwerts (THRESHOLD_3_TO_1_W) führt zu timer_down‑Inkrement; bei Erreichen von DELAY_DOWN_SECONDS wird auf 1‑phasig zurückgeschaltet.
Die Hysterese verhindert ein häufiges Hin‑ und Herschalten (Pendeln) bei schwankender PV‑Leistung.

6.4 Calculate_Charge_Power
Berechnet aus dem geglätteten PV‑Wert und der gewünschten Phasenzahl den einzustellenden Strom pro Phase.
- Formel: I_pro_Phase = P_gesamt / (U_Phase * Anzahl_Phasen)
- Da der Strom in Dezi‑Ampere (1/10 A) ausgegeben werden soll, wird vor der Division mit 10 multipliziert: current_da = (pv_surplus_w * 10) / (active_phases * VOLTAGE_LN_V).
- Anschließend wird der Wert auf den zulässigen Bereich (MIN_CHARGE_CURRENT_DA … MAX_CHARGE_CURRENT_DA) begrenzt. Bei Unterschreitung des Minimums wird der Strom auf 0 gesetzt (keine Ladung), bei Überschreitung auf das Maximum begrenzt.
- Falls active_phases ungültig ist (z.B. PHASES_OFF), wird sofort ein Befehl mit PHASES_OFF und Strom 0 zurückgegeben.

6.5 HardwareController_Init
Initialisiert die Hardware‑Zustandsmaschine:
- state = ZCS_IDLE, actual_hardware_phases = PHASES_1, pending_phases = PHASES_1, timer = 0, retry_counter = 0.
- Führt einen Selbsttest durch: Setzt PWM auf 0 und Relais auf 1‑phasig. Falls einer der Befehle fehlschlägt, wird hardware_fault = true gesetzt.

6.6 Execute_Hardware_Command
Zustandsmaschine zur sicheren Ansteuerung von PWM und Relais (Zero Current Switching – ZCS). Sie sorgt dafür, dass die Relais immer im stromlosen Zustand geschaltet werden.
- ZCS_IDLE:
  - Wenn der Befehl einen Phasenwechsel verlangt (cmd.active_phases ist 1 oder 3 und unterscheidet sich von actual_hardware_phases), wird zuerst der Strom auf 0 gesetzt (Set_PWM_Output_Amps(0)). Bei Erfolg wechselt die Maschine in den Zustand ZCS_WAIT_CAR_STOP und speichert die Zielphase in pending_phases. Fehler führen zu hardware_fault = true.
  - Andernfalls (kein Phasenwechsel) wird der gewünschte Strom direkt gesetzt. Bei Erfolg wird hardware_fault zurückgesetzt und der retry_counter gelöscht; bei Misserfolg wird hardware_fault gesetzt.
- ZCS_WAIT_CAR_STOP:
  - Hier wird gewartet (timer hochzählen), bis CAR_RAMPDOWN_TICKS erreicht sind. Danach wird versucht, die Relais auf die pending_phases umzuschalten. Bei Erfolg: actual_hardware_phases = pending_phases, Wechsel zu ZCS_WAIT_RELAY_SETTLE, timer zurücksetzen, hardware_fault = false. Bei Misserfolg: retry_counter erhöhen; wenn MAX_HW_RETRIES erreicht, wird hardware_fault = true gesetzt.
- ZCS_WAIT_RELAY_SETTLE:
  - Wartezeit (timer) bis RELAY_SETTLE_TICKS, damit die Relaiskontakte sicher eingeschwungen sind. Danach Rückkehr zu ZCS_IDLE und retry_counter zurücksetzen.
Die Maschine bleibt auch während der Wartezeiten empfänglich für neue Befehle? Nein, neue Befehle werden erst im IDLE‑Zustand bearbeitet. Ein laufender Umschaltvorgang wird nicht unterbrochen – das ist beabsichtigt, um die Sicherheit zu gewährleisten.

7. HAUPTLOOP (main, Abschnitt 7)

Globale Instanzen der Zustandsstrukturen: pv_filter, phase_hysteresis, hw_controller.

main() führt folgende Schritte aus:
- Watchdog aktivieren (2 s Timeout).
- Initialisierung der drei Module.
- Lokale Variablen: tick_counter_1sec (für die 1‑Sekunden‑Logik), sensor_error_counter, current_cmd (aktueller Befehl), smooth_pv (gefilterter PV‑Wert), last_valid_pv (letzter gültiger PV‑Wert).

Dann Endlosschleife (10 Hz Takt):
1. Rohwert des PV‑Sensors lesen (raw_pv).
2. Sensorwert validieren:
   - Bei Gültigkeit: last_valid_pv = raw_pv.
   - Bei Ungültigkeit: last_valid_pv = 0 (sicherer Ersatzwert, verhindert Einfrieren alter Werte).
3. Prüfen, ob ein kritischer Fehler vorliegt:
   - sensor_error_counter >= PV_MAX_ERRORS (dauerhafte Sensorfehler) ODER
   - hw_controller.hardware_fault (Hardwarefehler).
   Falls ja: current_cmd = (PHASES_OFF, 0). Der Befehl wird später an die Hardware‑Maschine übergeben (Not‑Aus).
4. Andernfalls (kein kritischer Fehler):
   - FIR_Process aufrufen (Eingang last_valid_pv, Ausgang smooth_pv). Der Rückgabewert wird ignoriert, weil smooth_pv immer den aktuellsten Wert enthält (wenn gerade kein neuer berechnet wurde, bleibt er unverändert – das ist für die träge Regelung in Ordnung).
   - tick_counter_1sec erhöhen; wenn 10 erreicht (d.h. 1 Sekunde vergangen), wird die Logik ausgeführt:
     * Hysteresis_Process mit smooth_pv aufrufen → target_phases.
     * Calculate_Charge_Power mit smooth_pv und target_phases → current_cmd.
5. Hardware‑Aktion: Execute_Hardware_Command mit current_cmd aufrufen (auch im Not‑Aus, damit der Abschaltbefehl ausgeführt wird).
6. 100 ms warten (_delay_ms(100)). Dies ist ein blockierendes Delay – die CPU pausiert hier. Für diese Anwendung ausreichend, aber für zukünftige Erweiterungen (Kommunikation) wäre ein Timer‑basiertes Design besser.
7. Watchdog zurücksetzen (wdt_reset()). Nur wenn die Schleife komplett durchlaufen wurde, erhält der Watchdog ein Lebenszeichen. Bleibt das System in einem Fehlerzustand hängen, löst der Watchdog nach 2 s einen Reset aus.

8. SICHERHEITSASPEKTE
- Statische Asserts fangen Konfigurationsfehler ab.
- Sensorvalidierung mit Fehlerzähler verhindert, dass einzelne Ausreißer sofort zum Not‑Aus führen.
- Bei Sensorfehlern wird last_valid_pv auf 0 gesetzt – das lässt den Filterausgang schnell abfallen, aber das System kann nach Ende der Fehlerserie sofort wieder reagieren.
- Hardware‑Zustandsmaschine trennt Strom vor dem Relais‑Umschalten (ZCS) – verhindert Lichtbögen und verlängert die Lebensdauer.
- Retry‑Zähler bei Relais‑Fehlern (3 Versuche) und anschließendes Setzen von hardware_fault.
- Self‑Healing: Bei erfolgreichen Hardware‑Befehlen wird hardware_fault zurückgesetzt, sodass das System nach einem kurzzeitigen Fehler automatisch weiterarbeitet.
- Watchdog als letzte Sicherung.

9. EINSCHRÄNKUNGEN UND HINWEISE
- Die Filterkoeffizienten sind für eine angenommene Abtastfrequenz von 10 Hz ausgelegt. Wird die tatsächliche Frequenz geändert, müssen die Koeffizienten neu berechnet werden.
- Das blockierende Delay verhindert Interrupt‑gesteuerte Nebenaufgaben. Für reine Regelung ausreichend, aber bei Kommunikationsaufgaben (z.B. Modbus) sollte auf einen Timer‑Interrupt umgestellt werden.
- Die Hardware‑API wird als korrekt vorausgesetzt; Fehlercodes müssen zuverlässig sein.
- Die Simulation des Codes auf dem PC (z.B. mit onlinegdb) ist möglich, indem die AVR‑spezifischen Teile durch Makros und die Hardware‑Stubs durch Konsolenausgaben ersetzt werden (wie bereits gezeigt).

ZUSAMMENFASSUNG
Der Code realisiert eine vollständige, sichere PV‑Überschussladesteuerung mit digitalem Filter, Hysterese und Hardware‑Schutz. Er ist wartungsfreundlich durch klare Struktur, ausführliche Kommentare und Compile‑Zeit‑Prüfungen. Ein erfahrener C‑Entwickler kann ihn ohne tiefe Elektrotechnik‑Kenntnisse anpassen, sofern er die grundlegenden Konzepte (Sinc‑Filter, Q15, ZCS) versteht – die Kommentare liefern dazu alle nötigen Erklärungen.
