/**
 * @file wallbox_control.c
 * @brief Hochverfügbare, industrielle Steuerung für PV-Überschussladen.
 * @version 2.2
 *
 * Diese Software implementiert eine vollständige DSP-Pipeline inklusive:
 * - Kausaler FIR-Filterung mit wählbaren Fensterfunktionen (Sinc-Kern).
 * - Polyphasen-Dezimierung zur Reduktion der CPU-Last auf AVR-Systemen.
 * - Zustandsbasierter Hysterese für materialschonende Phasenumschaltung.
 * - Hardware-Schutzschaltung (Zero Current Switching) mit Self-Healing.
 * - Mehrstufiger Sicherheitsarchitektur (Static Asserts, Watchdog, Pointer-Checks).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>   // NEU: Für Interrupt-Steuerung
#include <util/delay.h>       // Wird noch für Initialisierung genutzt, aber nicht mehr in der Hauptschleife

/* ==========================================================================
   1. KUNDEN-KONFIGURATION (DSP & LOGIK)
   ========================================================================== */

/**
 * @brief Auswahl der Fensterfunktion für den Sinc-Filter.
 * 1: BOXCAR (Sinc mit Rechteck-Fenster) - Schnellste Reaktion.
 * 2: HAMMING (Sinc mit Hamming-Fenster) - Der industrielle Standard.
 * 3: BLACKMAN-HARRIS - Maximale Rauschunterdrückung.
 * 4: KAISER - Beste Balance zwischen Dämpfung und Flankensteilheit.
 */
#define FILTER_WINDOW_SELECTION 2 

/** @brief Filter-Ordnung (Anzahl der Taps). Muss Zweierpotenz sein! */
#define NUM_TAPS             32
#define TAP_MASK             (NUM_TAPS - 1)

/** @brief Dezimationsfaktor (Berechnung findet nur jedes M-te Sample statt) */
#define DECIMATION_M         4

/** @brief Plausibilitätsgrenzen für den PV-Eingangssensor (Einheit: Watt) */
#define PV_LIMIT_LOW         -10000 
#define PV_LIMIT_HIGH         25000 
#define PV_MAX_ERRORS         50    /**< Entspricht 5.0s bei 100ms Taktung */

/** @brief Hysterese-Parameter für die Phasenumschaltung */
#define THRESHOLD_1_TO_3_W    4500  
#define THRESHOLD_3_TO_1_W    3500  
#define DELAY_UP_SECONDS      60
#define DELAY_DOWN_SECONDS    120

/** @brief Hardware-spezifische Sicherheitskonstanten */
#define CAR_RAMPDOWN_TICKS    50    /**< Wartezeit nach PWM=0 (5.0s) */
#define RELAY_SETTLE_TICKS    5     /**< Wartezeit nach Relais-Schaltung (0.5s) */
#define MAX_HW_RETRIES        3     /**< Versuche bei Hardware-Kommunikationsfehlern */

/** @brief Elektrische Grenzwerte */
#define VOLTAGE_LN_V          230   /**< Netzspannung Phase-Neutralleiter */
#define MIN_CHARGE_CURRENT_DA 60    /**< 6.0 Ampere (Minimum nach IEC 61851) */
#define MAX_CHARGE_CURRENT_DA 160   /**< 16.0 Ampere (Maximaler Ladestrom) */

/* ==========================================================================
   2. STATISCHE SICHERHEITS-WÄCHTER (Compile-Time Validation)
   ========================================================================== */

/* Struktur-Validierung */
_Static_assert(FILTER_WINDOW_SELECTION >= 1 && FILTER_WINDOW_SELECTION <= 4, "Ungueltiges DSP-Fenster gewaehlt");
_Static_assert((NUM_TAPS & TAP_MASK) == 0, "NUM_TAPS muss eine Zweierpotenz sein (Bitmasking-Anforderung)");

/* Datentyp-Validierung (Verhinderung von Overflows in den Zählern) */
_Static_assert(NUM_TAPS <= 256, "NUM_TAPS ueberschreitet uint8_t Indexbereich");
_Static_assert(DECIMATION_M <= 255, "DECIMATION_M ueberschreitet uint8_t Bereich");
_Static_assert(PV_MAX_ERRORS <= 255, "PV_MAX_ERRORS ueberschreitet uint8_t Bereich");

/* Symmetrische Validierung der Hysterese-Timer */
_Static_assert(DELAY_UP_SECONDS <= 65535, "DELAY_UP_SECONDS ueberschreitet uint16_t Zeitbereich");
_Static_assert(DELAY_DOWN_SECONDS <= 65535, "DELAY_DOWN_SECONDS ueberschreitet uint16_t Zeitbereich");

/* ==========================================================================
   3. DSP-KOEFFIZIENTEN-DATENBANK (Flash-Resident)
   ========================================================================== */

/**
 * @brief FIR-Filterkoeffizienten für verschiedene Fensterfunktionen.
 * 
 * Die Koeffizienten wurden mit dem Sinc-Fensterungsverfahren entworfen und
 * sind für eine Abtastfrequenz von 10 Hz ausgelegt (angenommen). Die Ziel-
 * Grenzfrequenz liegt bei ca. 2 Hz (angepasst an die träge PV-Leistungs-
 * änderung). Bei Änderung der Abtastfrequenz müssen die Koeffizienten neu
 * berechnet werden, da sich sonst die Filtercharakteristik verschiebt.
 * 
 * Die Werte sind bereits skaliert und können direkt in der MAC-Operation
 * verwendet werden. Die Summe der Koeffizienten ist bei allen Fenstern
 * etwa 32768, sodass der Gleichsignal-Durchlassfaktor 1 beträgt.
 */
#if (FILTER_WINDOW_SELECTION == 1)
  /* BOXCAR: Optimierte Sinc-Werte mit Rechteckfenster */
  const int16_t fir_coeffs[32] PROGMEM = {
      -146, -443, -712, -907, -986, -915, -676, -265, 301, 988, 1748, 2520, 3240, 3845, 4282, 4511,
      4511, 4282, 3845, 3240, 2520, 1748, 988, 301, -265, -676, -915, -986, -907, -712, -443, -146
  };
#elif (FILTER_WINDOW_SELECTION == 2)
  /* HAMMING: Optimierte Sinc-Werte mit Hamming-Fenster */
  const int16_t fir_coeffs[32] PROGMEM = {
      -10, -36, -75, -132, -198, -244, -231, -112, 152, 582, 1167, 1861, 2589, 3257, 3768, 4046,
      4046, 3768, 3257, 2589, 1861, 1167, 582, 152, -112, -231, -244, -198, -132, -75, -36, -10
  };
#elif (FILTER_WINDOW_SELECTION == 3)
  /* BLACKMAN-HARRIS: Optimierte Sinc-Werte mit Blackman-Harris-Fenster */
  const int16_t fir_coeffs[32] PROGMEM = {
      0, 0, -2, -9, -23, -44, -61, -40, 70, 335, 808, 1496, 2342, 3215, 3941, 4355,
      4355, 3941, 3215, 2342, 1496, 808, 335, 70, -40, -61, -44, -23, -9, -2, 0, 0
  };
#elif (FILTER_WINDOW_SELECTION == 4)
  /* KAISER: Optimierte Sinc-Werte mit Kaiser-Fenster */
  const int16_t fir_coeffs[32] PROGMEM = {
      0, 0, -1, -4, -12, -26, -40, -29, 56, 286, 728, 1411, 2288, 3224, 4022, 4482,
      4482, 4022, 3224, 2288, 1411, 728, 286, 56, -29, -40, -26, -12, -4, -1, 0, 0
  };
#endif

/* ==========================================================================
   4. DATENTYPEN & SYSTEM-ZUSTÄNDE
   ========================================================================== */

typedef enum { 
    PHASES_OFF = 0, 
    PHASES_1 = 1, 
    PHASES_3 = 3 
} PhaseMode;

typedef struct {
    PhaseMode active_phases;
    int16_t target_current_da; 
} ChargeCommand;

typedef struct {
    int16_t delay_line[NUM_TAPS];
    uint8_t write_idx;
    uint8_t decimation_counter;
} FIR_FilterState;

typedef struct {
    PhaseMode current_state;
    uint16_t timer_up;
    uint16_t timer_down;
} PhaseHysteresis;

typedef enum { 
    ZCS_IDLE = 0, 
    ZCS_WAIT_CAR_STOP, 
    ZCS_WAIT_RELAY_SETTLE 
} ZCS_State;

typedef struct {
    ZCS_State state;
    PhaseMode actual_hardware_phases;
    PhaseMode pending_phases; 
    uint16_t timer;
    bool hardware_fault; 
    uint8_t retry_counter; 
} HardwareController;

/* ==========================================================================
   5. HARDWARE-ABSTRAKTION (API)
   ========================================================================== */

extern int16_t Read_PV_Surplus_Watts(void);
extern bool    Set_PWM_Output_Amps(int16_t deci_amps); 
extern bool    Set_Relays(PhaseMode phases);           

/* ==========================================================================
   6. IMPLEMENTIERUNG DER LOGIK-BAUSTEINE
   ========================================================================== */

/**
 * @brief Prüft PV-Sensorwerte auf Plausibilität.
 * @param raw_val Aktueller Messwert des ADC/Sensors.
 * @param error_counter Zeiger auf den internen Fehlerzähler des Systems.
 * @return true bei validem Messwert, false bei Ausreißern.
 */
bool Validate_PV_Value(int16_t raw_val, uint8_t* error_counter) {
    assert(error_counter != NULL);
    if (raw_val < PV_LIMIT_LOW || raw_val > PV_LIMIT_HIGH) {
        if (*error_counter < PV_MAX_ERRORS) {
            (*error_counter)++;
        }
        return false;
    }
    *error_counter = 0; // Reset bei erfolgreicher Messung
    return true;
}

/**
 * @brief Initialisiert den FIR-Filterzustand.
 * @param state Zeiger auf die FIR-Struktur.
 */
void FIR_Init(FIR_FilterState* state) {
    assert(state != NULL);
    memset(state->delay_line, 0, sizeof(state->delay_line));
    state->write_idx = 0;
    state->decimation_counter = 0;
}

/**
 * @brief Echtzeitfähige Polyphasen-Filterung.
 * @details Schiebt Samples mit 10Hz ein, führt die schwere Faltung aber nur
 * jedes DECIMATION_M-te Mal aus, um CPU-Zeit zu sparen.
 * @return 1 wenn ein neuer dezimierter Wert berechnet wurde, sonst 0.
 */
uint8_t FIR_Process(FIR_FilterState* state, int16_t in_sample, const int16_t* coeffs, int16_t* out_sample) {
    assert(state != NULL); 
    assert(coeffs != NULL);
    
    // Sample-Einzug in den Ringpuffer
    state->delay_line[state->write_idx] = in_sample;
    uint8_t new_output_available = 0;

    // Dezimations-Prüfung
    state->decimation_counter++;
    if (state->decimation_counter >= DECIMATION_M) {
        state->decimation_counter = 0;
        assert(out_sample != NULL);

        // Faltung (MAC-Operationen)
        int32_t acc = 0;
        uint8_t read_idx = state->write_idx;
        for (uint8_t i = 0; i < NUM_TAPS; i++) {
            int16_t coeff = (int16_t)pgm_read_word(&coeffs[i]);
            acc += (int32_t)coeff * state->delay_line[read_idx];
            read_idx = (read_idx - 1) & TAP_MASK; 
        }

        // Rundung: Durch Addition von 2^14 (16384) und anschließenden Rechtsshift um 15 Bit
        // wird der Akkumulator auf den nächsten ganzzahligen Wert gerundet.
        // Die Skalierung ist nötig, weil die Koeffizienten bereits so skaliert sind,
        // dass das Ergebnis im Q15-Format vorliegt (1 Vorzeichenbit, 15 Bruchbits).
        acc += 16384; 
        int32_t result = (acc >> 15);
        
        // Sättigungsarithmetik: Falls das Ergebnis außerhalb des 16-Bit-Bereichs liegt,
        // wird es auf den maximalen/minimalen Wert begrenzt. Dies verhindert Überläufe
        // bei extremen Eingangssignalen.
        if (result > 32767) result = 32767; 
        else if (result < -32768) result = -32768;
        
        *out_sample = (int16_t)result;
        new_output_available = 1;
    }

    state->write_idx = (state->write_idx + 1) & TAP_MASK;
    return new_output_available;
}

/**
 * @brief Initialisiert die Hysteresesteuerung.
 * @param hyst Zeiger auf die Hysterese-Struktur.
 */
void Hysteresis_Init(PhaseHysteresis* hyst) {
    assert(hyst != NULL);
    hyst->current_state = PHASES_1; 
    hyst->timer_up = 0;
    hyst->timer_down = 0;
}

/**
 * @brief Entscheidet über die benötigte Phasenanzahl basierend auf PV-Überschuss.
 * @param pv_surplus_w Der bereits geglättete PV-Leistungswert.
 */
PhaseMode Hysteresis_Process(PhaseHysteresis* hyst, int32_t pv_surplus_w) {
    assert(hyst != NULL);
    if (hyst->current_state == PHASES_1) {
        if (pv_surplus_w >= THRESHOLD_1_TO_3_W) {
            hyst->timer_up++;
            if (hyst->timer_up >= DELAY_UP_SECONDS) {
                hyst->current_state = PHASES_3; 
                hyst->timer_up = 0; 
                hyst->timer_down = 0; 
            }
        } else {
            hyst->timer_up = 0;
        }
    } else if (hyst->current_state == PHASES_3) {
        if (pv_surplus_w <= THRESHOLD_3_TO_1_W) {
            hyst->timer_down++;
            if (hyst->timer_down >= DELAY_DOWN_SECONDS) {
                hyst->current_state = PHASES_1; 
                hyst->timer_down = 0; 
                hyst->timer_up = 0; 
            }
        } else {
            hyst->timer_down = 0;
        }
    }
    return hyst->current_state;
}

/**
 * @brief Berechnet den Soll-Strom in Deziamper pro Phase.
 * 
 * Physikalische Formel: I_pro_Phase = P_gesamt / (U_Phase * Anzahl_Phasen)
 * (hier vereinfacht für ohmsche Last, cos φ = 1).
 * Da der Strom in Dezi-Ampere (1/10 A) ausgegeben werden soll, wird das
 * Ergebnis mit 10 multipliziert. Beispiel: 2300 W / (230 V * 3) = 3,333 A
 * -> 33,33 dA. Durch die Multiplikation mit 10 im Zähler ((pv_surplus_w * 10) / denominator) ergibt sich der Wert direkt in dA.
 * 
 * @param pv_surplus_w Geglättete PV-Leistung in Watt.
 * @param active_phases Gewünschte Phasenanzahl (1 oder 3).
 * @return ChargeCommand mit Phasenmodus und Zielstrom.
 */
ChargeCommand Calculate_Charge_Power(int32_t pv_surplus_w, PhaseMode active_phases) {
    ChargeCommand cmd = {active_phases, 0};
    
    // Sicherheit-Fallback
    if (active_phases != PHASES_1 && active_phases != PHASES_3) {
        cmd.active_phases = PHASES_OFF; 
        return cmd;
    }

    int32_t denominator = (int32_t)active_phases * VOLTAGE_LN_V;
    int16_t current_da = (int16_t)((pv_surplus_w * 10) / denominator);

    if (current_da < MIN_CHARGE_CURRENT_DA) {
        cmd.target_current_da = 0;
    } else if (current_da > MAX_CHARGE_CURRENT_DA) {
        cmd.target_current_da = MAX_CHARGE_CURRENT_DA;
    } else {
        cmd.target_current_da = current_da;
    }
    
    return cmd;
}

/**
 * @brief Hardware-Boot-Initialisierung mit Selbsttest.
 */
void HardwareController_Init(HardwareController* hw) {
    assert(hw != NULL);
    hw->state = ZCS_IDLE;
    hw->actual_hardware_phases = PHASES_1;
    hw->pending_phases = PHASES_1;
    hw->timer = 0;
    hw->retry_counter = 0;
    
    // Kritischer HW-Check beim Systemstart
    bool pwm_ok = Set_PWM_Output_Amps(0);
    bool relay_ok = Set_Relays(PHASES_1);
    hw->hardware_fault = (!pwm_ok || !relay_ok);
}

/**
 * @brief Zustandsmaschine für sichere Hardware-Steuerung (ZCS & Self-Healing).
 * @details Überwacht PWM- und Relaisrückmeldungen. Wenn die Hardware sich erholt,
 * löscht das System den Fehlerzustand automatisch (Self-Healing).
 * 
 * Zustände:
 * - ZCS_IDLE: Keine laufende Umschaltung. Entweder wird der Strom direkt gesetzt
 *   oder bei einem Phasenwechsel in den WAIT_CAR_STOP-Zustand gewechselt.
 * - ZCS_WAIT_CAR_STOP: Nachdem PWM auf 0 gesetzt wurde, wird gewartet (5 s), damit
 *   der Strom tatsächlich abklingt, bevor die Relais umgeschaltet werden.
 * - ZCS_WAIT_RELAY_SETTLE: Nach dem Umschalten der Relais wird kurz (0,5 s) gewartet,
 *   bis die Kontakte eingeschwungen sind, bevor der neue Strom gesetzt wird.
 */
void Execute_Hardware_Command(HardwareController* hw, ChargeCommand cmd) {
    assert(hw != NULL);
    switch (hw->state) {
        case ZCS_IDLE:
            if ((cmd.active_phases == PHASES_1 || cmd.active_phases == PHASES_3) && 
                 cmd.active_phases != hw->actual_hardware_phases) {
                
                // Phasenwechsel einleiten (Stromabschaltung zuerst)
                if (Set_PWM_Output_Amps(0)) {
                    hw->state = ZCS_WAIT_CAR_STOP;
                    hw->timer = 0;
                    hw->pending_phases = cmd.active_phases;
                    hw->retry_counter = 0;
                    hw->hardware_fault = false; // Heilung
                } else {
                    hw->hardware_fault = true;
                }
            } else {
                // Regelbetrieb
                if (Set_PWM_Output_Amps(cmd.target_current_da)) {
                    hw->hardware_fault = false;
                    hw->retry_counter = 0; // Auch hier Zähler zurücksetzen
                } else {
                    hw->hardware_fault = true;
                }
            }
            break;

        case ZCS_WAIT_CAR_STOP:
            hw->timer++;
            if (hw->timer >= CAR_RAMPDOWN_TICKS) {
                if (Set_Relays(hw->pending_phases)) {
                    hw->actual_hardware_phases = hw->pending_phases;
                    hw->state = ZCS_WAIT_RELAY_SETTLE;
                    hw->timer = 0;
                    hw->hardware_fault = false;
                } else {
                    if (++hw->retry_counter >= MAX_HW_RETRIES) {
                        hw->hardware_fault = true;
                    }
                }
            }
            break;

        case ZCS_WAIT_RELAY_SETTLE:
            hw->timer++;
            if (hw->timer >= RELAY_SETTLE_TICKS) {
                hw->state = ZCS_IDLE;
                hw->retry_counter = 0;
            }
            break;
    }
}

// ==========================================================================
// NEU: Timer-Interrupt für 10-Hz-Takt
// ==========================================================================
volatile bool timer_flag = false;   // Flag wird im Interrupt gesetzt

ISR(TIMER1_COMPA_vect) {
    timer_flag = true;
}

/* ==========================================================================
   7. MAIN LOOP (Zentrale Ablaufsteuerung)
   ========================================================================== */

FIR_FilterState    pv_filter;
PhaseHysteresis    phase_hysteresis;
HardwareController hw_controller;

int main(void) {
    // 1. Hardware-Sicherheit (Watchdog 2s)
    wdt_enable(WDTO_2S);
    
    // 2. Modul-Initialisierung
    FIR_Init(&pv_filter);
    Hysteresis_Init(&phase_hysteresis);
    HardwareController_Init(&hw_controller);

    uint8_t tick_counter_1sec = 0;
    uint8_t sensor_error_counter = 0;
    ChargeCommand current_cmd = {PHASES_1, 0}; 
    int16_t smooth_pv = 0; 
    int16_t last_valid_pv = 0;

    // NEU: Timer1 für 10 Hz konfigurieren (CTC-Modus, Vorteiler 64)
    // Annahme: Taktfrequenz 16 MHz. Bei abweichendem Takt muss OCR1A angepasst werden.
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, Vorteiler 64
    OCR1A = 24999;  // (16000000 / 64) / 10 - 1 = 24999
    TIMSK1 = (1 << OCIE1A); // Interrupt bei Compare Match A aktivieren
    sei(); // Globale Interrupts einschalten

    // 3. Unendliche Steuerungsschleife (nicht-blockierend)
    while(1) {
        // Nur wenn der Timer-Interrupt ein neues 100-ms-Intervall signalisiert
        if (timer_flag) {
            timer_flag = false; // Flag zurücksetzen

            // --- Beginn der Regelung (wie bisher, jedoch ohne _delay_ms) ---
            int16_t raw_pv = Read_PV_Surplus_Watts();
            
            // Sensor-Validierung mit Ersatzwert bei Fehlern
            if (Validate_PV_Value(raw_pv, &sensor_error_counter)) {
                last_valid_pv = raw_pv;
            } else {
                last_valid_pv = 0;
            }
            
            if (sensor_error_counter >= PV_MAX_ERRORS || hw_controller.hardware_fault) {
                current_cmd.active_phases = PHASES_OFF;
                current_cmd.target_current_da = 0;
            } else {
                FIR_Process(&pv_filter, last_valid_pv, fir_coeffs, &smooth_pv);
                
                tick_counter_1sec++;
                if (tick_counter_1sec >= 10) {
                    tick_counter_1sec = 0;
                    PhaseMode target_phases = Hysteresis_Process(&phase_hysteresis, (int32_t)smooth_pv);
                    current_cmd = Calculate_Charge_Power((int32_t)smooth_pv, target_phases);
                }
            }

            Execute_Hardware_Command(&hw_controller, current_cmd);
            // --- Ende der Regelung ---

            // NEU: Watchdog-Reset NUR nach erfolgreicher Regelung
            // Falls die Regelung in einer Endlosschleife hängen bleibt,
            // wird der Watchdog nicht mehr bedient und löst nach 2 Sekunden einen Reset aus.
            wdt_reset();
        }

        // Platz für andere Aufgaben (z.B. UART-Kommunikation, Tasterabfrage, …)
        // Diese werden hier in jedem Schleifendurchlauf ausgeführt, unabhängig vom Timer-Flag.
        // Sie dürfen jedoch nicht so lange blockieren, dass der Watchdog zwischendurch auslösen würde.
        // (Die maximale Blockadezeit muss kleiner als 2 s sein, oder man setzt auch hier wdt_reset().)
    }

    return 0; // Nie erreicht
}
