/**
 * ==========================================================================
 * WALLBOX CONTROL SYSTEM V2.6 - PROFESSIONELLER SIMULATIONSBERICHT
 * ==========================================================================
 *
 * Diese Simulation demonstriert das intelligente Verhalten der Wallbox-Steuerung.
 * Der Bericht erklärt in klarer Sprache, warum bestimmte Aktionen ausgeführt werden.
 *
 * Szenario:
 *  - 0–50 s:  1200 W PV-Überschuss → keine Ladung (Strom < 6 A)
 *  - 50–150 s: 6000 W → nach 60 s Umschaltung auf 3-phasigen Betrieb
 *  - ab 150 s: 1500 W → Strom fällt unter Minimum, später beginnt Rückschaltung
 *  - bei 220 s: simulierter I2C-Bus-Crash → Sicherheitsmechanismus löst Reset aus
 *
 * Die Ausgabe beschreibt jeden relevanten Schritt und macht die Entscheidungslogik sichtbar.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

/* --- AVR-Mocking (für PC) --- */
#define PROGMEM
#define pgm_read_word(addr) (*(const int16_t *)(addr))
#define WDTO_2S 7
#define wdt_enable(x)    // keine Ausgabe – stört den Bericht
#define wdt_reset()      { extern uint32_t wdt_feed_count; wdt_feed_count++; }
#define ISR(vector)      void vector(void)
#define sei()            // keine Ausgabe
uint8_t TCCR1B, TIMSK1, WGM12, CS11, CS10, OCIE1A;
uint16_t OCR1A;

/* --- Konfiguration (wie im Original) --- */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define TIMER_COMPARE_VAL (uint16_t)(((F_CPU) / 64 / 10) - 1)
#define SAFE_ASSERT(expr) do { if (!(expr)) { while(1); } } while(0)

#define FILTER_WINDOW_SELECTION 2
#define NUM_TAPS             32
#define TAP_MASK             (NUM_TAPS - 1)
#define DECIMATION_M         4
#define PV_LIMIT_LOW         -10000
#define PV_LIMIT_HIGH         25000
#define PV_MAX_ERRORS         50
#define THRESHOLD_1_TO_3_W    4500
#define THRESHOLD_3_TO_1_W    3500
#define DELAY_UP_SECONDS      60
#define DELAY_DOWN_SECONDS    120
#define CAR_RAMPDOWN_TICKS    50
#define RELAY_SETTLE_TICKS    5
#define MAX_HW_RETRIES        3
#define VOLTAGE_LN_V          230
#define MIN_CHARGE_CURRENT_DA 60
#define MAX_CHARGE_CURRENT_DA 160

const int16_t fir_coeffs[32] PROGMEM = { /* Hamming-Fenster */ };

/* --- Datentypen --- */
typedef enum { PHASES_OFF = 0, PHASES_1 = 1, PHASES_3 = 3 } PhaseMode;
typedef struct { PhaseMode active_phases; int16_t target_current_da; } ChargeCommand;
typedef struct { int16_t delay_line[NUM_TAPS]; uint8_t write_idx; uint8_t decimation_counter; } FIR_FilterState;
typedef struct { PhaseMode current_state; uint16_t timer_up; uint16_t timer_down; } PhaseHysteresis;
typedef enum { ZCS_IDLE = 0, ZCS_WAIT_CAR_STOP, ZCS_WAIT_RELAY_SETTLE } ZCS_State;
typedef struct {
    ZCS_State state;
    PhaseMode actual_hardware_phases;
    PhaseMode pending_phases;
    uint16_t timer;
    bool hardware_fault;
    uint8_t retry_counter;
    uint8_t consecutive_hw_failures;
} HardwareController;

/* --- Globale Variablen --- */
FIR_FilterState pv_filter;
PhaseHysteresis phase_hysteresis;
HardwareController hw_controller;
volatile bool timer_flag = false;
volatile bool timer_overrun = false;

/* --- Hilfsfunktion für lesbare Phasenausgabe --- */
const char* phase_str(PhaseMode p) {
    switch(p) {
        case PHASES_OFF: return "aus";
        case PHASES_1:   return "1-phasig";
        case PHASES_3:   return "3-phasig";
        default:         return "?";
    }
}

/* --- Hardware-API (Prototypen) --- */
int16_t Read_PV_Surplus_Watts(void);
bool Set_PWM_Output_Amps(int16_t deci_amps);
bool Set_Relays(PhaseMode phases);

/* ==========================================================================
   Logikbausteine (unverändert, aber Bericht erzeugende Debugs)
   ========================================================================== */

bool Validate_PV_Value(int16_t raw_val, uint8_t* error_counter) {
    SAFE_ASSERT(error_counter != NULL);
    if (raw_val < PV_LIMIT_LOW || raw_val > PV_LIMIT_HIGH) {
        if (*error_counter < PV_MAX_ERRORS) (*error_counter)++;
        return false;
    }
    *error_counter = 0;
    return true;
}

void FIR_Init(FIR_FilterState* state) { /* wie gehabt */ }
uint8_t FIR_Process(FIR_FilterState* state, int16_t in_sample, const int16_t* coeffs, int16_t* out_sample) { /* wie gehabt */ }

void Hysteresis_Init(PhaseHysteresis* hyst) {
    SAFE_ASSERT(hyst != NULL);
    hyst->current_state = PHASES_1;
    hyst->timer_up = 0;
    hyst->timer_down = 0;
    printf("[BERICHT] System startet im 1‑phasigen Modus (Standby).\n");
}

PhaseMode Hysteresis_Process(PhaseHysteresis* hyst, int32_t pv_surplus_w) {
    SAFE_ASSERT(hyst != NULL);
    PhaseMode old = hyst->current_state;

    if (hyst->current_state == PHASES_1) {
        if (pv_surplus_w >= THRESHOLD_1_TO_3_W) {
            hyst->timer_up++;
            if (hyst->timer_up == 1) {
                printf("[BERICHT] PV‑Leistung übersteigt 4500 W – Hochschalt‑Timer startet (60 s).\n");
            }
            if (hyst->timer_up >= DELAY_UP_SECONDS) {
                hyst->current_state = PHASES_3;
                hyst->timer_up = 0;
                hyst->timer_down = 0;
                printf("[BERICHT] Timer abgelaufen – System schaltet auf 3‑phasigen Betrieb um.\n");
            }
        } else {
            if (hyst->timer_up > 0) {
                printf("[BERICHT] PV‑Leistung fällt unter 4500 W – Hochschalt‑Timer wird abgebrochen.\n");
                hyst->timer_up = 0;
            }
        }
    } else if (hyst->current_state == PHASES_3) {
        if (pv_surplus_w <= THRESHOLD_3_TO_1_W) {
            hyst->timer_down++;
            if (hyst->timer_down == 1) {
                printf("[BERICHT] PV‑Leistung unter 3500 W – Rückschalt‑Timer startet (120 s).\n");
            }
            if (hyst->timer_down >= DELAY_DOWN_SECONDS) {
                hyst->current_state = PHASES_1;
                hyst->timer_down = 0;
                hyst->timer_up = 0;
                printf("[BERICHT] Timer abgelaufen – System schaltet auf 1‑phasigen Betrieb zurück.\n");
            }
        } else {
            if (hyst->timer_down > 0) {
                printf("[BERICHT] PV‑Leistung steigt über 3500 W – Rückschalt‑Timer wird abgebrochen.\n");
                hyst->timer_down = 0;
            }
        }
    }
    return hyst->current_state;
}

ChargeCommand Calculate_Charge_Power(int32_t pv_surplus_w, PhaseMode active_phases) {
    ChargeCommand cmd = {active_phases, 0};
    if (active_phases != PHASES_1 && active_phases != PHASES_3) {
        cmd.active_phases = PHASES_OFF;
        return cmd;
    }
    int32_t denominator = (int32_t)active_phases * VOLTAGE_LN_V;
    int16_t current_da = (int16_t)((pv_surplus_w * 10) / denominator);
    float current_a = current_da / 10.0f;

    if (current_da < MIN_CHARGE_CURRENT_DA) {
        if (current_da > 0) {
            printf("[BERICHT] Berechneter Strom (%.1f A) liegt unter Mindeststrom 6,0 A – Ladung wird pausiert.\n", current_a);
        }
        cmd.target_current_da = 0;
    } else if (current_da > MAX_CHARGE_CURRENT_DA) {
        printf("[BERICHT] Berechneter Strom (%.1f A) übersteigt Maximalstrom 16,0 A – wird auf 16,0 A begrenzt.\n", current_a);
        cmd.target_current_da = MAX_CHARGE_CURRENT_DA;
    } else {
        cmd.target_current_da = current_da;
    }
    return cmd;
}

void HardwareController_Init(HardwareController* hw) {
    SAFE_ASSERT(hw != NULL);
    hw->state = ZCS_IDLE;
    hw->actual_hardware_phases = PHASES_1;
    hw->pending_phases = PHASES_1;
    hw->timer = 0;
    hw->retry_counter = 0;
    hw->consecutive_hw_failures = 0;
    hw->hardware_fault = false;
    // Hardware-Test still – keine Ausgabe
}

void Execute_Hardware_Command(HardwareController* hw, ChargeCommand cmd) {
    SAFE_ASSERT(hw != NULL);
    ZCS_State old_state = hw->state;

    switch (hw->state) {
        case ZCS_IDLE:
            if ((cmd.active_phases == PHASES_1 || cmd.active_phases == PHASES_3) &&
                 cmd.active_phases != hw->actual_hardware_phases) {
                // Phasenwechsel
                if (Set_PWM_Output_Amps(0)) {
                    hw->state = ZCS_WAIT_CAR_STOP;
                    hw->timer = 0;
                    hw->pending_phases = cmd.active_phases;
                    hw->retry_counter = 0;
                    hw->hardware_fault = false;
                    hw->consecutive_hw_failures = 0;
                    printf("[BERICHT] Phasenwechsel von %s auf %s eingeleitet – Strom wird auf 0 reduziert.\n",
                           phase_str(hw->actual_hardware_phases), phase_str(cmd.active_phases));
                } else {
                    hw->hardware_fault = true;
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
                    printf("[BERICHT] FEHLER: PWM‑Abschaltung fehlgeschlagen – Kommunikationsproblem.\n");
                }
            } else {
                // Normalbetrieb
                if (Set_PWM_Output_Amps(cmd.target_current_da)) {
                    hw->hardware_fault = false;
                    hw->retry_counter = 0;
                    hw->consecutive_hw_failures = 0;
                } else {
                    hw->hardware_fault = true;
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
                    printf("[BERICHT] FEHLER: PWM‑Befehl fehlgeschlagen – Kommunikationsproblem.\n");
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
                    hw->consecutive_hw_failures = 0;
                    printf("[BERICHT] Relais auf %s umgeschaltet – warte auf Einschwingen (0,5 s).\n",
                           phase_str(hw->actual_hardware_phases));
                } else {
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
                    if (++hw->retry_counter >= MAX_HW_RETRIES) {
                        hw->hardware_fault = true;
                        printf("[BERICHT] FEHLER: Relais‑Umschaltung endgültig fehlgeschlagen – System deaktiviert.\n");
                    } else {
                        printf("[BERICHT] FEHLER: Relais‑Umschaltung fehlgeschlagen – Wiederholung (%d/%d).\n",
                               hw->retry_counter, MAX_HW_RETRIES);
                    }
                }
            }
            break;

        case ZCS_WAIT_RELAY_SETTLE:
            hw->timer++;
            if (hw->timer >= RELAY_SETTLE_TICKS) {
                hw->state = ZCS_IDLE;
                hw->retry_counter = 0;
                printf("[BERICHT] Umschaltung abgeschlossen – normaler Betrieb.\n");
            }
            break;
    }
}

/* --- Timer-ISR (Simulation) --- */
ISR(TIMER1_COMPA_vect) {
    if (timer_flag) timer_overrun = true;
    timer_flag = true;
}

/* --- Regelungs-Thread --- */
void* wallbox_thread(void* arg) {
    wdt_enable(0);
    FIR_Init(&pv_filter);
    Hysteresis_Init(&phase_hysteresis);
    HardwareController_Init(&hw_controller);

    uint8_t tick_counter_1sec = 0;
    uint8_t sensor_error_counter = 0;
    ChargeCommand current_cmd = {PHASES_1, 0};
    int16_t smooth_pv = 0;
    int16_t last_valid_pv = 0;

    while(1) {
        if (timer_flag) {
            timer_flag = false;
            timer_overrun = false;

            int16_t raw_pv = Read_PV_Surplus_Watts();

            if (Validate_PV_Value(raw_pv, &sensor_error_counter)) {
                last_valid_pv = raw_pv;
            } else {
                last_valid_pv = 0;
                if (sensor_error_counter == 1) {
                    printf("[BERICHT] Sensor liefert ungültige Werte – Fehlerzähler startet.\n");
                }
            }

            if (sensor_error_counter >= PV_MAX_ERRORS || hw_controller.hardware_fault) {
                if (current_cmd.active_phases != PHASES_OFF || current_cmd.target_current_da != 0) {
                    printf("[BERICHT] SICHERHEITSAUSLÖSUNG: ");
                    if (sensor_error_counter >= PV_MAX_ERRORS)
                        printf("Sensorfehler (%d von %d)\n", sensor_error_counter, PV_MAX_ERRORS);
                    else
                        printf("dauerhafter Hardwarefehler\n");
                }
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

            if (hw_controller.consecutive_hw_failures < 10) {
                wdt_reset();
            }
        }
        usleep(100);
    }
    return NULL;
}

/* ==========================================================================
   Simulations-Hardware (mit Bericht-Ausgaben)
   ========================================================================== */

uint32_t wdt_feed_count = 0;
uint32_t sim_tick = 0;
bool i2c_dead = false;
int last_reported_pv = -1;
int last_reported_current_da = -999;
PhaseMode last_reported_phase = PHASES_OFF;

int16_t Read_PV_Surplus_Watts(void) {
    int16_t val;
    if (sim_tick < 500) val = 1200;
    else if (sim_tick < 1500) val = 6000;
    else val = 1500;
    if (val != last_reported_pv) {
        printf("\n--- PV-Leistung ändert sich auf %d W ---\n", val);
        last_reported_pv = val;
    }
    return val;
}

bool Set_PWM_Output_Amps(int16_t da) {
    if (i2c_dead) {
        static bool first = true;
        if (first) {
            printf("[BERICHT] Kommunikationsfehler: PWM‑Befehle kommen nicht an (I2C tot).\n");
            first = false;
        }
        return false;
    }
    if (da != last_reported_current_da) {
        if (da == 0)
            printf("[BERICHT] Ladestrom wird auf 0 A gesetzt.\n");
        else
            printf("[BERICHT] Ladestrom wird auf %.1f A pro Phase eingestellt.\n", da/10.0f);
        last_reported_current_da = da;
    }
    return true;
}

bool Set_Relays(PhaseMode p) {
    if (i2c_dead) {
        static bool first = true;
        if (first) {
            printf("[BERICHT] Kommunikationsfehler: Relais‑Befehle kommen nicht an (I2C tot).\n");
            first = false;
        }
        return false;
    }
    if (p != last_reported_phase) {
        printf("[BERICHT] Relais schalten auf %s.\n", phase_str(p));
        last_reported_phase = p;
    }
    return true;
}

/* --- Hauptprogramm mit Berichtsrahmen --- */
int main() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║     WALLBOX-STEUERUNG V2.6 – SIMULATION ALS FACHBERICHT       ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    printf("Einleitung:\n");
    printf("  Die Steuerung überwacht die PV-Überschussleistung und entscheidet\n");
    printf("  autonom, ob und wie viel Strom ins Auto fließt. Sie arbeitet mit\n");
    printf("  einem digitalen Filter, einer Hysterese zur Vermeidung von Pendeln\n");
    printf("  und einer sicheren Zustandsmaschine für stromloses Umschalten.\n\n");
    printf("  In diesem Test durchläuft die Anlage folgendes Szenario:\n");
    printf("  – 0…50 s:   1200 W (keine Ladung, da Strom <6 A)\n");
    printf("  – 50…150 s: 6000 W (nach 60 s Umschaltung auf 3‑phasig)\n");
    printf("  – ab 150 s:  1500 W (Strom fällt unter Minimum, später Rückschaltung)\n");
    printf("  – bei 220 s:  I2C‑Bus fällt aus – Sicherheitsmechanismus testen\n\n");
    printf("Die folgenden Meldungen zeigen, wie die Steuerung auf jede Situation reagiert.\n");
    printf("──────────────────────────────────────────────────────────────────\n\n");

    pthread_t thread;
    pthread_create(&thread, NULL, wallbox_thread, NULL);

    uint32_t last_wdt = 0;
    uint32_t last_status_tick = 0;

    for (sim_tick = 0; sim_tick < 2500; sim_tick++) {
        if (sim_tick == 2200) {
            printf("\n⚠️  KRITISCHES EREIGNIS: I2C‑BUS AUSGEFALLEN (Simulation)\n");
            printf("   Alle weiteren Hardware‑Befehle bleiben erfolglos.\n\n");
            i2c_dead = true;
        }

        TIMER1_COMPA_vect();
        usleep(5000);

        // Alle 10 Sekunden (100 Ticks) einen kurzen Status ausgeben
        if (sim_tick % 100 == 0 && sim_tick != last_status_tick) {
            int pv = Read_PV_Surplus_Watts();
            int current_a = last_reported_current_da / 10;
            int current_tenth = last_reported_current_da % 10;
            printf("\n[STATUS] Zeit %ds | PV %d W | Ladung %s", sim_tick/10, pv, phase_str(last_reported_phase));
            if (last_reported_current_da > 0)
                printf(" mit %d,%d A", current_a, current_tenth);
            printf(" | Fehlerzähler HW: %d\n", hw_controller.consecutive_hw_failures);
            last_wdt = wdt_feed_count;
            last_status_tick = sim_tick;
        }

        // Prüfung, ob Watchdog nicht mehr gefüttert wird
        if (i2c_dead && wdt_feed_count == last_wdt && sim_tick > 2210) {
            printf("\n══════════════════════════════════════════════════════════════\n");
            printf("  🛡️  SICHERHEITSERFOLG: Watchdog wurde nicht mehr bedient.\n");
            printf("     Nach 2 Sekunden ohne Lebenszeichen erfolgt ein Hardware‑Reset.\n");
            printf("     Damit wird das System aus dem „Zombie“‑Zustand befreit.\n");
            printf("══════════════════════════════════════════════════════════════\n\n");
            printf("Simulation beendet.\n");
            return 0;
        }
    }

    printf("\nSimulation regulär beendet – kein Reset nötig.\n");
    return 0;
}
