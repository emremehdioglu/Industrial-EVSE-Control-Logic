/**
 * @file wallbox_simulation.c
 * @brief PC-Simulation der Wallbox-Steuerung (Version 2.0)
 * 
 * Dieser Code simuliert die Logik der AVR-Wallbox-Steuerung auf einem PC.
 * Alle Hardware-Abhängigkeiten wurden durch plattformunabhängige
 * Konstrukte ersetzt. Die Ausgabe erfolgt über printf.
 * 
 * Kompilierung mit gcc: gcc -o wallbox_sim wallbox_simulation.c -lm (falls math verwendet würde, hier nicht nötig)
 * 
 * Hinweis: Die Filterkoeffizienten sind fest eingestellt (Hamming).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <time.h>

/* ==========================================================================
   Ersatz für AVR-spezifische Elemente
   ========================================================================== */
// Da wir auf PC sind, definieren wir PROGMEM als leer und pgm_read_word als direkten Zugriff.
#define PROGMEM
#define pgm_read_word(x) (*(x))

// Watchdog-Makros (auf PC inaktiv)
#define wdt_enable(x)
#define wdt_reset()

// Eigene Delay-Funktion (blockierend, nutzt clock)
static void my_delay_ms(uint32_t ms) {
    clock_t start = clock();
    while ((clock() - start) * 1000 / CLOCKS_PER_SEC < ms) {
        // busy wait
    }
}
#define _delay_ms(ms) my_delay_ms(ms)

/* ==========================================================================
   1. KUNDEN-KONFIGURATION (DSP & LOGIK) - unverändert
   ========================================================================== */
#define FILTER_WINDOW_SELECTION 2   // Hamming

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

/* ==========================================================================
   2. STATISCHE SICHERHEITS-WÄCHTER (unverändert, aber auf PC ohne AVR-Asm)
   ========================================================================== */
_Static_assert(FILTER_WINDOW_SELECTION >= 1 && FILTER_WINDOW_SELECTION <= 4, "Ungueltiges DSP-Fenster gewaehlt");
_Static_assert((NUM_TAPS & TAP_MASK) == 0, "NUM_TAPS muss eine Zweierpotenz sein");
_Static_assert(NUM_TAPS <= 256, "NUM_TAPS ueberschreitet uint8_t Indexbereich");
_Static_assert(DECIMATION_M <= 255, "DECIMATION_M ueberschreitet uint8_t Bereich");
_Static_assert(PV_MAX_ERRORS <= 255, "PV_MAX_ERRORS ueberschreitet uint8_t Bereich");
_Static_assert(DELAY_UP_SECONDS <= 65535, "DELAY_UP_SECONDS ueberschreitet uint16_t Zeitbereich");
_Static_assert(DELAY_DOWN_SECONDS <= 65535, "DELAY_DOWN_SECONDS ueberschreitet uint16_t Zeitbereich");

/* ==========================================================================
   3. DSP-KOEFFIZIENTEN-DATENBANK (jetzt ohne PROGMEM, aber Makro lässt es gleich)
   ========================================================================== */
#if (FILTER_WINDOW_SELECTION == 1)
  const int16_t fir_coeffs[32] = {
      -146, -443, -712, -907, -986, -915, -676, -265, 301, 988, 1748, 2520, 3240, 3845, 4282, 4511,
      4511, 4282, 3845, 3240, 2520, 1748, 988, 301, -265, -676, -915, -986, -907, -712, -443, -146
  };
#elif (FILTER_WINDOW_SELECTION == 2)
  const int16_t fir_coeffs[32] = {
      -10, -36, -75, -132, -198, -244, -231, -112, 152, 582, 1167, 1861, 2589, 3257, 3768, 4046,
      4046, 3768, 3257, 2589, 1861, 1167, 582, 152, -112, -231, -244, -198, -132, -75, -36, -10
  };
#elif (FILTER_WINDOW_SELECTION == 3)
  const int16_t fir_coeffs[32] = {
      0, 0, -2, -9, -23, -44, -61, -40, 70, 335, 808, 1496, 2342, 3215, 3941, 4355,
      4355, 3941, 3215, 2342, 1496, 808, 335, 70, -40, -61, -44, -23, -9, -2, 0, 0
  };
#elif (FILTER_WINDOW_SELECTION == 4)
  const int16_t fir_coeffs[32] = {
      0, 0, -1, -4, -12, -26, -40, -29, 56, 286, 728, 1411, 2288, 3224, 4022, 4482,
      4482, 4022, 3224, 2288, 1411, 728, 286, 56, -29, -40, -26, -12, -4, -1, 0, 0
  };
#endif

/* ==========================================================================
   4. DATENTYPEN & SYSTEM-ZUSTÄNDE (unverändert)
   ========================================================================== */
typedef enum { PHASES_OFF = 0, PHASES_1 = 1, PHASES_3 = 3 } PhaseMode;

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

typedef enum { ZCS_IDLE = 0, ZCS_WAIT_CAR_STOP, ZCS_WAIT_RELAY_SETTLE } ZCS_State;

typedef struct {
    ZCS_State state;
    PhaseMode actual_hardware_phases;
    PhaseMode pending_phases;
    uint16_t timer;
    bool hardware_fault;
    uint8_t retry_counter;
} HardwareController;

/* ==========================================================================
   5. SIMULIERTE HARDWARE-ABSTRAKTION (Stubs mit Ausgabe)
   ========================================================================== */

// Simulierter PV-Sensor: Liefert Werte aus einer festen Sequenz.
// Für interaktive Simulation könnte man hier scanf einbauen.
static int16_t test_pv_values[] = {1000, 2000, 3000, 4000, 5000, 4500, 4000, 3500, 3000, 2500, 2000, 1500, 1000};
static int test_idx = 0;

int16_t Read_PV_Surplus_Watts(void) {
    // Einfache zyklische Testsequenz
    int16_t val = test_pv_values[test_idx];
    test_idx = (test_idx + 1) % (sizeof(test_pv_values)/sizeof(test_pv_values[0]));
    printf("PV raw: %d W\n", val);
    return val;
}

bool Set_PWM_Output_Amps(int16_t deci_amps) {
    printf("PWM set to %d dA (%d.%d A)\n", deci_amps, deci_amps/10, deci_amps%10);
    return true;  // immer Erfolg
}

bool Set_Relays(PhaseMode phases) {
    const char* phase_str = (phases == PHASES_OFF) ? "OFF" : (phases == PHASES_1) ? "1-phasig" : "3-phasig";
    printf("Relays set to %s\n", phase_str);
    return true;  // immer Erfolg
}

/* ==========================================================================
   6. IMPLEMENTIERUNG DER LOGIK-BAUSTEINE (unverändert vom Original)
   ========================================================================== */

bool Validate_PV_Value(int16_t raw_val, uint8_t* error_counter) {
    assert(error_counter != NULL);
    if (raw_val < PV_LIMIT_LOW || raw_val > PV_LIMIT_HIGH) {
        if (*error_counter < PV_MAX_ERRORS) {
            (*error_counter)++;
        }
        return false;
    }
    *error_counter = 0;
    return true;
}

void FIR_Init(FIR_FilterState* state) {
    assert(state != NULL);
    memset(state->delay_line, 0, sizeof(state->delay_line));
    state->write_idx = 0;
    state->decimation_counter = 0;
}

uint8_t FIR_Process(FIR_FilterState* state, int16_t in_sample, const int16_t* coeffs, int16_t* out_sample) {
    assert(state != NULL);
    assert(coeffs != NULL);

    state->delay_line[state->write_idx] = in_sample;
    uint8_t new_output_available = 0;

    state->decimation_counter++;
    if (state->decimation_counter >= DECIMATION_M) {
        state->decimation_counter = 0;
        assert(out_sample != NULL);

        int32_t acc = 0;
        uint8_t read_idx = state->write_idx;
        for (uint8_t i = 0; i < NUM_TAPS; i++) {
            int16_t coeff = (int16_t)pgm_read_word(&coeffs[i]); // Makro, das auf PC direkt liest
            acc += (int32_t)coeff * state->delay_line[read_idx];
            read_idx = (read_idx - 1) & TAP_MASK;
        }

        acc += 16384;
        int32_t result = (acc >> 15);

        if (result > 32767) result = 32767;
        else if (result < -32768) result = -32768;

        *out_sample = (int16_t)result;
        new_output_available = 1;
    }

    state->write_idx = (state->write_idx + 1) & TAP_MASK;
    return new_output_available;
}

void Hysteresis_Init(PhaseHysteresis* hyst) {
    assert(hyst != NULL);
    hyst->current_state = PHASES_1;
    hyst->timer_up = 0;
    hyst->timer_down = 0;
}

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

ChargeCommand Calculate_Charge_Power(int32_t pv_surplus_w, PhaseMode active_phases) {
    ChargeCommand cmd = {active_phases, 0};

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

void HardwareController_Init(HardwareController* hw) {
    assert(hw != NULL);
    hw->state = ZCS_IDLE;
    hw->actual_hardware_phases = PHASES_1;
    hw->pending_phases = PHASES_1;
    hw->timer = 0;
    hw->retry_counter = 0;

    // Simulierter Hardware-Check
    bool pwm_ok = Set_PWM_Output_Amps(0);
    bool relay_ok = Set_Relays(PHASES_1);
    hw->hardware_fault = (!pwm_ok || !relay_ok);
}

void Execute_Hardware_Command(HardwareController* hw, ChargeCommand cmd) {
    assert(hw != NULL);
    switch (hw->state) {
        case ZCS_IDLE:
            if ((cmd.active_phases == PHASES_1 || cmd.active_phases == PHASES_3) &&
                 cmd.active_phases != hw->actual_hardware_phases) {

                if (Set_PWM_Output_Amps(0)) {
                    hw->state = ZCS_WAIT_CAR_STOP;
                    hw->timer = 0;
                    hw->pending_phases = cmd.active_phases;
                    hw->retry_counter = 0;
                    hw->hardware_fault = false;
                } else {
                    hw->hardware_fault = true;
                }
            } else {
                if (Set_PWM_Output_Amps(cmd.target_current_da)) {
                    hw->hardware_fault = false;
                    hw->retry_counter = 0;
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

/* ==========================================================================
   7. SIMULIERTE MAIN-FUNKTION (mit begrenzter Laufzeit)
   ========================================================================== */

FIR_FilterState    pv_filter;
PhaseHysteresis    phase_hysteresis;
HardwareController hw_controller;

int main(void) {
    printf("=== Wallbox-Simulation gestartet ===\n");

    FIR_Init(&pv_filter);
    Hysteresis_Init(&phase_hysteresis);
    HardwareController_Init(&hw_controller);

    uint8_t tick_counter_1sec = 0;
    uint8_t sensor_error_counter = 0;
    ChargeCommand current_cmd = {PHASES_1, 0};
    int16_t smooth_pv = 0;
    int16_t last_valid_pv = 0;

    // Anzahl der Simulationsschritte (hier 200 ≈ 20 Sekunden, da jeder Schritt 100 ms)
    const int MAX_STEPS = 200;
    for (int step = 0; step < MAX_STEPS; step++) {
        printf("\n--- Schritt %d ---\n", step);

        int16_t raw_pv = Read_PV_Surplus_Watts();

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
                printf("Hysterese: Zielphasen = %s\n", 
                       (target_phases == PHASES_1) ? "1" : (target_phases == PHASES_3) ? "3" : "OFF");
            }
        }

        Execute_Hardware_Command(&hw_controller, current_cmd);

        // Zustandsinformationen ausgeben
        printf("Filter smooth_pv = %d W\n", smooth_pv);
        printf("Hardware State = %d, actual phases = %s, fault = %d\n",
               hw_controller.state,
               (hw_controller.actual_hardware_phases == PHASES_1) ? "1" : 
               (hw_controller.actual_hardware_phases == PHASES_3) ? "3" : "OFF",
               hw_controller.hardware_fault);
        printf("Sensor error counter = %d\n", sensor_error_counter);

        // Simuliere 100 ms
        _delay_ms(100);
        wdt_reset(); // Makro, tut nichts
    }

    printf("=== Simulation beendet ===\n");
    return 0;
}
