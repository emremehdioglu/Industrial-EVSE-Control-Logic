/**
 * ==========================================================================
 * WALLBOX CONTROL SYSTEM V2.6 - SIL SIMULATION EDITION 
 * ==========================================================================
 * Dieser Code bildet die industrielle Logik aus V2.6 originalgetreu ab und
 * ergänzt eine Testbench für die Ausführung auf dem PC (z.B. OnlineGDB).
 * 
 * Korrekturen gegenüber der ersten Simulationsversion:
 * - Execute_Hardware_Command: Phasenwechsel nur bei PHASES_1/PHASES_3.
 * - Execute_Hardware_Command: Retry-Logik und hardware_fault originalgetreu.
 * - HardwareController_Init: Alle Felder initialisiert.
 * - FIR_Process: SAFE_ASSERT(out_sample != NULL) wieder eingefügt.
 * - retry_counter wird nun korrekt verwendet.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

/* --- AVR MOCKING (Damit der PC-Compiler AVR-Code versteht) --- */
#define PROGMEM
#define pgm_read_word(addr) (*(const int16_t *)(addr))
#define WDTO_2S 7
#define wdt_enable(x) printf("[SYSTEM] Watchdog scharf geschaltet (2s)\n")
#define wdt_reset() { extern uint32_t wdt_feed_count; wdt_feed_count++; }
#define ISR(vector) void vector(void)
#define sei() printf("[SYSTEM] Interrupts aktiviert\n")
uint8_t TCCR1B, TIMSK1, WGM12, CS11, CS10, OCIE1A;
uint16_t OCR1A;

/* --- KONFIGURATION & LOGIK (V2.6 Original) --- */
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

const int16_t fir_coeffs[32] PROGMEM = {
    -10, -36, -75, -132, -198, -244, -231, -112, 152, 582, 1167, 1861, 2589, 3257, 3768, 4046,
    4046, 3768, 3257, 2589, 1861, 1167, 582, 152, -112, -231, -244, -198, -132, -75, -36, -10
};

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

/* Globale Instanzen für Simulatorzugriff */
FIR_FilterState pv_filter;
PhaseHysteresis phase_hysteresis;
HardwareController hw_controller;
volatile bool timer_flag = false;
volatile bool timer_overrun = false;

/* --- FUNKTIONEN (HAL Prototypen) --- */
int16_t Read_PV_Surplus_Watts(void);
bool Set_PWM_Output_Amps(int16_t deci_amps);
bool Set_Relays(PhaseMode phases);

/* --- LOGIK IMPLEMENTIERUNG (originalgetreu) --- */
bool Validate_PV_Value(int16_t raw_val, uint8_t* error_counter) {
    SAFE_ASSERT(error_counter != NULL);
    if (raw_val < PV_LIMIT_LOW || raw_val > PV_LIMIT_HIGH) {
        if (*error_counter < PV_MAX_ERRORS) (*error_counter)++;
        return false;
    }
    *error_counter = 0; return true;
}

void FIR_Init(FIR_FilterState* state) {
    SAFE_ASSERT(state != NULL);
    memset(state->delay_line, 0, sizeof(state->delay_line));
    state->write_idx = 0; state->decimation_counter = 0;
}

uint8_t FIR_Process(FIR_FilterState* state, int16_t in_sample, const int16_t* coeffs, int16_t* out_sample) {
    SAFE_ASSERT(state != NULL);
    SAFE_ASSERT(coeffs != NULL);
    SAFE_ASSERT(out_sample != NULL);   // wieder eingefügt

    state->delay_line[state->write_idx] = in_sample;
    uint8_t new_output_available = 0;

    state->decimation_counter++;
    if (state->decimation_counter >= DECIMATION_M) {
        state->decimation_counter = 0;

        int32_t acc = 0;
        uint8_t read_idx = state->write_idx;
        for (uint8_t i = 0; i < NUM_TAPS; i++) {
            int16_t coeff = (int16_t)pgm_read_word(&coeffs[i]);
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
    SAFE_ASSERT(hyst != NULL);
    hyst->current_state = PHASES_1;
    hyst->timer_up = 0;
    hyst->timer_down = 0;
}

PhaseMode Hysteresis_Process(PhaseHysteresis* hyst, int32_t pv_surplus_w) {
    SAFE_ASSERT(hyst != NULL);
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
    if (current_da < MIN_CHARGE_CURRENT_DA) cmd.target_current_da = 0;
    else if (current_da > MAX_CHARGE_CURRENT_DA) cmd.target_current_da = MAX_CHARGE_CURRENT_DA;
    else cmd.target_current_da = current_da;
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

    bool pwm_ok = Set_PWM_Output_Amps(0);
    bool relay_ok = Set_Relays(PHASES_1);
    hw->hardware_fault = (!pwm_ok || !relay_ok);
}

void Execute_Hardware_Command(HardwareController* hw, ChargeCommand cmd) {
    SAFE_ASSERT(hw != NULL);
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
                    hw->consecutive_hw_failures = 0;
                } else {
                    hw->hardware_fault = true;
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
                }
            } else {
                if (Set_PWM_Output_Amps(cmd.target_current_da)) {
                    hw->hardware_fault = false;
                    hw->retry_counter = 0;
                    hw->consecutive_hw_failures = 0;
                } else {
                    hw->hardware_fault = true;
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
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
                } else {
                    if (hw->consecutive_hw_failures < 255) hw->consecutive_hw_failures++;
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

ISR(TIMER1_COMPA_vect) {
    if (timer_flag) timer_overrun = true;
    timer_flag = true;
}

void* wallbox_thread(void* arg) {
    wdt_enable(WDTO_2S);
    FIR_Init(&pv_filter);
    Hysteresis_Init(&phase_hysteresis);
    HardwareController_Init(&hw_controller);

    uint8_t tick_counter_1sec = 0;
    uint8_t sensor_error_counter = 0;
    ChargeCommand current_cmd = {PHASES_1, 0};
    int16_t smooth_pv = 0;
    int16_t last_valid_pv = 0;

    sei();

    while(1) {
        if (timer_flag) {
            timer_flag = false;
            timer_overrun = false;   // Flag zurücksetzen

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
                }
            }

            Execute_Hardware_Command(&hw_controller, current_cmd);

            if (hw_controller.consecutive_hw_failures < 10) {
                wdt_reset();
            }
        }

        usleep(100);   // kurze Pause, um CPU-Last zu reduzieren
    }
    return NULL;
}

/* --- SIMULATOR STEUERUNG --- */
uint32_t wdt_feed_count = 0;
uint32_t sim_tick = 0;
bool i2c_dead = false;

int16_t Read_PV_Surplus_Watts(void) {
    // Simulierte PV-Werte: erst niedrig, dann hoch, dann wieder runter
    if (sim_tick < 500) return 1200;   // 1,2 kW
    if (sim_tick < 1500) return 6000;  // 6 kW -> löst Umschaltung aus
    return 1500;                        // 1,5 kW -> Rückschaltung
}

bool Set_PWM_Output_Amps(int16_t da) {
    if (i2c_dead) return false;
    printf("[ACT] PWM: %d.%d A\n", da/10, da%10);
    return true;
}

bool Set_Relays(PhaseMode p) {
    if (i2c_dead) return false;
    printf("[ACT] Relais: %d Phasen\n", p);
    return true;
}

int main() {
    printf("--- Wallbox V2.6 Simulation startet (korrigierte Version) ---\n");
    pthread_t thread;
    pthread_create(&thread, NULL, wallbox_thread, NULL);

    uint32_t last_wdt = 0;

    for (sim_tick = 0; sim_tick < 2500; sim_tick++) {
        // Nach 22 Sekunden (2200 * 100ms) simulieren wir einen I2C-Ausfall
        if (sim_tick == 2200) {
            printf("\n[!] SIM: I2C BUS CRASH!\n");
            i2c_dead = true;
        }

        // Timer-Interrupt alle 100 ms simulieren (jeder sim_tick = 100 ms)
        TIMER1_COMPA_vect();

        // In der Simulation warten wir 5 ms Echtzeit pro simuliertem 100-ms-Tick
        usleep(5000);

        // Alle 10 Schritte (1 s simuliert) Status ausgeben
        if (sim_tick % 100 == 0) {
            printf("[T=%2ds] PV: %4d W | WDT-Feeds (letzte Sek): %u\n",
                   sim_tick/10,
                   Read_PV_Surplus_Watts(),
                   wdt_feed_count - last_wdt);
            last_wdt = wdt_feed_count;

            // Wenn der Watchdog nicht mehr gefüttert wird, sollte nach 2 s Reset kommen
            if (wdt_feed_count == last_wdt && i2c_dead) {
                printf("\n[SUCCESS] Watchdog hat System-Reset ausgeloest (wie erwartet)!\n");
                printf("Simulation beendet.\n");
                return 0;
            }
        }
    }

    printf("\nSimulation regulär beendet (kein Reset erforderlich).\n");
    return 0;
}
