import numpy as np
from scipy import signal

# --- Konfiguration ---
NUM_TAPS = 32
CUTOFF_NORM = 0.125 # Normierte Grenzfrequenz (0.0 bis 1.0, wobei 1.0 der Nyquist-Frequenz entspricht)

# Die geforderten Fenster-Funktionen
# Für Kaiser benötigen wir einen Beta-Parameter. 14.0 gibt eine exzellente Sperrdämpfung.
windows = {
    "Boxcar": "boxcar",               # Rechteckfenster
    "Hamming": "hamming",
    "Blackman-Harris": "blackmanharris",
    "Kaiser": ("kaiser", 14.0) 
}

print("// --- FIR Koeffizienten (Automatisch generiert) ---")
print("// Format: Q15, Zielsystem: AVR (PROGMEM)")
print(f"// Taps: {NUM_TAPS}, Cutoff: {CUTOFF_NORM} * Nyquist\n")

for name, win_params in windows.items():
    # 1. Float-Koeffizienten berechnen
    # firwin generiert automatisch einen kausalen (verschobenen) Sinc-Tiefpassfilter
    h_float = signal.firwin(NUM_TAPS, CUTOFF_NORM, window=win_params, pass_zero=True)
    
    # 2. In das Q15-Format konvertieren (-32768 bis +32767)
    # Multiplikation mit 32767 (nicht 32768), um bei exakt 1.0 keinen Überlauf in int16 zu riskieren.
    h_q15 = np.round(h_float * 32767).astype(int)
    
    # 3. Sicherheits-Check für den 32-Bit Akkumulator (AVR)
    # Ein int16_t Eingangswert hat max. 32767. Der int32_t Akku fasst max 2147483647.
    # 2147483647 / 32767 = ~65535. Wenn die Summe der Absolutwerte <= 65535 ist, 
    # ist ein Überlauf mathematisch ausgeschlossen.
    sum_abs = np.sum(np.abs(h_q15))
    is_safe = sum_abs <= 65535
    
    # 4. C-Code generieren
    print(f"// Fenster: {name}")
    print(f"// Summe der Absolutwerte: {sum_abs} (Überlaufsicher: {'JA' if is_safe else 'NEIN!'})")
    
    array_name = f"fir_coeffs_{name.lower().replace('-', '_')}"
    print(f"const int16_t {array_name}[{NUM_TAPS}] PROGMEM = {{")
    
    # Koeffizienten formatieren (8 pro Zeile für Lesbarkeit)
    for i in range(0, NUM_TAPS, 8):
        row = h_q15[i:i+8]
        row_str = ", ".join(f"{val:6d}" for val in row)
        print(f"    {row_str},")
    print("};\n")
