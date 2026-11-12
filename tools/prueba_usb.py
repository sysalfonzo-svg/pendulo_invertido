"""Comprueba que un receptor detenido no frena el muestreo de la STM32.

Solo envia ?, G, H y S; nunca calibra ni solicita movimiento.
Usar Python de PlatformIO (incluye pyserial). Cerrar MATLAB/monitor serie.
"""
import argparse
import csv
import io
import json
from pathlib import Path
import time
import serial


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port")
    p.add_argument("--stall", type=float, default=5.0)
    p.add_argument("--output", default="registros/prueba_usb")
    args = p.parse_args()
    if not 0 <= args.stall <= 30:
        p.error("--stall debe estar entre 0 y 30 s")
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    chunks = []
    confirmed = False
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.5) as s:
        try:
            s.dtr = True
            deadline = time.monotonic() + 5
            identity = bytearray()
            while time.monotonic() < deadline:
                s.write(b"?\n")
                identity.extend(s.read(512))
                if b"# PENDULO_STATES_V1;" in identity:
                    confirmed = True
                    break
            if not confirmed:
                raise RuntimeError("No se identifica PENDULO_STATES_V1; no se envia G")
            s.reset_input_buffer()
            s.write(b"G\n")

            def receive_for(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    chunks.append(s.read(max(1, min(s.in_waiting, 16384))))

            receive_for(2)
            # Pausa intencional SOLO del receptor PC. La STM32 sigue midiendo.
            time.sleep(args.stall)
            receive_for(3)
            s.write(b"H\n")
            receive_for(0.2)
        finally:
            if confirmed:
                s.write(b"H\nS\n")
    raw = b"".join(chunks).decode("ascii", errors="replace")
    out.with_suffix(".txt").write_text(raw, encoding="utf-8")
    rows = []
    headers = None
    for line in raw.splitlines():
        if line.startswith("# CSV:"):
            headers = line[6:].split(",")
        elif line and not line.startswith("#"):
            fields = next(csv.reader(io.StringIO(line)))
            if len(fields) != 27:
                raise RuntimeError("Fila incompleta: " + line)
            rows.append([float(x) for x in fields])
    if not headers or len(rows) < 100:
        raise RuntimeError("No hay cabecera/datos suficientes")
    with out.with_suffix(".csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(headers)
        w.writerows(rows)
    increments = lambda col: (rows[-1][col] - rows[0][col]) % (2**32)
    summary = {
        "muestras": len(rows), "pausa_receptor_s": args.stall,
        "dt_min_us": min(r[2] for r in rows[1:]),
        "dt_max_us": max(r[2] for r in rows[1:]),
        "sensor_invalidas": sum(bool(int(r[11]) & 1) for r in rows),
        "incremento_missed": increments(16),
        "incremento_tx_drop": increments(17),
        "incremento_i2c_errors": increments(18),
        "pwm_siempre_cero": all(r[12] == r[13] == 0 for r in rows),
        "acq_max_us": max(r[19] for r in rows),
        "late_max_us": max(r[20] for r in rows),
    }
    out.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    assert summary["incremento_missed"] == 0, "Se perdieron ciclos de adquisicion"
    assert summary["sensor_invalidas"] == summary["incremento_i2c_errors"] == 0
    assert summary["pwm_siempre_cero"]
    assert summary["dt_min_us"] >= 4900 and summary["dt_max_us"] <= 5100
    if args.stall > 0 and summary["incremento_tx_drop"] == 0:
        print("No se lleno la cola: Windows absorbio la pausa; no demuestra saturacion USB.")


if __name__ == "__main__":
    main()
