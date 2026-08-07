#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Generate sensors.h from sensors.csv.

sensors.csv is the hand-maintained source of truth, one line per sensor:

    name,i2c_addr,mclk_hz,id_regs,id_vals,reg_bytes,val_bytes,soc

  id_regs / id_vals   colon-separated register/value lists, same length
  reg_bytes           register address width on the wire (1-4)
  val_bytes           value width per read (1-4)
  soc                 any | t41-only | not-t41

Table order matters: among rebadged sensors with identical IDs the first
entry becomes the reported primary name. Keep families grouped and put
the canonical name first.

Sensors needing special probe sequences (unlock writes, alternate IDs)
are matched by name in sinfo.c's probe loop; adding one of those needs
a code hook as well as a table row.

Usage: gen-sensors.py [--check]
  --check   regenerate to a temp buffer and fail if sensors.h differs
"""

import csv
import io
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
CSV = ROOT / "sensors.csv"
OUT = ROOT / "sensors.h"

SOC_FLAGS = {"any": "S_ANY", "t41-only": "S_T41_ONLY", "not-t41": "S_NOT_T41"}
MAX_IDS = 4

HEADER = """\
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sensors.h - Ingenic sensor identification database
 *
 * GENERATED FILE - do not edit. The source of truth is sensors.csv;
 * regenerate with tools/gen-sensors.py (or `make sensors`).
 *
 * Table order matters: among rebadges with identical IDs the first
 * entry becomes the reported primary name.
 */
#ifndef SINFO_SENSORS_H
#define SINFO_SENSORS_H

#include <stdint.h>

/* SoC applicability flags */
#define S_ANY	    0
#define S_T41_ONLY  1
#define S_NOT_T41   2

/* Packed for size: this table dominates the binary's data segment. */
struct sensor_def {
	const char *name;	/* sensor model name */
	uint16_t id_value[4];	/* expected ID register values */
	uint16_t id_addr[4];	/* ID register addresses */
	uint32_t clk;		/* MCLK frequency in Hz */
	uint8_t i2c_addr;	/* 7-bit I2C address */
	uint8_t id_value_len;	/* value width in bytes per read */
	uint8_t id_addr_len;	/* register address width in bytes */
	uint8_t id_cnt;		/* number of ID registers to check */
	uint8_t soc;		/* S_ANY / S_T41_ONLY / S_NOT_T41 */
};

static const struct sensor_def sensor_db[] = {
"""

FOOTER = """\
};

#define SENSOR_COUNT (sizeof(sensor_db) / sizeof(sensor_db[0]))

#endif /* SINFO_SENSORS_H */
"""


def die(msg):
    sys.exit(f"gen-sensors: {msg}")


def parse_num(field, s):
    try:
        return int(s, 0)
    except ValueError:
        die(f"bad {field}: {s!r}")


def generate():
    out = io.StringIO()
    out.write(HEADER)
    seen = set()
    with open(CSV, newline="") as f:
        for ln, row in enumerate(csv.reader(f), 1):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) != 8:
                die(f"line {ln}: expected 8 fields, got {len(row)}")
            name, addr_s, clk_s, regs_s, vals_s, rw_s, vw_s, soc_s = (
                x.strip() for x in row
            )
            addr = parse_num("i2c_addr", addr_s)
            clk = parse_num("mclk_hz", clk_s)
            regs = [parse_num("id_reg", x) for x in regs_s.split(":")]
            vals = [parse_num("id_val", x) for x in vals_s.split(":")]
            rw = parse_num("reg_bytes", rw_s)
            vw = parse_num("val_bytes", vw_s)
            if soc_s not in SOC_FLAGS:
                die(f"line {ln}: bad soc {soc_s!r}")
            if len(regs) != len(vals):
                die(f"line {ln}: {name}: id_regs/id_vals length mismatch")
            if not 1 <= len(regs) <= MAX_IDS:
                die(f"line {ln}: {name}: {len(regs)} ID registers")
            if not (1 <= rw <= 4 and 1 <= vw <= 4):
                die(f"line {ln}: {name}: reg/val width out of range")
            if any(x > 0xFFFF for x in regs + vals):
                die(f"line {ln}: {name}: register/value exceeds 16 bits")
            if not 0 <= addr <= 0x7F:
                die(f"line {ln}: {name}: I2C address out of range")
            key = (name, addr, tuple(regs), tuple(vals), soc_s)
            if key in seen:
                die(f"line {ln}: duplicate entry {name}@{addr:#x}")
            seen.add(key)
            vals_c = ", ".join(f"{v:#x}" for v in vals)
            regs_c = ", ".join(f"{r:#x}" for r in regs)
            out.write(
                f'\t{{"{name}", {{{vals_c}}}, {{{regs_c}}}, {clk}, '
                f"{addr:#04x}, {vw}, {rw}, "
                f"{len(regs)}, {SOC_FLAGS[soc_s]}}},\n"
            )
    out.write(FOOTER)
    return out.getvalue()


def main():
    text = generate()
    if "--check" in sys.argv:
        current = OUT.read_text() if OUT.exists() else ""
        if current != text:
            die("sensors.h is out of date, run tools/gen-sensors.py")
        print("sensors.h is in sync with sensors.csv")
        return
    OUT.write_text(text)
    print(f"wrote {OUT} ({text.count(chr(10))} lines)")


if __name__ == "__main__":
    main()
