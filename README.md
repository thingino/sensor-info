# sensor-info

Userspace Ingenic image-sensor detector for thingino. Probes the
sensor I2C bus against a database of ~290 known sensors, driving the
sensor's master clock and reset/power-down GPIOs itself, and reports
what is on the board.

It is a userspace port of the ingenic-sdk `sinfo` kernel module.
Where the module needed a `.ko` built for the exact SoC *and* kernel
version, this is one static MIPS binary that runs on any kernel with:

- `CONFIG_I2C_CHARDEV` (`/dev/i2c-N`)
- `CONFIG_GPIO_SYSFS` (`/sys/class/gpio`)
- `CONFIG_DEVMEM` (`/dev/mem`)

## Usage

```
sinfo                        # probe, auto-detecting the SoC
sinfo -v probe               # verbose progress on stderr
sinfo -s t31 -b 0 -r 18      # explicit SoC / bus / reset GPIO
sinfo -b all                 # scan every /dev/i2c-* bus
sinfo -m 0                   # select MCLK block (multi-sensor T40)
sinfo -p 21                  # sensors needing a PWDN dance (e.g. T10)
sinfo open gc2053            # enable MCLK + reset for bench I2C work
sinfo i2c-r 0x37 1           # raw I2C read (after open)
sinfo release                # stop MCLK, free GPIOs
```

Run as root. If a sensor kernel driver is loaded it owns the reset
GPIO and the bus; unload it (`rmmod sensor_*`) for a clean probe.

Exit codes: `0` at least one sensor found, `1` none found, `2` error.
The report ends with a script-friendly `Detected sensors: N` line.

### SoC auto-detection

Resolution order: `-s` flag, `isvp_<codename>` in `uname -r`,
`/proc/cpuinfo` (`system type` codename or the XBurst2 machine
name), then thingino's `soc -f`. Known codenames: swan=t31,
pike=t23, bull=t20, monkey=t30, turkey=t21, mango=t10; machine
names marmot=t41, shark=t40.

## How it works

For every database entry the tool mirrors what the kernel module's
probe did, from userspace:

1. **MCLK.** Programs the CIM clock divider in the CPM (via
   `/dev/mem`) for the sensor's rate: reads the currently selected
   PLL parent, computes the divider the same way the kernel clock
   framework does (smallest divider with parent/div <= rate), and
   programs it under the CE/BUSY handshake. If the selected parent
   PLL is parked it falls back to the first live PLL. The register
   is left gated with the parent preserved after the run.
2. **Pin mux (XBurst2).** T40/T41 MCLK pins are muxed to their device
   function through the GPIO set/clear registers, plus the T40
   vendor VDD-select write, so probing works even when no sensor
   driver has run since boot.
3. **Reset/PWDN dance** via sysfs GPIO, including the per-sensor
   timing quirks (sp1409, sc2336p/sc2337p/sc3336p).
4. **ID reads** over `/dev/i2c-N` (`I2C_RDWR`), including the
   sc2336p-vs-sc2337p disambiguation and the ov2735b alternate-ID
   check, then compares against the table.

Rebadged sensors with identical ID registers (gc5603/gc5613,
gc2053/gc2063, imx291/imx307, ...) all match one physical chip; the
report groups them into one device with the other names listed as
aliases. Which name leads is table order.

### Multi-sensor units

- Two sensors on one bus at different addresses: found in one run.
- Sensors on different buses: `-b all`, or per-bus runs. Each
  device is reported with its bus.
- XBurst2 units with a second sensor on another MCLK block: one
  block drives a run (`-m`, T40: 0/1/2 = CIM0/1/2 on PC31/PC30/PC29,
  default CIM1); run once per block. Sensors clocked by an external
  oscillator (common on vanhua boards) probe regardless.

## SoC support

| SoC | MCLK reg | CIM parent mux | PLL format | status |
|-----|----------|----------------|------------|--------|
| t31 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | HW-validated |
| t23 | CIMCDR 0x7c | APLL/MPLL @30 | new | HW-validated |
| c100 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | untested |
| t20 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | HW-validated |
| t30 | CIMCDR 0x7c | APLL/MPLL/VPLL/EPLL @30 | old | HW-validated |
| t21 | CIMCDR 0x7c | APLL/MPLL/VPLL/EPLL @30 | old | HW-validated |
| t10 | CIMCDR 0x7c | APLL/MPLL @31 (1 bit) | new | HW-validated |
| t40 | CIM0/1/2CDR 0x90/94/98 | APLL/MPLL/VPLL/EPLL @30 | new | HW-validated |
| t41 | CIM0CDR 0x90 | APLL/MPLL/VPLL @30 | t41 | HW-validated |
| a1 | - | - | - | not implemented (module never supported it) |

PLL formats: "new" `EXTAL*M/N/OD0/OD1`; "old"
`EXTAL*2*(M+1)/(N+1)/2^OD`; "t41"
`EXTAL*2*(M+1)/((N+1)*2^OD0*(OD1+1))`. EXTAL is assumed 24 MHz.
Register data comes from the ingenic-u-boot-xburst1/-xburst2 trees
and the vendor 4.4.94 SDK; every hardware-validated row was proven
on a real board with the values cross-checked against the running
kernel's own clock programming.

## Adding a sensor

The table lives in `sensors.csv` (one line per sensor); `sensors.h`
is generated from it.

1. Add a line: `name,i2c_addr,mclk_hz,id_regs,id_vals,reg_bytes,val_bytes,soc`
   (see the header comments in the csv for field details).
   Keep vendor families grouped; among rebadges with identical IDs
   the first entry becomes the reported primary name.
2. `make sensors` to regenerate `sensors.h` (validates widths,
   address range, and duplicates), and commit both files.
3. `make test` - the table invariants run against the new entry,
   and `check-table` fails the build if the two files drift.

Sensors that need a special probe sequence (unlock writes, alternate
ID values) are matched by name in `sinfo.c`'s probe loop; those need
a code hook in addition to the table row.

### Identifying an unknown sensor

If a chip responds but nothing matches, the `ALL I2C DEVICES
DETECTED` section of the report shows every register that answered
and the values read. To build the new csv line from that you need:

1. the sensor model name (board silkscreen, vendor firmware, or the
   value pattern - most vendors encode the model in the ID, e.g.
   `0x46:0x53` = gc4653),
2. which registers are the chip's ID registers and their expected
   values (the reads in the report are your candidates),
3. the register/value byte widths (what the working entries of the
   same vendor family use is almost always right),
4. the MCLK frequency (24000000 unless the vendor driver says
   otherwise).

Note that entries probing the same address with different ID
registers will show as extra "unknown device" rows in the report -
partial reads of an already-identified chip are normal, not a
second sensor.

## Building

```
make CROSS_COMPILE=mipsel-linux-    # static MIPS binary (-Os)
make test                           # host tests under ASAN+UBSan
make sensors                        # regenerate sensors.h from csv
clang-format -i sinfo.c sinfo_hw.c  # before committing
```

The tree splits into `sinfo.c` (probe/clock/report logic) and
`sinfo_hw.c` (the `hw_*` hardware seam: /dev/mem, I2C, sysfs).
`tests/mock_hw.c` implements the same seam in memory, so the whole
probe path runs host-side; the PLL test vectors are live register
values captured from the validated boards.

## Known table defects (inherited from the kernel module)

- `imx327`'s ID registers partially overlap `imx307`'s and
  false-match on imx307 hardware.
- `imx662` expects all-zero ID values, which Sony sensors return
  for unmapped registers, so it false-matches any idle Sony chip.
- The kernel module additionally has an `int8_t` sensor-index bug:
  its `/proc` primary-sensor line and `IOCTL_SINFO_GET` are corrupt
  for any sensor past table index 127 (the whole SmartSens block).
  This port fixes the index type and carries a regression test.

## License

GPL-2.0, same as the ingenic-sdk `sinfo` kernel module the sensor
database and probe behavior derive from.
