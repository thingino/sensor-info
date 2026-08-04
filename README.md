# sinfo (userspace)

Userspace port of the ingenic-sdk `sinfo` kernel module: detects the image
sensor on Ingenic T-series camera SoCs by probing the sensor I2C bus against
a database of known sensors.

Where the kernel module needed a `.ko` built for the exact SoC *and* kernel
version, this is one static MIPS binary that runs on any kernel that has:

- `CONFIG_I2C_CHARDEV` (`/dev/i2c-N`)
- `CONFIG_GPIO_SYSFS` (`/sys/class/gpio`)
- `CONFIG_DEVMEM` (`/dev/mem`)

## How it works

For each database entry (same table and scan order as the kernel module):

1. Program the CIM MCLK to the sensor's rate: CGU `CIMCDR` divider from the
   currently selected PLL parent, CE/BUSY handshake, STOP cleared. Done with
   direct CPM register access via `/dev/mem` (offsets and bit layouts from
   ingenic-u-boot-xburst1, verified on hardware).
2. Run the reset/pwdn GPIO sequence via `/sys/class/gpio`, including the
   per-sensor timing quirks (sp1409, sc2336p/sc2337p/sc3336p).
3. Read the ID registers over `/dev/i2c-N` (`I2C_RDWR`, same message shapes
   as the module's `i2c_transfer` calls) and compare, including the
   sc2336p-vs-sc2337p disambiguation and the ov2735b alternate-ID quirk.

Matches are reported in the same format as `/proc/jz/sinfo/info`, plus a
final `Detected sensors: N` summary line. Exit code: `0` = at least one
match, `1` = none, `2` = error.

## Usage

```
sinfo                        # probe, autodetecting SoC from uname -r
sinfo -s t31 -v              # explicit SoC, verbose
sinfo -b 0 -r 18 -p -1       # explicit bus / reset gpio / pwdn gpio
sinfo open gc2053            # enable MCLK + reset for bench I2C poking
sinfo i2c-r 0x37 1           # raw I2C read (after open)
sinfo release                # stop MCLK, free GPIOs
```

Run as root. If a sensor kernel driver is loaded it owns the reset GPIO, so
the GPIO dance is skipped for those pins (same as the module's
`gpio_request` failing); unload the sensor driver for a clean probe.

## SoC support

| SoC | MCLK reg | CIM parent mux | PLL format | status |
|-----|----------|----------------|------------|--------|
| t31 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | HW-validated |
| t23 | CIMCDR 0x7c | APLL/MPLL @30 | new | untested |
| c100 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | untested |
| t20 | CIMCDR 0x7c | APLL/MPLL/VPLL @30 | new | untested |
| t30 | CIMCDR 0x7c | APLL/MPLL/VPLL/EPLL @30 | old | untested |
| t21 | CIMCDR 0x7c | APLL/MPLL/VPLL/EPLL @30 | old | untested |
| t10 | CIMCDR 0x7c | APLL/MPLL @31 (1 bit) | new | untested |
| t40 | CIM1CDR 0x94 | APLL/MPLL/VPLL/EPLL @30 | new | untested |
| t41 | CIM0CDR 0x90 | APLL/MPLL/VPLL @30 | t41 | untested |
| a1 | - | - | - | not implemented (module never supported it) |

PLL formats: "new" `EXTAL*M/N/OD0/OD1`; "old" `EXTAL*2*(M+1)/(N+1)/2^OD`;
"t41" `EXTAL*2*(M+1)/((N+1)*2^OD0*(OD1+1))`. EXTAL is assumed 24 MHz.

Notes:
- On XBurst1 the MCLK pin mux is not touched (same as the kernel module);
  on a normal boot the boot chain / sensor driver has already set it.
- On XBurst2 the MCLK pin is muxed like the module does: T40 PC30 func1
  (CIM1), T41 PA15 func1 (CIM0), via the GPIO set/clear registers.

## Build

```
make CROSS_COMPILE=mipsel-linux-       # static MIPS binary
make                                   # host build (for -h, table checks)
```
