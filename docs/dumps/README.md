# Sensor register dumps

Fingerprint provenance for sensors whose vendors ship no chip-id
register (the Sony STARVIS families in particular). Every
identification rule we ever add must cite dumps in this directory;
nothing is adopted from third-party detection code.

## Method

With the sensor driver unloaded:

```
sinfo dump 0x1a 0x3000 0x33ff 2 1 37125000 > dump.txt
```

(address / range / register width / value width / MCLK.) The tool
drives MCLK and the reset dance itself, so the chip answers with its
reset-default values. Take dumps only from sensors whose identity is
known from board markings, vendor firmware, or a working driver.

## Findings so far

imx307, two independent units (a: T31 production camera, b: T40
dev board), dumped at 37.125 MHz:

- 1018 of 1024 registers identical across units.
- Per-unit values (mask these out of any fingerprint):
  0x31DD, 0x31DE, 0x31E0, 0x31E1, 0x31E2, 0x31E3 - consecutive
  fuse-area bytes, evidently lot/trim/serial data.
- Stable values of interest on both units:
  0x3011=0x00 (the mainline kernel, citing the datasheets, has
  imx327 initialize this to 0x02 and imx290 to 0x00),
  0x31DC=0x0C, 0x309C=0x22.

## Wanted

Dumps from known imx290, imx291, imx327, imx462, imx662 (and any
other rebadge-family silicon: gc2053/gc2063, gc5603/gc5613,
sc2336/sc2336p...). Open an issue with the dump attached and how
the sensor's identity is known. Discriminator rules ship only once
dumps from at least two independent units of a model agree.
