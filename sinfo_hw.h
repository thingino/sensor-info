/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sinfo_hw.h - hardware access seam
 *
 * Everything sinfo does to the machine goes through these primitives:
 * CPM registers and raw physical mappings via /dev/mem, sensor I2C via
 * /dev/i2c-N, GPIO via sysfs, and sleeps. tests/mock_hw.c provides a
 * simulated implementation so the probe logic runs host-side under the
 * sanitizers.
 */
#ifndef SINFO_HW_H
#define SINFO_HW_H

#include <stddef.h>
#include <stdint.h>
#include <linux/i2c.h>

int hw_cpm_init(void);
uint32_t hw_cpm_rd(uint32_t off);
void hw_cpm_wr(uint32_t off, uint32_t val);

/* Map a page of physical address space (GPIO blocks). NULL on failure. */
volatile uint32_t *hw_map_phys(uint32_t phys, size_t len);

int hw_i2c_open(int bus);
int hw_i2c_xfer(struct i2c_msg *msgs, int n);

int hw_sysfs_write(const char *path, const char *val);
int hw_path_writable(const char *path);

void hw_msleep(unsigned int ms);

#endif /* SINFO_HW_H */
