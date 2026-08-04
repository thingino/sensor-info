// SPDX-License-Identifier: GPL-2.0
/*
 * mock_hw.c - simulated hardware for host-side tests
 *
 * Implements the sinfo_hw.h seam: CPM registers are an array with a
 * write log (for CE/BUSY handshake assertions), GPIO pages are plain
 * memory, I2C is a table of mock chips with an optional per-test hook
 * for stateful devices, and sysfs GPIO tracks the exported set.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../sinfo_hw.h"
#include "mock_hw.h"

uint32_t mock_cpm[0x1000 / 4];
struct mock_cpm_write mock_cpm_writes[MOCK_MAX_WRITES];
int mock_cpm_nwrites;

static uint32_t mock_gpio[MOCK_GPIO_PORTS][0x1000 / 4];
uint32_t mock_efuse[0x1000 / 4];
uint32_t mock_sub[0x1000 / 4];

struct mock_chip mock_chips[MOCK_MAX_CHIPS];
int mock_nchips;
mock_i2c_hook_fn mock_i2c_hook;

static int cur_bus = -1;
int mock_buses[MOCK_MAX_BUSES];
int mock_nbuses;

char mock_sysfs_log[MOCK_MAX_SYSFS][128];
int mock_nsysfs;
static int exported[1024];

unsigned long mock_slept_ms;
int mock_export_fail;
int mock_i2c_xfers;

void mock_reset(void)
{
	memset(mock_cpm, 0, sizeof(mock_cpm));
	memset(mock_gpio, 0, sizeof(mock_gpio));
	memset(mock_efuse, 0, sizeof(mock_efuse));
	memset(mock_sub, 0, sizeof(mock_sub));
	mock_cpm_nwrites = 0;
	mock_nchips = 0;
	mock_i2c_hook = NULL;
	cur_bus = -1;
	mock_nbuses = 1;
	mock_buses[0] = 0;
	mock_nsysfs = 0;
	memset(exported, 0, sizeof(exported));
	mock_slept_ms = 0;
	mock_export_fail = 0;
	mock_i2c_xfers = 0;
}

struct mock_chip *mock_chip_add(int bus, uint8_t addr)
{
	struct mock_chip *c;

	if (mock_nchips >= MOCK_MAX_CHIPS)
		abort();
	c = &mock_chips[mock_nchips++];
	memset(c, 0, sizeof(*c));
	c->bus = bus;
	c->addr = addr;
	return c;
}

void mock_chip_reg(struct mock_chip *c, uint32_t reg, uint32_t val)
{
	if (c->nregs >= MOCK_MAX_REGS)
		abort();
	c->regs[c->nregs].reg = reg;
	c->regs[c->nregs].val = val;
	c->nregs++;
}

void mock_gpio_export(int gpio)
{
	if (gpio >= 0 && gpio < 1024)
		exported[gpio] = 1;
}

int mock_gpio_exported(int gpio)
{
	return gpio >= 0 && gpio < 1024 && exported[gpio];
}

int mock_sysfs_wrote(const char *needle)
{
	int i;

	for (i = 0; i < mock_nsysfs; i++)
		if (strstr(mock_sysfs_log[i], needle))
			return 1;
	return 0;
}

/* ------------------------------------------------------------- seam impl */

int hw_cpm_init(void)
{
	return 0;
}

uint32_t hw_cpm_rd(uint32_t off)
{
	return mock_cpm[off / 4];
}

void hw_cpm_wr(uint32_t off, uint32_t val)
{
	if (mock_cpm_nwrites < MOCK_MAX_WRITES) {
		mock_cpm_writes[mock_cpm_nwrites].off = off;
		mock_cpm_writes[mock_cpm_nwrites].val = val;
		mock_cpm_nwrites++;
	}
	mock_cpm[off / 4] = val;
}

volatile uint32_t *hw_map_phys(uint32_t phys, size_t len)
{
	(void)len;
	if (phys == 0x10000000)
		return mock_cpm;
	if (phys >= 0x10010000 && phys < 0x10010000 + MOCK_GPIO_PORTS * 0x1000u)
		return mock_gpio[(phys - 0x10010000) / 0x1000];
	if (phys == 0x13000000)
		return mock_efuse;
	if (phys == 0x13540000)
		return mock_sub;
	return NULL;
}

int hw_i2c_open(int bus)
{
	cur_bus = bus;
	return 0;
}

static struct mock_chip *find_chip(uint8_t addr)
{
	int i;

	for (i = 0; i < mock_nchips; i++)
		if (mock_chips[i].bus == cur_bus && mock_chips[i].addr == addr)
			return &mock_chips[i];
	return NULL;
}

static int chip_read(struct mock_chip *c, uint32_t reg, uint32_t *val)
{
	int i;

	for (i = 0; i < c->nregs; i++)
		if (c->regs[i].reg == reg) {
			*val = c->regs[i].val;
			return 0;
		}
	*val = 0; /* real sensors answer unmapped registers with zeros */
	return 0;
}

int hw_i2c_xfer(struct i2c_msg *msgs, int n)
{
	struct mock_chip *c;
	uint32_t reg = 0, val = 0;
	int i;

	mock_i2c_xfers++;

	if (mock_i2c_hook) {
		int r = mock_i2c_hook(cur_bus, msgs, n);

		if (r != MOCK_I2C_PASS)
			return r;
	}

	c = find_chip(msgs[0].addr);
	if (!c)
		return -1; /* NACK: nothing at this address */

	if (n == 2 && (msgs[1].flags & I2C_M_RD)) {
		for (i = 0; i < msgs[0].len; i++)
			reg = (reg << 8) | msgs[0].buf[i];
		chip_read(c, reg, &val);
		for (i = 0; i < msgs[1].len; i++)
			msgs[1].buf[i] = (val >> (8 * (msgs[1].len - 1 - i))) & 0xff;
		return 0;
	}
	return 0; /* plain writes accepted */
}

int hw_i2c_buses(int *buses, int max)
{
	int i, n = mock_nbuses < max ? mock_nbuses : max;

	for (i = 0; i < n; i++)
		buses[i] = mock_buses[i];
	return n;
}

int hw_sysfs_write(const char *path, const char *val)
{
	int gpio;

	if (mock_nsysfs < MOCK_MAX_SYSFS)
		snprintf(mock_sysfs_log[mock_nsysfs++], sizeof(mock_sysfs_log[0]), "%s=%s", path,
			 val);

	if (strstr(path, "/export") && !strstr(path, "unexport")) {
		if (mock_export_fail)
			return -1;
		if (sscanf(val, "%d", &gpio) == 1)
			mock_gpio_export(gpio);
	} else if (strstr(path, "/unexport")) {
		if (sscanf(val, "%d", &gpio) == 1 && gpio >= 0 && gpio < 1024)
			exported[gpio] = 0;
	}
	return 0;
}

int hw_path_writable(const char *path)
{
	int gpio;

	if (sscanf(path, "/sys/class/gpio/gpio%d/", &gpio) == 1)
		return mock_gpio_exported(gpio);
	return 0;
}

void hw_msleep(unsigned int ms)
{
	mock_slept_ms += ms;
}
