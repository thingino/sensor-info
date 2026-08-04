/* SPDX-License-Identifier: MIT */
#ifndef SINFO_MOCK_HW_H
#define SINFO_MOCK_HW_H

#include <stdint.h>
#include <linux/i2c.h>

#define MOCK_MAX_WRITES 4096
#define MOCK_MAX_CHIPS	8
#define MOCK_MAX_REGS	16
#define MOCK_MAX_BUSES	8
#define MOCK_MAX_SYSFS	256
#define MOCK_GPIO_PORTS 8

/* returned by a hook to fall through to the default chip model */
#define MOCK_I2C_PASS (-2)

struct mock_cpm_write {
	uint32_t off;
	uint32_t val;
};

struct mock_chip {
	int bus;
	uint8_t addr;
	struct {
		uint32_t reg;
		uint32_t val;
	} regs[MOCK_MAX_REGS];
	int nregs;
};

typedef int (*mock_i2c_hook_fn)(int bus, struct i2c_msg *msgs, int n);

extern uint32_t mock_cpm[];
extern struct mock_cpm_write mock_cpm_writes[];
extern int mock_cpm_nwrites;
extern struct mock_chip mock_chips[];
extern int mock_nchips;
extern mock_i2c_hook_fn mock_i2c_hook;
extern int mock_buses[];
extern int mock_nbuses;
extern unsigned long mock_slept_ms;

void mock_reset(void);
struct mock_chip *mock_chip_add(int bus, uint8_t addr);
void mock_chip_reg(struct mock_chip *c, uint32_t reg, uint32_t val);
void mock_gpio_export(int gpio);
int mock_gpio_exported(int gpio);
int mock_sysfs_wrote(const char *needle);

#endif /* SINFO_MOCK_HW_H */
