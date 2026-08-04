/*
 * sinfo - userspace Ingenic image sensor detector
 *
 * Userspace port of the ingenic-sdk sinfo kernel module. Probes the sensor
 * I2C bus for every sensor in the database, driving the sensor MCLK (CGU CIM
 * clock via /dev/mem) and the reset/pwdn GPIOs (via /sys/class/gpio) the same
 * way the kernel module does, then reads and compares ID registers through
 * /dev/i2c-N (I2C_RDWR).
 *
 * One static binary runs on any kernel; no per-kernel-version .ko needed.
 *
 * Requirements: root, CONFIG_I2C_CHARDEV, CONFIG_GPIO_SYSFS, CONFIG_DEVMEM.
 *
 * Usage:
 *   sinfo [options] [command]
 * Commands:
 *   probe                     scan for sensors and print report (default)
 *   open <sensor>             set MCLK + reset for a named sensor, leave on
 *   release                   stop MCLK, free GPIOs
 *   i2c-r <addr> <len>        raw I2C read  (like echo i2c-r:... > proc)
 *   i2c-w <addr> <data> <len> raw I2C write (like echo i2c-w:... > proc)
 * Options:
 *   -s <soc>    SoC (t10/t20/t21/t23/t30/t31/c100/t40/t41); default: auto-detect
 *   -b <bus>    I2C bus number (default: per-SoC, 0)
 *   -r <gpio>   reset GPIO number (default: per-SoC, 18 = PA18; -1 = none)
 *   -p <gpio>   power-down GPIO number (default: -1 = none)
 *   -v          verbose progress on stderr
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "sensors.h"

#define MAX_DETECTED_SENSORS 4
#define MAX_I2C_SCAN_RESULTS 128

#define EXTAL_HZ	     24000000ull

/* ---------------------------------------------------------------- SoC data */

/*
 * All supported SoCs: CPM at 0x10000000. The CIM MCLK divider register
 * (CIMCDR on XBurst1 at +0x7c, CIM0CDR/CIM1CDR on XBurst2 at +0x90/+0x94)
 * shares one layout: parent mux in the top bits, CE bit 29, BUSY bit 28,
 * STOP bit 27, 8-bit divider. Parent maps and PLL register formats differ
 * per SoC (source: ingenic-u-boot-xburst1/-xburst2 per-SoC clk.c/cpm.h).
 */
#define CPM_PHYS  0x10000000
#define GPIO_PHYS 0x10010000
#define CE_BIT	  29
#define BUSY_BIT  28
#define STOP_BIT  27
#define DIV_MASK  0xff

enum { PLL_NONE = 0, PLL_A, PLL_M, PLL_V, PLL_E };

/* rate = EXTAL * m/n/od0/od1        m[31:20] n[19:14] od1[13:11] od0[10:8] */
#define PLLSTYLE_NEW 0
/* rate = EXTAL * 2*(m+1)/(n+1)/2^od m[28:20] n[19:14] od[13:11] */
#define PLLSTYLE_OLD 1
/* rate = EXTAL * 2*(m+1)/((n+1) * 2^od0 * (od1+1))
 *                                   m[28:20] n[19:14] od1[13:11] od0[10:7] */
#define PLLSTYLE_T41 2

struct soc_desc {
	const char *name;
	const char *uname_tag; /* "isvp_<tag>" in uname -r, NULL = unknown */
	int pll_style;
	uint32_t cimcdr_off; /* CIM MCLK divider register, CPM offset */
	uint8_t mux_shift;
	uint8_t mux_width;
	uint8_t parent[4];   /* PLL per mux field value */
	uint8_t default_mux; /* used when current parent is off/invalid */
	int i2c_bus;
	int reset_gpio;
	int8_t mux_port; /* MCLK pin function fixup: GPIO port, -1 = none */
	int8_t mux_pin;	 /* pin within port */
	int8_t mux_func; /* device function number */
	int tested;	 /* HW-validated */
};

/*
 * XBurst2 MCLK pin fixups mirror the kernel module's jzgpio_set_func()
 * calls: T40 cim1_gpio PC30 FUNC_1, T41 cim_gpio PA15 FUNC_1.
 * XBurst1 entries deliberately have none (the module doesn't either;
 * the boot chain / a previously loaded sensor driver sets the mux).
 */
static const struct soc_desc soc_table[] = {
	{
		.name = "t31",
		.uname_tag = "swan",
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_NONE},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "t23",
		.uname_tag = "pike",
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_NONE, PLL_NONE},
		.default_mux = 1,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "c100",
		.uname_tag = NULL,
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_NONE},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 0,
	},
	{
		.name = "t20",
		.uname_tag = "bull",
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_V},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "t30",
		.uname_tag = "monkey",
		.pll_style = PLLSTYLE_OLD,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_E},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "t21",
		.uname_tag = "turkey",
		.pll_style = PLLSTYLE_OLD,
		.cimcdr_off = 0x7c,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_E},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "t10",
		.uname_tag = "mango",
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x7c,
		.mux_shift = 31,
		.mux_width = 1,
		.parent = {PLL_A, PLL_M, PLL_NONE, PLL_NONE},
		.default_mux = 1,
		.i2c_bus = 0,
		.reset_gpio = 18,
		.mux_port = -1,
		.tested = 1,
	},
	{
		.name = "t40",
		.uname_tag = NULL,
		.pll_style = PLLSTYLE_NEW,
		.cimcdr_off = 0x94,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_E},
		.default_mux = 2,
		.i2c_bus = 1,
		.reset_gpio = 91,
		.mux_port = 2,
		.mux_pin = 30,
		.mux_func = 1,
		.tested = 1,
	},
	{
		.name = "t41",
		.uname_tag = NULL,
		.pll_style = PLLSTYLE_T41,
		.cimcdr_off = 0x90,
		.mux_shift = 30,
		.mux_width = 2,
		.parent = {PLL_A, PLL_M, PLL_V, PLL_NONE},
		.default_mux = 2,
		.i2c_bus = 0,
		.reset_gpio = 92,
		.mux_port = 0,
		.mux_pin = 15,
		.mux_func = 1,
		.tested = 1,
	},
};
#define SOC_COUNT (sizeof(soc_table) / sizeof(soc_table[0]))

/* ------------------------------------------------------------------ state */

static const struct soc_desc *cur_soc;
static int bus_nr = -1; /* -1 = use SoC default */
static int reset_pin = -9999;
static int pwdn_pin = -1;
static int verbose;

static int i2c_fd = -1;
static volatile uint32_t *cpm_base;

/*
 * int, not int8_t: the table has ~290 entries, so an int8_t index
 * overflows for any sensor past index 127 (the kernel module has this
 * exact bug: its int8_t primary_idx corrupts the primary sensor and
 * IOCTL_SINFO_GET for the whole SmartSens block).
 */
static int primary_idx = -1;
static int match_idx[MAX_DETECTED_SENSORS];
static int num_matches;

struct i2c_scan_result {
	uint8_t i2c_addr;
	uint8_t responded;
	uint32_t reg_values[8];
	uint32_t reg_addrs[8];
	uint8_t num_regs;
	char sensor_name[32];
};
static struct i2c_scan_result scan_results[MAX_I2C_SCAN_RESULTS];
static int num_scan_results;
/* observed ID reads per match, for grouping indistinguishable entries */
static struct i2c_scan_result match_res[MAX_DETECTED_SENSORS];

#define vlog(...)                                                                                  \
	do {                                                                                       \
		if (verbose)                                                                       \
			fprintf(stderr, "sinfo: " __VA_ARGS__);                                    \
	} while (0)
#define elog(...) fprintf(stderr, "sinfo: [Error] " __VA_ARGS__)

static void msleep(unsigned ms)
{
	usleep(ms * 1000);
}

/* ------------------------------------------------------------- /dev/mem */

static void *map_phys(uint32_t phys, size_t len)
{
	static int memfd = -1;
	void *p;

	if (memfd < 0) {
		memfd = open("/dev/mem", O_RDWR | O_SYNC);
		if (memfd < 0) {
			elog("cannot open /dev/mem: %s\n", strerror(errno));
			return NULL;
		}
	}
	p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, phys);
	if (p == MAP_FAILED) {
		elog("mmap 0x%08x failed: %s\n", phys, strerror(errno));
		return NULL;
	}
	return p;
}

static int cpm_init(void)
{
	if (cpm_base)
		return 0;
	cpm_base = map_phys(CPM_PHYS, 0x1000);
	return cpm_base ? 0 : -1;
}

static uint32_t cpm_rd(uint32_t off)
{
	return cpm_base[off / 4];
}

static void cpm_wr(uint32_t off, uint32_t val)
{
	cpm_base[off / 4] = val;
}

static int cim_wait_not_busy(void)
{
	int i;

	for (i = 0; i < 10000; i++) {
		if (!(cpm_rd(cur_soc->cimcdr_off) & (1u << BUSY_BIT)))
			return 0;
		usleep(10);
	}
	elog("CIMCDR busy bit stuck\n");
	return -1;
}

static uint64_t pll_rate(int pll)
{
	uint32_t off, r, m, n;

	switch (pll) {
	case PLL_A:
		off = 0x10;
		break;
	case PLL_M:
		off = 0x14;
		break;
	case PLL_V:
		off = 0xe0;
		break;
	case PLL_E:
		off = 0x58;
		break;
	default:
		return 0;
	}
	r = cpm_rd(off);

	if (cur_soc->pll_style == PLLSTYLE_NEW) {
		uint32_t od1, od0;

		if (!(r & 1)) /* PLL not on */
			return 0;
		m = (r >> 20) & 0xfff;
		n = (r >> 14) & 0x3f;
		od1 = (r >> 11) & 0x7;
		od0 = (r >> 8) & 0x7;
		if (!n || !od0 || !od1)
			return 0;
		return EXTAL_HZ * m / n / od0 / od1;
	} else if (cur_soc->pll_style == PLLSTYLE_T41) {
		uint32_t od1, od0;

		if (!(r & 1)) /* PLL not on */
			return 0;
		m = (r >> 20) & 0x1ff;
		n = (r >> 14) & 0x3f;
		od1 = (r >> 11) & 0x7;
		od0 = (r >> 7) & 0xf;
		return EXTAL_HZ * 2 * (m + 1) / ((n + 1) * (1u << od0) * (od1 + 1));
	} else {
		uint32_t od;

		if (!(r & 1)) /* PLL not on (e.g. T30 EPLL) */
			return 0;
		m = (r >> 20) & 0x1ff;
		n = (r >> 14) & 0x3f;
		od = (r >> 11) & 0x7;
		return EXTAL_HZ * 2 * (m + 1) / (n + 1) / (1u << od);
	}
}

/*
 * Set the CIM MCLK to the requested rate and ungate it. Mirrors what the
 * kernel clk framework does for clk_set_rate("cgu_cim")+clk_enable():
 * smallest divider with parent/div <= rate, programmed under the CE
 * handshake, STOP cleared.
 */
static int mclk_enable(uint32_t hz)
{
	uint32_t v, muxmask, mux, div, nv;
	uint64_t prate;

	v = cpm_rd(cur_soc->cimcdr_off);
	muxmask = (1u << cur_soc->mux_width) - 1;
	mux = (v >> cur_soc->mux_shift) & muxmask;
	prate = pll_rate(cur_soc->parent[mux]);
	if (!prate) {
		/*
		 * Unprogrammed/parked CDR, or its parent PLL is off (seen on
		 * T20: u-boot's nominal CIM parent is VPLL but VPLL is not
		 * running). Try the SoC default, then any parent whose PLL
		 * is actually alive.
		 */
		uint32_t m;

		mux = cur_soc->default_mux;
		prate = pll_rate(cur_soc->parent[mux]);
		for (m = 0; !prate && m <= muxmask; m++) {
			mux = m;
			prate = pll_rate(cur_soc->parent[mux]);
		}
		if (!prate) {
			elog("no usable CIM parent clock\n");
			return -1;
		}
	}

	div = (uint32_t)((prate + hz - 1) / hz);
	if (div < 1)
		div = 1;
	if (div > DIV_MASK + 1)
		div = DIV_MASK + 1;

	if (cim_wait_not_busy())
		return -1;
	nv = v & ~(muxmask << cur_soc->mux_shift) & ~DIV_MASK & ~(1u << STOP_BIT);
	nv |= (mux << cur_soc->mux_shift) | (div - 1) | (1u << CE_BIT);
	cpm_wr(cur_soc->cimcdr_off, nv);
	if (cim_wait_not_busy())
		return -1;
	cpm_wr(cur_soc->cimcdr_off, nv & ~(1u << CE_BIT));

	vlog("MCLK: parent %llu Hz / %u = %llu Hz (CIMCDR 0x%08x)\n", (unsigned long long)prate,
	     div, (unsigned long long)(prate / div), cpm_rd(cur_soc->cimcdr_off));
	return 0;
}

static void mclk_disable(void)
{
	uint32_t v;

	if (cim_wait_not_busy())
		return;
	v = cpm_rd(cur_soc->cimcdr_off) | (1u << CE_BIT) | (1u << STOP_BIT);
	cpm_wr(cur_soc->cimcdr_off, v);
	cim_wait_not_busy();
	cpm_wr(cur_soc->cimcdr_off, v & ~(1u << CE_BIT));
}

/*
 * T21 kernel module init does: *(volatile u32 *)0xB0010104 = 0x1;
 * (GPIO block, phys 0x10010104). Replicated verbatim from the module.
 */
static void t21_init_quirk(void)
{
	volatile uint32_t *gpio = map_phys(GPIO_PHYS, 0x1000);

	if (gpio)
		gpio[0x104 / 4] = 0x1;
}

/*
 * XBurst2 MCLK pin function select. Ports sit 0x1000 apart and expose
 * set/clear registers for INT/MSK/PAT1/PAT0, so the function bits can be
 * programmed without read-modify-write (same scheme as the kernel's
 * jzgpio_set_func() and u-boot's gpio_set_func()). Pull config is left
 * untouched.
 */
static void xb2_mclk_pin_mux(void)
{
	volatile uint32_t *port;
	uint32_t pins;

	if (cur_soc->mux_port < 0)
		return;
	port = map_phys(GPIO_PHYS + (uint32_t)cur_soc->mux_port * 0x1000, 0x1000);
	if (!port) {
		elog("cannot map GPIO port %c\n", 'A' + cur_soc->mux_port);
		return;
	}
	pins = 1u << cur_soc->mux_pin;
	port[(cur_soc->mux_func & 0x8 ? 0x14 : 0x18) / 4] = pins; /* INT  S/C */
	port[(cur_soc->mux_func & 0x4 ? 0x24 : 0x28) / 4] = pins; /* MSK  S/C */
	port[(cur_soc->mux_func & 0x2 ? 0x34 : 0x38) / 4] = pins; /* PAT1 S/C */
	port[(cur_soc->mux_func & 0x1 ? 0x44 : 0x48) / 4] = pins; /* PAT0 S/C */
	vlog("MCLK pin mux: P%c%d -> function %d\n", 'A' + cur_soc->mux_port, cur_soc->mux_pin,
	     cur_soc->mux_func);
}

/* ------------------------------------------------------------ GPIO sysfs */

static int sysfs_write(const char *path, const char *val)
{
	int fd, ret;

	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -1;
	ret = write(fd, val, strlen(val));
	close(fd);
	return ret < 0 ? -1 : 0;
}

/* Equivalent of gpio_request(): claim via sysfs export. */
static int gpio_claim(int gpio)
{
	char buf[64], path[80];

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
	if (access(path, W_OK) == 0)
		return 0; /* already exported (e.g. by a previous run) */

	snprintf(buf, sizeof(buf), "%d", gpio);
	if (sysfs_write("/sys/class/gpio/export", buf) < 0)
		return -1;
	return access(path, W_OK) == 0 ? 0 : -1;
}

static void gpio_release(int gpio)
{
	char buf[64];

	snprintf(buf, sizeof(buf), "%d", gpio);
	sysfs_write("/sys/class/gpio/unexport", buf);
}

/* Equivalent of gpio_direction_output(): atomic level set via direction. */
static int gpio_out(int gpio, int level)
{
	char path[80];

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
	return sysfs_write(path, level ? "high" : "low");
}

/* --------------------------------------------------------------- I2C */

static int i2c_open(void)
{
	char path[32];

	if (i2c_fd >= 0)
		return 0;
	snprintf(path, sizeof(path), "/dev/i2c-%d", bus_nr);
	i2c_fd = open(path, O_RDWR);
	if (i2c_fd < 0) {
		elog("cannot open %s: %s%s\n", path, strerror(errno),
		     errno == ENOENT ? " (kernel needs CONFIG_I2C_CHARDEV=y)" : "");
		return -1;
	}
	return 0;
}

static int i2c_xfer(struct i2c_msg *msgs, int n)
{
	struct i2c_rdwr_ioctl_data d = {.msgs = msgs, .nmsgs = n};

	return ioctl(i2c_fd, I2C_RDWR, &d) < 0 ? -1 : 0;
}

/*
 * Read one ID register. Register address width = id_addr_len bytes,
 * value width = id_value_len bytes, both big-endian on the wire,
 * exactly like the kernel module's sensor_read().
 */
static int sensor_read(const struct sensor_def *s, uint32_t addr, uint32_t *value)
{
	uint8_t buf[4] = {0}, data[4] = {0};
	uint8_t rlen = s->id_value_len;
	uint8_t wlen = s->id_addr_len;
	int i, ret;
	struct i2c_msg msg[2] = {
		{.addr = s->i2c_addr, .flags = 0, .len = wlen, .buf = buf},
		{.addr = s->i2c_addr, .flags = I2C_M_RD, .len = rlen, .buf = data},
	};

	if (wlen < 1 || wlen > 4 || rlen < 1 || rlen > 4) {
		elog("invalid reg/value width %u/%u\n", wlen, rlen);
		return -1;
	}
	for (i = 0; i < wlen; i++)
		buf[i] = (addr >> (8 * (wlen - 1 - i))) & 0xff;

	ret = i2c_xfer(msg, 2);

	*value = 0;
	for (i = 0; i < rlen; i++)
		*value = (*value << 8) | data[i];

	if (ret == 0)
		vlog("read 0x%x = 0x%x (addr 0x%02x)\n", addr, *value, s->i2c_addr);
	return ret;
}

/* 16-bit register, 8-bit value write, like the module's sensor_write(). */
static int sensor_write(const struct sensor_def *s, uint16_t reg, uint8_t value)
{
	uint8_t buf[3] = {(reg >> 8) & 0xff, reg & 0xff, value};
	struct i2c_msg msg = {.addr = s->i2c_addr, .flags = 0, .len = 3, .buf = buf};

	return i2c_xfer(&msg, 1);
}

/* ----------------------------------------------------------- probe logic */

static int sensor_matches_soc(const struct sensor_def *s)
{
	int is_t41 = strcmp(cur_soc->name, "t41") == 0;

	if (s->soc == S_T41_ONLY)
		return is_t41;
	if (s->soc == S_NOT_T41)
		return !is_t41;
	return 1;
}

/*
 * Reset/power-down dance for one probe attempt.
 * Faithful port of the sequences in process_one_adapter().
 */
static void sensor_hw_prepare(const struct sensor_def *s)
{
	if (reset_pin != -1) {
		if (gpio_claim(reset_pin) == 0) {
			gpio_out(reset_pin, 1);
			msleep(20);
			gpio_out(reset_pin, 0);
			if (!strcmp(s->name, "sp1409")) {
				msleep(600);
			} else if (!strcmp(s->name, "sc2336p") || !strcmp(s->name, "sc2337p") ||
				   !strcmp(s->name, "sc3336p")) {
				msleep(250);
				gpio_out(reset_pin, 1);
				msleep(20);
			} else {
				msleep(20);
				gpio_out(reset_pin, 1);
				msleep(20);
			}
		} else {
			elog("GPIO request failed for reset GPIO %d\n", reset_pin);
		}
	}
	if (pwdn_pin != -1) {
		if (gpio_claim(pwdn_pin) == 0) {
			gpio_out(pwdn_pin, 1);
			msleep(150);
			gpio_out(pwdn_pin, 0);
			if (!strcmp(s->name, "sp1409"))
				msleep(600);
			else
				msleep(10);
		} else {
			elog("GPIO request failed for pwdn GPIO %d\n", pwdn_pin);
		}
	}
}

static void sensor_hw_release(void)
{
	if (reset_pin != -1)
		gpio_release(reset_pin);
	if (pwdn_pin != -1)
		gpio_release(pwdn_pin);
}

/*
 * Probe every database entry. Faithful port of process_one_adapter():
 * per-sensor MCLK + reset dance, ID register compare with the sc2336p/
 * sc2337p/sc3336p unlock writes, the sc2336p-vs-sc2337p disambiguation
 * via reg 0x801e, and the ov2735b alternate-ID quirk.
 */
static int do_probe(void)
{
	unsigned i;
	int j;

	num_matches = 0;
	num_scan_results = 0;
	primary_idx = -1;

	for (i = 0; i < SENSOR_COUNT; i++) {
		const struct sensor_def *s = &sensor_db[i];
		struct i2c_scan_result scan_res;
		uint8_t idcnt = s->id_cnt;
		int ret;

		if (!sensor_matches_soc(s))
			continue;

		vlog("probing %s @ 0x%02x (MCLK %u Hz)\n", s->name, s->i2c_addr, s->clk);

		if (mclk_enable(s->clk) < 0)
			return -1;
		sensor_hw_prepare(s);

		memset(&scan_res, 0, sizeof(scan_res));
		scan_res.i2c_addr = s->i2c_addr;

		for (j = 0; j < idcnt; j++) {
			uint32_t value = 0;

			if (j == 0 &&
			    (!strcmp(s->name, "sc2336p") || !strcmp(s->name, "sc2337p"))) {
				ret = sensor_write(s, 0x301a, 0xf8);
				ret += sensor_write(s, 0x0100, 0x01);
				if (ret != 0)
					break;
				msleep(5);
			} else if (j == 0 && !strcmp(s->name, "sc3336p")) {
				ret = sensor_write(s, 0x440d, 0x10);
				ret += sensor_write(s, 0x4400, 0x11);
				if (ret != 0)
					break;
				msleep(10);
			}

			ret = sensor_read(s, s->id_addr[j], &value);

			if (scan_res.num_regs < 8) {
				scan_res.reg_addrs[scan_res.num_regs] = s->id_addr[j];
				scan_res.reg_values[scan_res.num_regs] = value;
				scan_res.num_regs++;
			}

			if (ret != 0)
				break;

			scan_res.responded = 1;

			if ((!strcmp(s->name, "sc2336p") || !strcmp(s->name, "sc2337p")) &&
			    j == 1) {
				uint32_t reg_val = 0;

				ret = sensor_read(s, 0x801e, &reg_val);
				if (ret != 0)
					break;
				sensor_write(s, 0x0100, 0x00);
				if (!strcmp(s->name, "sc2336p") && (reg_val & 0x0f) != 0) {
					j--;
					break;
				}
				if (!strcmp(s->name, "sc2337p") && (reg_val & 0x0f) == 0) {
					j--;
					break;
				}
			}

			if (!strcmp(s->name, "ov2735b") && j == 2) {
				if (value == s->id_value[j])
					j++;
			} else {
				if (value != s->id_value[j])
					break;
			}
		}

		sensor_hw_release();
		mclk_disable();

		if (j == idcnt) {
			strncpy(scan_res.sensor_name, s->name, sizeof(scan_res.sensor_name) - 1);

			if (num_matches < MAX_DETECTED_SENSORS) {
				match_res[num_matches] = scan_res;
				match_idx[num_matches++] = i;
				vlog("MATCH: %s, I2C bus %d, address 0x%02X\n", s->name, bus_nr,
				     s->i2c_addr);
			}
		}

		if (scan_res.responded && num_scan_results < MAX_I2C_SCAN_RESULTS)
			scan_results[num_scan_results++] = scan_res;
	}

	if (num_matches > 0)
		primary_idx = match_idx[0];

	return 0;
}

/* ---------------------------------------------------------------- report */

/*
 * Several table entries are rebadges with identical I2C address, ID
 * registers and ID values (e.g. gc5603/gc5613, gc2053/gc2063). One
 * physical chip matches all of them, so group matches whose observed
 * reads are identical and report one device with the others as aliases.
 */
static int same_device(const struct i2c_scan_result *a, const struct i2c_scan_result *b)
{
	int j;

	if (a->i2c_addr != b->i2c_addr || a->num_regs != b->num_regs)
		return 0;
	for (j = 0; j < a->num_regs; j++)
		if (a->reg_addrs[j] != b->reg_addrs[j] || a->reg_values[j] != b->reg_values[j])
			return 0;
	return 1;
}

static int num_devices;

static void print_report(void)
{
	int grp[MAX_DETECTED_SENSORS];
	int i, j, k, n;

	printf("========== Sensor Detection Report ==========\n\n");

	num_devices = 0;
	for (i = 0; i < num_matches; i++) {
		grp[i] = -1;
		for (j = 0; j < i; j++)
			if (same_device(&match_res[i], &match_res[j])) {
				grp[i] = grp[j];
				break;
			}
		if (grp[i] < 0)
			grp[i] = num_devices++;
	}

	if (num_matches > 0) {
		printf("MATCHED SENSORS (%d):\n", num_devices);
		printf("-------------------------------------------\n");
		for (n = 0; n < num_devices; n++) {
			const struct sensor_def *s = NULL;
			int aliases = 0;

			for (i = 0; i < num_matches; i++) {
				if (grp[i] != n)
					continue;
				if (!s) {
					s = &sensor_db[match_idx[i]];
					continue;
				}
				aliases++;
			}

			printf("%d. %s\n", n + 1, s->name);

			if (aliases) {
				printf("   Also matches: ");
				for (i = 0, k = 0; i < num_matches; i++) {
					if (grp[i] != n || &sensor_db[match_idx[i]] == s)
						continue;
					printf("%s%s", k++ ? ", " : "",
					       sensor_db[match_idx[i]].name);
				}
				printf(" (identical ID registers)\n");
			}

			printf("   I2C Bus: %d\n", bus_nr);
			printf("   I2C Address: 0x%02X\n", s->i2c_addr);
			printf("   Clock: %s @ %u Hz\n", s->mclk_name, s->clk);

			printf("   ID Registers: ");
			for (j = 0; j < s->id_cnt; j++)
				printf("0x%04X%s", s->id_addr[j], (j < s->id_cnt - 1) ? ", " : "");
			printf("\n");

			printf("   ID Values:    ");
			for (j = 0; j < s->id_cnt; j++)
				printf("0x%02X%s", s->id_value[j], (j < s->id_cnt - 1) ? ", " : "");
			printf("\n\n");
		}
		printf("Primary sensor: %s\n\n", sensor_db[primary_idx].name);
	} else {
		printf("MATCHED SENSORS: None\n\n");
	}

	if (num_scan_results > 0) {
		printf("ALL I2C DEVICES DETECTED (%d):\n", num_scan_results);
		printf("-------------------------------------------\n");
		for (i = 0; i < num_scan_results; i++) {
			struct i2c_scan_result *res = &scan_results[i];

			if (res->sensor_name[0] != '\0')
				printf("I2C Address 0x%02X: %s [MATCHED]\n", res->i2c_addr,
				       res->sensor_name);
			else
				printf("I2C Address 0x%02X: UNKNOWN DEVICE\n", res->i2c_addr);

			if (res->num_regs > 0) {
				printf("  Register reads:\n");
				for (j = 0; j < res->num_regs; j++)
					printf("    0x%04X = 0x%02X\n", res->reg_addrs[j],
					       res->reg_values[j]);
			}
			printf("\n");
		}
	} else {
		printf("I2C DEVICES DETECTED: None responded\n\n");
	}

	if (num_scan_results > num_matches) {
		printf("SETUP HELP FOR UNKNOWN DEVICES:\n");
		printf("-------------------------------------------\n");
		printf("Found %d device(s) that responded but didn't match known sensors.\n\n",
		       num_scan_results - num_matches);
		for (i = 0; i < num_scan_results; i++) {
			struct i2c_scan_result *res = &scan_results[i];

			if (res->sensor_name[0] != '\0')
				continue;
			printf("Device at I2C address 0x%02X:\n", res->i2c_addr);
			printf("  To add this sensor to the database, you need:\n");
			printf("  1. Sensor model name\n");
			printf("  2. ID register addresses (tried: ");
			for (j = 0; j < res->num_regs; j++)
				printf("0x%04X%s", res->reg_addrs[j],
				       (j < res->num_regs - 1) ? ", " : "");
			printf(")\n");
			printf("  3. Expected ID values (read: ");
			for (j = 0; j < res->num_regs; j++)
				printf("0x%02X%s", res->reg_values[j],
				       (j < res->num_regs - 1) ? ", " : "");
			printf(")\n");
			printf("  4. Clock frequency (currently using default)\n\n");
		}
	}

	printf("CONFIGURATION:\n");
	printf("-------------------------------------------\n");
	printf("SoC: %s\n", cur_soc->name);
	printf("I2C Adapter: %d\n", bus_nr);
	printf("Reset GPIO: %d\n", reset_pin);
	printf("Power Down GPIO: %d\n", pwdn_pin);
	printf("\n");
	printf("Detected sensors: %d\n", num_devices);
	printf("=============================================\n");
}

/* ------------------------------------------------------------- commands */

/* Port of the module's sensor_open(): MCLK on + generic reset dance. */
static int do_open(const char *name)
{
	const struct sensor_def *s = NULL;
	unsigned i;

	for (i = 0; i < SENSOR_COUNT; i++) {
		if (sensor_matches_soc(&sensor_db[i]) && !strcmp(sensor_db[i].name, name)) {
			s = &sensor_db[i];
			break;
		}
	}
	if (!s) {
		elog("sensor '%s' not found\n", name);
		return -1;
	}

	if (mclk_enable(s->clk) < 0)
		return -1;

	if (reset_pin != -1) {
		if (gpio_claim(reset_pin) == 0) {
			gpio_out(reset_pin, 1);
			msleep(20);
			gpio_out(reset_pin, 0);
			msleep(20);
			gpio_out(reset_pin, 1);
			msleep(20);
		} else {
			elog("GPIO request failed for reset GPIO %d\n", reset_pin);
		}
	}
	if (pwdn_pin != -1) {
		if (gpio_claim(pwdn_pin) == 0) {
			gpio_out(pwdn_pin, 1);
			msleep(150);
			gpio_out(pwdn_pin, 0);
			msleep(10);
		} else {
			elog("GPIO request failed for pwdn GPIO %d\n", pwdn_pin);
		}
	}

	printf("%s: MCLK %u Hz enabled, reset released\n", s->name, s->clk);
	return 0;
}

static int do_release(void)
{
	sensor_hw_release();
	mclk_disable();
	printf("MCLK stopped, GPIOs released\n");
	return 0;
}

/* Port of the module's i2c-r/i2c-w proc commands (raw single transfers). */
static int do_raw_i2c(int is_write, uint32_t addr, uint32_t data, int len)
{
	uint8_t buf[4] = {0};
	uint32_t value = 0;
	int i;
	struct i2c_msg msg = {
		.addr = addr,
		.flags = is_write ? 0 : I2C_M_RD,
		.len = len,
		.buf = buf,
	};

	if (len < 1 || len > 4) {
		elog("invalid length %d\n", len);
		return -1;
	}
	if (is_write)
		for (i = 0; i < len; i++)
			buf[i] = (data >> (8 * (len - 1 - i))) & 0xff;

	if (i2c_xfer(&msg, 1) != 0) {
		elog("I2C transfer failed\n");
		return -1;
	}
	if (!is_write) {
		for (i = 0; i < len; i++)
			value = (value << 8) | buf[i];
		printf("I2C read from address 0x%x: 0x%x\n", addr, value);
	}
	return 0;
}

/* -------------------------------------------------------------- SoC pick */

static const struct soc_desc *soc_by_name(const char *name)
{
	unsigned i;

	for (i = 0; i < SOC_COUNT; i++)
		if (!strcasecmp(soc_table[i].name, name))
			return &soc_table[i];
	return NULL;
}

static void str_tolower(char *s)
{
	for (; *s; s++)
		*s = tolower((unsigned char)*s);
}

/*
 * Detection chain:
 * 1. isvp_<codename> in uname -r (XBurst1 vendor kernels, e.g. isvp_swan)
 * 2. /proc/cpuinfo: "system type" codename, or the machine name for
 *    XBurst2 built-in DTs (T41 vendor DT is ingenic,marmot)
 * 3. thingino's `soc -f` tool
 */
static const struct soc_desc *soc_autodetect(void)
{
	struct utsname u;
	unsigned i;
	char buf[256];
	FILE *f;

	if (uname(&u) == 0) {
		str_tolower(u.release);
		for (i = 0; i < SOC_COUNT; i++)
			if (soc_table[i].uname_tag && strstr(u.release, soc_table[i].uname_tag))
				return &soc_table[i];
	}

	f = fopen("/proc/cpuinfo", "r");
	if (f) {
		while (fgets(buf, sizeof(buf), f)) {
			if (strncmp(buf, "system type", 11) && strncmp(buf, "machine", 7))
				continue;
			str_tolower(buf);
			for (i = 0; i < SOC_COUNT; i++)
				if (soc_table[i].uname_tag && strstr(buf, soc_table[i].uname_tag)) {
					fclose(f);
					return &soc_table[i];
				}
			/* XBurst2 vendor DT machine names */
			if (strstr(buf, "marmot")) {
				fclose(f);
				return soc_by_name("t41");
			}
			if (strstr(buf, "shark")) {
				fclose(f);
				return soc_by_name("t40");
			}
		}
		fclose(f);
	}

	f = popen("soc -f 2>/dev/null", "r");
	if (f) {
		if (fgets(buf, sizeof(buf), f)) {
			buf[strcspn(buf, "\r\n")] = '\0';
			pclose(f);
			return soc_by_name(buf);
		}
		pclose(f);
	}
	return NULL;
}

/* ------------------------------------------------------------------ main */

static void usage(void)
{
	fprintf(stderr,
		"sinfo - userspace Ingenic image sensor detector\n"
		"\n"
		"usage: sinfo [-s soc] [-b bus] [-r gpio] [-p gpio] [-v] [command]\n"
		"\n"
		"commands:\n"
		"  probe                      scan for sensors, print report (default)\n"
		"  open <sensor>              enable MCLK + reset for named sensor\n"
		"  release                    stop MCLK, free GPIOs\n"
		"  i2c-r <addr> <len>         raw I2C read\n"
		"  i2c-w <addr> <data> <len>  raw I2C write\n"
		"\n"
		"options:\n"
		"  -s <soc>   t10 t20 t21 t23 t30 t31 c100 t40 t41 (default: auto from uname)\n"
		"  -b <bus>   I2C bus number (default: SoC default)\n"
		"  -r <gpio>  reset GPIO (default: SoC default; -1 = none)\n"
		"  -p <gpio>  power-down GPIO (default: -1)\n"
		"  -v         verbose\n");
}

int main(int argc, char **argv)
{
	const char *soc_name = NULL;
	const char *cmd = "probe";
	int opt, ret = 0;

	while ((opt = getopt(argc, argv, "s:b:r:p:vh")) != -1) {
		switch (opt) {
		case 's':
			soc_name = optarg;
			break;
		case 'b':
			bus_nr = atoi(optarg);
			break;
		case 'r':
			reset_pin = atoi(optarg);
			break;
		case 'p':
			pwdn_pin = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		case 'h':
			usage();
			return 0;
		default:
			usage();
			return 2;
		}
	}
	if (optind < argc)
		cmd = argv[optind];

	if (soc_name) {
		cur_soc = soc_by_name(soc_name);
		if (!cur_soc) {
			elog("unknown SoC '%s'\n", soc_name);
			return 2;
		}
	} else {
		cur_soc = soc_autodetect();
		if (!cur_soc) {
			elog("cannot auto-detect SoC, use -s\n");
			return 2;
		}
		vlog("auto-detected SoC: %s\n", cur_soc->name);
	}

	if (!cur_soc->tested)
		fprintf(stderr,
			"sinfo: [Warning] SoC %s support is not yet "
			"hardware-validated\n",
			cur_soc->name);

	if (bus_nr == -1)
		bus_nr = cur_soc->i2c_bus;
	if (reset_pin == -9999)
		reset_pin = cur_soc->reset_gpio;

	if (geteuid() != 0)
		fprintf(stderr, "sinfo: [Warning] not running as root, "
				"/dev/mem and GPIO access will likely fail\n");

	if (cpm_init())
		return 2;
	if (i2c_open())
		return 2;

	if (!strcmp(cur_soc->name, "t21"))
		t21_init_quirk();
	xb2_mclk_pin_mux();

	if (!strcmp(cmd, "probe") || !strcmp(cmd, "1")) {
		if (do_probe() < 0)
			return 2;
		print_report();
		ret = num_matches > 0 ? 0 : 1;
	} else if (!strcmp(cmd, "open")) {
		if (optind + 1 >= argc) {
			elog("open needs a sensor name\n");
			return 2;
		}
		ret = do_open(argv[optind + 1]) ? 2 : 0;
	} else if (!strcmp(cmd, "release")) {
		ret = do_release() ? 2 : 0;
	} else if (!strcmp(cmd, "i2c-r")) {
		if (optind + 2 >= argc) {
			elog("i2c-r needs <addr> <len>\n");
			return 2;
		}
		ret = do_raw_i2c(0, strtoul(argv[optind + 1], NULL, 0), 0, atoi(argv[optind + 2]))
			      ? 2
			      : 0;
	} else if (!strcmp(cmd, "i2c-w")) {
		if (optind + 3 >= argc) {
			elog("i2c-w needs <addr> <data> <len>\n");
			return 2;
		}
		ret = do_raw_i2c(1, strtoul(argv[optind + 1], NULL, 0),
				 strtoul(argv[optind + 2], NULL, 0), atoi(argv[optind + 3]))
			      ? 2
			      : 0;
	} else {
		elog("unknown command '%s'\n", cmd);
		usage();
		return 2;
	}

	return ret;
}
