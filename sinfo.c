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
 *   -s <soc>    SoC (t10/t20/t21/t23/t30/t31/c100); default: auto-detect
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

#define EXTAL_HZ 24000000ull

/* ---------------------------------------------------------------- SoC data */

/*
 * All XBurst1 SoCs: CPM at 0x10000000, CIMCDR at +0x7c with
 * CE bit 29, BUSY bit 28, STOP bit 27, 8-bit divider.
 * Parent mux field and PLL register format differ per SoC
 * (source: ingenic-u-boot-xburst1 per-SoC clk.c/cpm.h).
 */
#define CPM_PHYS	0x10000000
#define GPIO_PHYS	0x10010000
#define CIMCDR_OFF	0x7c
#define CE_BIT		29
#define BUSY_BIT	28
#define STOP_BIT	27
#define DIV_MASK	0xff

enum { PLL_NONE = 0, PLL_A, PLL_M, PLL_V, PLL_E };

/* rate = EXTAL * m/n/od0/od1        m[31:20] n[19:14] od1[13:11] od0[10:8] */
#define PLLSTYLE_NEW 0
/* rate = EXTAL * 2*(m+1)/(n+1)/2^od m[28:20] n[19:14] od[13:11] */
#define PLLSTYLE_OLD 1

struct soc_desc {
	const char *name;
	const char *uname_tag;	/* "isvp_<tag>" in uname -r, NULL = unknown */
	int pll_style;
	uint8_t mux_shift;
	uint8_t mux_width;
	uint8_t parent[4];	/* PLL per mux field value */
	uint8_t default_mux;	/* used when current parent is off/invalid */
	int i2c_bus;
	int reset_gpio;
	int tested;		/* HW-validated */
};

static const struct soc_desc soc_table[] = {
	{ "t31",  "swan", PLLSTYLE_NEW, 30, 2, {PLL_A, PLL_M, PLL_V, PLL_NONE}, 2, 0, 18, 1 },
	{ "t23",  "pike", PLLSTYLE_NEW, 30, 2, {PLL_A, PLL_M, PLL_NONE, PLL_NONE}, 1, 0, 18, 0 },
	{ "c100", NULL,   PLLSTYLE_NEW, 30, 2, {PLL_A, PLL_M, PLL_V, PLL_NONE}, 2, 0, 18, 0 },
	{ "t20",  NULL,   PLLSTYLE_NEW, 30, 2, {PLL_A, PLL_M, PLL_V, PLL_V}, 2, 0, 18, 0 },
	{ "t30",  NULL,   PLLSTYLE_OLD, 30, 2, {PLL_A, PLL_M, PLL_V, PLL_E}, 2, 0, 18, 0 },
	{ "t21",  NULL,   PLLSTYLE_OLD, 30, 2, {PLL_A, PLL_M, PLL_V, PLL_E}, 2, 0, 18, 0 },
	{ "t10",  NULL,   PLLSTYLE_NEW, 31, 1, {PLL_A, PLL_M, PLL_NONE, PLL_NONE}, 1, 0, 18, 0 },
};
#define SOC_COUNT (sizeof(soc_table)/sizeof(soc_table[0]))

/* ------------------------------------------------------------------ state */

static const struct soc_desc *g_soc;
static int g_bus = -1;		/* -1 = use SoC default */
static int g_reset_gpio = -9999;
static int g_pwdn_gpio = -1;
static int g_verbose;

static int g_i2c_fd = -1;
static volatile uint32_t *g_cpm;

static int8_t g_sensor_id = -1;
static int g_sensor_ids[MAX_DETECTED_SENSORS];
static int g_num_detected_sensors;

struct i2c_scan_result {
	uint8_t i2c_addr;
	uint8_t responded;
	uint32_t reg_values[8];
	uint32_t reg_addrs[8];
	uint8_t num_regs;
	char sensor_name[32];
};
static struct i2c_scan_result g_scan_results[MAX_I2C_SCAN_RESULTS];
static int g_num_scan_results;

#define vlog(...) do { if (g_verbose) fprintf(stderr, "sinfo: " __VA_ARGS__); } while (0)
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
	if (g_cpm)
		return 0;
	g_cpm = map_phys(CPM_PHYS, 0x1000);
	return g_cpm ? 0 : -1;
}

static uint32_t cpm_rd(uint32_t off)
{
	return g_cpm[off / 4];
}

static void cpm_wr(uint32_t off, uint32_t val)
{
	g_cpm[off / 4] = val;
}

static int cim_wait_not_busy(void)
{
	int i;

	for (i = 0; i < 10000; i++) {
		if (!(cpm_rd(CIMCDR_OFF) & (1u << BUSY_BIT)))
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
	case PLL_A: off = 0x10; break;
	case PLL_M: off = 0x14; break;
	case PLL_V: off = 0xe0; break;
	case PLL_E: off = 0x58; break;
	default: return 0;
	}
	r = cpm_rd(off);

	if (g_soc->pll_style == PLLSTYLE_NEW) {
		uint32_t od1, od0;

		if (!(r & 1))	/* PLL not on */
			return 0;
		m = (r >> 20) & 0xfff;
		n = (r >> 14) & 0x3f;
		od1 = (r >> 11) & 0x7;
		od0 = (r >> 8) & 0x7;
		if (!n || !od0 || !od1)
			return 0;
		return EXTAL_HZ * m / n / od0 / od1;
	} else {
		uint32_t od;

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

	v = cpm_rd(CIMCDR_OFF);
	muxmask = (1u << g_soc->mux_width) - 1;
	mux = (v >> g_soc->mux_shift) & muxmask;
	prate = pll_rate(g_soc->parent[mux]);
	if (!prate) {
		/* unprogrammed/parked CDR: fall back to the SoC's default parent */
		mux = g_soc->default_mux;
		prate = pll_rate(g_soc->parent[mux]);
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
	nv = v & ~(muxmask << g_soc->mux_shift) & ~DIV_MASK & ~(1u << STOP_BIT);
	nv |= (mux << g_soc->mux_shift) | (div - 1) | (1u << CE_BIT);
	cpm_wr(CIMCDR_OFF, nv);
	if (cim_wait_not_busy())
		return -1;
	cpm_wr(CIMCDR_OFF, nv & ~(1u << CE_BIT));

	vlog("MCLK: parent %llu Hz / %u = %llu Hz (CIMCDR 0x%08x)\n",
	     (unsigned long long)prate, div,
	     (unsigned long long)(prate / div), cpm_rd(CIMCDR_OFF));
	return 0;
}

static void mclk_disable(void)
{
	uint32_t v;

	if (cim_wait_not_busy())
		return;
	v = cpm_rd(CIMCDR_OFF) | (1u << CE_BIT) | (1u << STOP_BIT);
	cpm_wr(CIMCDR_OFF, v);
	cim_wait_not_busy();
	cpm_wr(CIMCDR_OFF, v & ~(1u << CE_BIT));
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
		return 0;	/* already exported (e.g. by a previous run) */

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

	if (g_i2c_fd >= 0)
		return 0;
	snprintf(path, sizeof(path), "/dev/i2c-%d", g_bus);
	g_i2c_fd = open(path, O_RDWR);
	if (g_i2c_fd < 0) {
		elog("cannot open %s: %s%s\n", path, strerror(errno),
		     errno == ENOENT ?
		     " (kernel needs CONFIG_I2C_CHARDEV=y)" : "");
		return -1;
	}
	return 0;
}

static int i2c_xfer(struct i2c_msg *msgs, int n)
{
	struct i2c_rdwr_ioctl_data d = { .msgs = msgs, .nmsgs = n };

	return ioctl(g_i2c_fd, I2C_RDWR, &d) < 0 ? -1 : 0;
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
		{ .addr = s->i2c_addr, .flags = 0,        .len = wlen, .buf = buf },
		{ .addr = s->i2c_addr, .flags = I2C_M_RD, .len = rlen, .buf = data },
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
	uint8_t buf[3] = { (reg >> 8) & 0xff, reg & 0xff, value };
	struct i2c_msg msg = { .addr = s->i2c_addr, .flags = 0, .len = 3, .buf = buf };

	return i2c_xfer(&msg, 1);
}

/* ----------------------------------------------------------- probe logic */

static int sensor_matches_soc(const struct sensor_def *s)
{
	int is_t41 = strcmp(g_soc->name, "t41") == 0;

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
	if (g_reset_gpio != -1) {
		if (gpio_claim(g_reset_gpio) == 0) {
			gpio_out(g_reset_gpio, 1);
			msleep(20);
			gpio_out(g_reset_gpio, 0);
			if (!strcmp(s->name, "sp1409")) {
				msleep(600);
			} else if (!strcmp(s->name, "sc2336p") ||
				   !strcmp(s->name, "sc2337p") ||
				   !strcmp(s->name, "sc3336p")) {
				msleep(250);
				gpio_out(g_reset_gpio, 1);
				msleep(20);
			} else {
				msleep(20);
				gpio_out(g_reset_gpio, 1);
				msleep(20);
			}
		} else {
			elog("GPIO request failed for reset GPIO %d\n", g_reset_gpio);
		}
	}
	if (g_pwdn_gpio != -1) {
		if (gpio_claim(g_pwdn_gpio) == 0) {
			gpio_out(g_pwdn_gpio, 1);
			msleep(150);
			gpio_out(g_pwdn_gpio, 0);
			if (!strcmp(s->name, "sp1409"))
				msleep(600);
			else
				msleep(10);
		} else {
			elog("GPIO request failed for pwdn GPIO %d\n", g_pwdn_gpio);
		}
	}
}

static void sensor_hw_release(void)
{
	if (g_reset_gpio != -1)
		gpio_release(g_reset_gpio);
	if (g_pwdn_gpio != -1)
		gpio_release(g_pwdn_gpio);
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

	g_num_detected_sensors = 0;
	g_num_scan_results = 0;
	g_sensor_id = -1;

	for (i = 0; i < SENSOR_COUNT; i++) {
		const struct sensor_def *s = &g_sinfo[i];
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

			if (j == 0 && (!strcmp(s->name, "sc2336p") ||
				       !strcmp(s->name, "sc2337p"))) {
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

			if ((!strcmp(s->name, "sc2336p") ||
			     !strcmp(s->name, "sc2337p")) && j == 1) {
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
			strncpy(scan_res.sensor_name, s->name,
				sizeof(scan_res.sensor_name) - 1);

			if (g_num_detected_sensors < MAX_DETECTED_SENSORS) {
				g_sensor_ids[g_num_detected_sensors++] = i;
				vlog("MATCH: %s, I2C bus %d, address 0x%02X\n",
				     s->name, g_bus, s->i2c_addr);
			}
		}

		if (scan_res.responded && g_num_scan_results < MAX_I2C_SCAN_RESULTS)
			g_scan_results[g_num_scan_results++] = scan_res;
	}

	if (g_num_detected_sensors > 0)
		g_sensor_id = g_sensor_ids[0];

	return 0;
}

/* ---------------------------------------------------------------- report */

static void print_report(void)
{
	int i, j;

	printf("========== Sensor Detection Report ==========\n\n");

	if (g_num_detected_sensors > 0) {
		printf("MATCHED SENSORS (%d):\n", g_num_detected_sensors);
		printf("-------------------------------------------\n");
		for (i = 0; i < g_num_detected_sensors; i++) {
			const struct sensor_def *s = &g_sinfo[g_sensor_ids[i]];

			printf("%d. %s\n", i + 1, s->name);
			printf("   I2C Bus: %d\n", g_bus);
			printf("   I2C Address: 0x%02X\n", s->i2c_addr);
			printf("   Clock: %s @ %u Hz\n", s->mclk_name, s->clk);

			printf("   ID Registers: ");
			for (j = 0; j < s->id_cnt; j++)
				printf("0x%04X%s", s->id_addr[j],
				       (j < s->id_cnt - 1) ? ", " : "");
			printf("\n");

			printf("   ID Values:    ");
			for (j = 0; j < s->id_cnt; j++)
				printf("0x%02X%s", s->id_value[j],
				       (j < s->id_cnt - 1) ? ", " : "");
			printf("\n\n");
		}
		printf("Primary sensor: %s\n\n", g_sinfo[g_sensor_id].name);
	} else {
		printf("MATCHED SENSORS: None\n\n");
	}

	if (g_num_scan_results > 0) {
		printf("ALL I2C DEVICES DETECTED (%d):\n", g_num_scan_results);
		printf("-------------------------------------------\n");
		for (i = 0; i < g_num_scan_results; i++) {
			struct i2c_scan_result *res = &g_scan_results[i];

			if (res->sensor_name[0] != '\0')
				printf("I2C Address 0x%02X: %s [MATCHED]\n",
				       res->i2c_addr, res->sensor_name);
			else
				printf("I2C Address 0x%02X: UNKNOWN DEVICE\n",
				       res->i2c_addr);

			if (res->num_regs > 0) {
				printf("  Register reads:\n");
				for (j = 0; j < res->num_regs; j++)
					printf("    0x%04X = 0x%02X\n",
					       res->reg_addrs[j], res->reg_values[j]);
			}
			printf("\n");
		}
	} else {
		printf("I2C DEVICES DETECTED: None responded\n\n");
	}

	if (g_num_scan_results > g_num_detected_sensors) {
		printf("SETUP HELP FOR UNKNOWN DEVICES:\n");
		printf("-------------------------------------------\n");
		printf("Found %d device(s) that responded but didn't match known sensors.\n\n",
		       g_num_scan_results - g_num_detected_sensors);
		for (i = 0; i < g_num_scan_results; i++) {
			struct i2c_scan_result *res = &g_scan_results[i];

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
	printf("SoC: %s\n", g_soc->name);
	printf("I2C Adapter: %d\n", g_bus);
	printf("Reset GPIO: %d\n", g_reset_gpio);
	printf("Power Down GPIO: %d\n", g_pwdn_gpio);
	printf("\n");
	printf("Detected sensors: %d\n", g_num_detected_sensors);
	printf("=============================================\n");
}

/* ------------------------------------------------------------- commands */

/* Port of the module's sensor_open(): MCLK on + generic reset dance. */
static int do_open(const char *name)
{
	const struct sensor_def *s = NULL;
	unsigned i;

	for (i = 0; i < SENSOR_COUNT; i++) {
		if (sensor_matches_soc(&g_sinfo[i]) &&
		    !strcmp(g_sinfo[i].name, name)) {
			s = &g_sinfo[i];
			break;
		}
	}
	if (!s) {
		elog("sensor '%s' not found\n", name);
		return -1;
	}

	if (mclk_enable(s->clk) < 0)
		return -1;

	if (g_reset_gpio != -1) {
		if (gpio_claim(g_reset_gpio) == 0) {
			gpio_out(g_reset_gpio, 1);
			msleep(20);
			gpio_out(g_reset_gpio, 0);
			msleep(20);
			gpio_out(g_reset_gpio, 1);
			msleep(20);
		} else {
			elog("GPIO request failed for reset GPIO %d\n", g_reset_gpio);
		}
	}
	if (g_pwdn_gpio != -1) {
		if (gpio_claim(g_pwdn_gpio) == 0) {
			gpio_out(g_pwdn_gpio, 1);
			msleep(150);
			gpio_out(g_pwdn_gpio, 0);
			msleep(10);
		} else {
			elog("GPIO request failed for pwdn GPIO %d\n", g_pwdn_gpio);
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

static const struct soc_desc *soc_autodetect(void)
{
	struct utsname u;
	unsigned i;
	char low[sizeof(u.release)];
	char *p;

	if (uname(&u) == 0) {
		for (p = u.release; *p; p++)
			low[p - u.release] = tolower((unsigned char)*p);
		low[p - u.release] = '\0';
		for (i = 0; i < SOC_COUNT; i++)
			if (soc_table[i].uname_tag && strstr(low, soc_table[i].uname_tag))
				return &soc_table[i];
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
"  -s <soc>   t10 t20 t21 t23 t30 t31 c100 (default: auto from uname)\n"
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
		case 's': soc_name = optarg; break;
		case 'b': g_bus = atoi(optarg); break;
		case 'r': g_reset_gpio = atoi(optarg); break;
		case 'p': g_pwdn_gpio = atoi(optarg); break;
		case 'v': g_verbose = 1; break;
		case 'h': usage(); return 0;
		default: usage(); return 2;
		}
	}
	if (optind < argc)
		cmd = argv[optind];

	if (soc_name) {
		g_soc = soc_by_name(soc_name);
		if (!g_soc) {
			elog("unknown SoC '%s'\n", soc_name);
			return 2;
		}
	} else {
		g_soc = soc_autodetect();
		if (!g_soc) {
			elog("cannot auto-detect SoC, use -s\n");
			return 2;
		}
		vlog("auto-detected SoC: %s\n", g_soc->name);
	}

	if (!g_soc->tested)
		fprintf(stderr, "sinfo: [Warning] SoC %s support is not yet "
			"hardware-validated\n", g_soc->name);

	if (g_bus == -1)
		g_bus = g_soc->i2c_bus;
	if (g_reset_gpio == -9999)
		g_reset_gpio = g_soc->reset_gpio;

	if (geteuid() != 0)
		fprintf(stderr, "sinfo: [Warning] not running as root, "
			"/dev/mem and GPIO access will likely fail\n");

	if (cpm_init())
		return 2;
	if (i2c_open())
		return 2;

	if (!strcmp(g_soc->name, "t21"))
		t21_init_quirk();

	if (!strcmp(cmd, "probe") || !strcmp(cmd, "1")) {
		if (do_probe() < 0)
			return 2;
		print_report();
		ret = g_num_detected_sensors > 0 ? 0 : 1;
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
		ret = do_raw_i2c(0, strtoul(argv[optind + 1], NULL, 0), 0,
				 atoi(argv[optind + 2])) ? 2 : 0;
	} else if (!strcmp(cmd, "i2c-w")) {
		if (optind + 3 >= argc) {
			elog("i2c-w needs <addr> <data> <len>\n");
			return 2;
		}
		ret = do_raw_i2c(1, strtoul(argv[optind + 1], NULL, 0),
				 strtoul(argv[optind + 2], NULL, 0),
				 atoi(argv[optind + 3])) ? 2 : 0;
	} else {
		elog("unknown command '%s'\n", cmd);
		usage();
		return 2;
	}

	return ret;
}
