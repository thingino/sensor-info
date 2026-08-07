// SPDX-License-Identifier: GPL-2.0
/*
 * test_sinfo.c - host-side unit tests (run under ASAN+UBSan)
 *
 * Includes sinfo.c directly to reach its statics and links mock_hw.c
 * as the hardware. PLL vectors are live register values captured from
 * the hardware-validated bench boards, with the rates the silicon was
 * actually running at.
 */
#include <fcntl.h>
#include <unistd.h>

#include "mock_hw.h"

#define main sinfo_main
#include "../sinfo.c"
#undef main

static int failures;
static int checks;

#define CHECK(cond)                                                                                \
	do {                                                                                       \
		checks++;                                                                          \
		if (!(cond)) {                                                                     \
			failures++;                                                                \
			fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
		}                                                                                  \
	} while (0)

#define CHECK_EQ(a, b)                                                                             \
	do {                                                                                       \
		checks++;                                                                          \
		unsigned long long _a = (unsigned long long)(a);                                   \
		unsigned long long _b = (unsigned long long)(b);                                   \
		if (_a != _b) {                                                                    \
			failures++;                                                                \
			fprintf(stderr, "FAIL %s:%d: %s = %llu, want %llu (%s)\n", __FILE__,       \
				__LINE__, #a, _a, _b, #b);                                         \
		}                                                                                  \
	} while (0)

static void use_soc(const char *name)
{
	cur_soc = soc_by_name(name);
	if (!cur_soc)
		abort();
	cur_mclk = &cur_soc->mclk[cur_soc->default_mclk];
	bus_nr = cur_soc->i2c_bus;
	bus_all = 0;
	reset_pin = -1;
	pwdn_pin = -1;
}

/* run print_report capturing stdout, return 1 if needle appears */
static int report_contains(const char *needle)
{
	char buf[16384] = {0};
	int saved = dup(1);
	FILE *tmp = tmpfile();

	fflush(stdout);
	dup2(fileno(tmp), 1);
	print_report();
	fflush(stdout);
	dup2(saved, 1);
	close(saved);
	rewind(tmp);
	fread(buf, 1, sizeof(buf) - 1, tmp);
	fclose(tmp);
	return strstr(buf, needle) != NULL;
}

/* run print_report with stdout thrown away (it computes num_devices) */
static void silent_report(void)
{
	int saved = dup(1);
	int devnull = open("/dev/null", O_WRONLY);

	fflush(stdout);
	dup2(devnull, 1);
	print_report();
	fflush(stdout);
	dup2(saved, 1);
	close(saved);
	close(devnull);
}

static const char *primary_name(void)
{
	return primary_idx >= 0 ? sensor_db[primary_idx].name : "(none)";
}

/* ------------------------------------------------- PLL decode, live vectors */

static void test_pll_new_style(void)
{
	use_soc("t31"); /* wyze cam3, VPLL parent, 24 MHz sensor */
	mock_cpm[0x10 / 4] = 0x0740510D;
	mock_cpm[0x14 / 4] = 0x0640510D;
	mock_cpm[0xe0 / 4] = 0x0640510D;
	CHECK_EQ(pll_rate(PLL_A), 1392000u);
	CHECK_EQ(pll_rate(PLL_M), 1200000u);
	CHECK_EQ(pll_rate(PLL_V), 1200000u);

	use_soc("t20"); /* wyze v2: VPLL parked, kernel uses APLL */
	mock_cpm[0x10 / 4] = 0x0470890D;
	mock_cpm[0x14 / 4] = 0x07D0C90D;
	mock_cpm[0xe0 / 4] = 0x010049C0; /* bit0 clear = off */
	CHECK_EQ(pll_rate(PLL_A), 852000u);
	CHECK_EQ(pll_rate(PLL_M), 1000000u);
	CHECK_EQ(pll_rate(PLL_V), 0u);

	use_soc("t40"); /* t40nn: EPLL parent for CIM1 */
	mock_cpm[0x10 / 4] = 0x04C0510D;
	mock_cpm[0x14 / 4] = 0x1130990D;
	mock_cpm[0xe0 / 4] = 0x0540510D;
	mock_cpm[0x58 / 4] = 0x1290A10D;
	CHECK_EQ(pll_rate(PLL_A), 912000u);
	CHECK_EQ(pll_rate(PLL_M), 1100000u);
	CHECK_EQ(pll_rate(PLL_V), 1008000u);
	CHECK_EQ(pll_rate(PLL_E), 891000u);

	/* n = 0 must not divide by zero */
	mock_cpm[0x10 / 4] = 0x07400101;
	CHECK_EQ(pll_rate(PLL_A), 0u);
}

static void test_pll_t41_style(void)
{
	use_soc("t41");
	mock_cpm[0x10 / 4] = 0x05B008BD; /* t41nq APLL */
	mock_cpm[0x14 / 4] = 0x15D0889D; /* t41nq MPLL */
	mock_cpm[0xe0 / 4] = 0x059008BD; /* t41nq VPLL */
	CHECK_EQ(pll_rate(PLL_A), 1104000u);
	CHECK_EQ(pll_rate(PLL_M), 1400000u);
	CHECK_EQ(pll_rate(PLL_V), 1080000u);

	mock_cpm[0x14 / 4] = 0x063008BD; /* t41lq MPLL */
	CHECK_EQ(pll_rate(PLL_M), 1200000u);

	mock_cpm[0x10 / 4] = 0x05B008BC; /* off bit */
	CHECK_EQ(pll_rate(PLL_A), 0u);
}

static void test_pll_old_style(void)
{
	use_soc("t30"); /* wyze vdb1 */
	mock_cpm[0x10 / 4] = 0x04A0484D;
	mock_cpm[0x14 / 4] = 0x07C0882D;
	mock_cpm[0xe0 / 4] = 0x0310086D;
	mock_cpm[0x58 / 4] = 0x06305040; /* EPLL parked */
	CHECK_EQ(pll_rate(PLL_A), 900000u);
	CHECK_EQ(pll_rate(PLL_M), 1000000u);
	CHECK_EQ(pll_rate(PLL_V), 1200000u);
	CHECK_EQ(pll_rate(PLL_E), 0u);

	use_soc("t21"); /* t21n */
	mock_cpm[0x10 / 4] = 0x0470484D;
	mock_cpm[0x14 / 4] = 0x04A0484D;
	CHECK_EQ(pll_rate(PLL_A), 864000u);
	CHECK_EQ(pll_rate(PLL_M), 900000u);
}

/* --------------------------------------------------------- MCLK programming */

static void test_mclk_divider_and_handshake(void)
{
	uint32_t final;
	int first = -1, last = -1, i;

	mock_reset();
	use_soc("t31");
	mock_cpm[0xe0 / 4] = 0x0640510D; /* VPLL 1200 MHz */
	mock_cpm[0x7c / 4] = 0x88000031; /* gated, VPLL parent, div 50 */

	CHECK_EQ(mclk_enable(24000000), 0);
	final = mock_cpm[0x7c / 4];
	CHECK_EQ((final >> 30) & 3, 2); /* parent preserved */
	CHECK_EQ(final & 0xff, 49);	/* 1200/24 = 50 */
	CHECK_EQ((final >> 27) & 1, 0); /* running */
	CHECK_EQ((final >> 29) & 1, 0); /* CE dropped */

	for (i = 0; i < mock_cpm_nwrites; i++)
		if (mock_cpm_writes[i].off == 0x7c) {
			if (first < 0)
				first = i;
			last = i;
		}
	CHECK(first >= 0 && last > first);
	CHECK_EQ((mock_cpm_writes[first].val >> 29) & 1, 1); /* CE high first */
	CHECK_EQ((mock_cpm_writes[last].val >> 29) & 1, 0);  /* CE low last */

	CHECK_EQ(mclk_enable(37125000), 0);
	CHECK_EQ(mock_cpm[0x7c / 4] & 0xff, 32); /* ceil(1200/37.125) = 33 */

	mclk_disable();
	CHECK_EQ((mock_cpm[0x7c / 4] >> 27) & 1, 1); /* gated again */
}

static void test_mclk_parent_fallback(void)
{
	mock_reset();
	use_soc("t20");
	/* CDR points at VPLL, VPLL and the default are dead, APLL alive */
	mock_cpm[0x10 / 4] = 0x0470890D; /* APLL 852 MHz */
	mock_cpm[0xe0 / 4] = 0x010049C0; /* VPLL off */
	mock_cpm[0x7c / 4] = 0x80000000; /* mux = VPLL */

	CHECK_EQ(mclk_enable(24000000), 0);
	CHECK_EQ((mock_cpm[0x7c / 4] >> 30) & 3, 0); /* fell back to APLL */
	CHECK_EQ(mock_cpm[0x7c / 4] & 0xff, 35);     /* ceil(852/24) = 36 */
}

/* -------------------------------------------------------------- probe logic */

static void setup_probe(const char *soc)
{
	mock_reset();
	use_soc(soc);
	mock_cpm[0x10 / 4] = 0x0740510D;
	mock_cpm[0x14 / 4] = 0x0640510D;
	mock_cpm[0xe0 / 4] = 0x0640510D;
	if (!strcmp(soc, "t41")) {
		mock_cpm[0x10 / 4] = 0x05B008BD;
		mock_cpm[0x14 / 4] = 0x15D0889D;
		mock_cpm[0xe0 / 4] = 0x059008BD;
	}
}

static void test_probe_id_twins_and_grouping(void)
{
	struct mock_chip *c;

	setup_probe("t31");
	c = mock_chip_add(0, 0x37);
	mock_chip_reg(c, 0xf0, 0x20);
	mock_chip_reg(c, 0xf1, 0x53);

	CHECK_EQ(probe_all(), 0);
	CHECK_EQ(num_matches, 3); /* gc2063, gc2063s1, gc2053 */
	CHECK(!strcmp(primary_name(), "gc2063"));
	silent_report();
	CHECK_EQ(num_devices, 1);
}

static void test_probe_int8_regression(void)
{
	struct mock_chip *c;

	/* sc2336 sits past table index 127: the module's int8_t bug */
	setup_probe("t31");
	c = mock_chip_add(0, 0x30);
	mock_chip_reg(c, 0x3107, 0xcb);
	mock_chip_reg(c, 0x3108, 0x3a);

	CHECK_EQ(probe_all(), 0);
	CHECK(primary_idx > 127);
	CHECK(!strcmp(primary_name(), "sc2336"));
}

static void test_probe_t41_table_filter(void)
{
	struct mock_chip *c;

	setup_probe("t41");
	c = mock_chip_add(0, 0x37);
	mock_chip_reg(c, 0xf0, 0x20);
	mock_chip_reg(c, 0xf1, 0x53);

	CHECK_EQ(probe_all(), 0);
	/* gc2053 is not-t41; both gc2063 entries and gc2063s1 match */
	CHECK(!strcmp(primary_name(), "gc2063"));
	for (int i = 0; i < num_matches; i++)
		CHECK(strcmp(sensor_db[match_idx[i]].name, "gc2053") != 0);
}

static void test_probe_ov2735b_quirk(void)
{
	struct mock_chip *c;

	/* reg 0x04 = 0x06: ov2735b matches via the alternate-ID leg */
	setup_probe("t21");
	mock_cpm[0x10 / 4] = 0x0470484D;
	mock_cpm[0x14 / 4] = 0x04A0484D;
	mock_cpm[0xe0 / 4] = 0x0310086D;
	c = mock_chip_add(0, 0x3c);
	mock_chip_reg(c, 0x02, 0x27);
	mock_chip_reg(c, 0x03, 0x35);
	mock_chip_reg(c, 0x04, 0x06);
	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "ov2735b"));

	/* reg 0x04 = 0x07: the other accepted value */
	mock_chips[0].regs[2].val = 0x07;
	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "ov2735b"));

	/* reg 0x04 = 0x05: plain ov2735, not the b variant */
	mock_chips[0].regs[2].val = 0x05;
	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "ov2735"));
	for (int i = 0; i < num_matches; i++)
		CHECK(strcmp(sensor_db[match_idx[i]].name, "ov2735b") != 0);
}

/* stateful sc2336p/sc2337p model: unlock writes flip the ID page */
static int scp_unlocked;
static int scp_2337; /* 0x801e low nibble nonzero = sc2337p per the table */

static int scp_hook(int bus, struct i2c_msg *msgs, int n)
{
	uint32_t reg = 0;
	int i;

	(void)bus;
	if (msgs[0].addr != 0x30)
		return -1; /* nothing else on this bus */

	if (n == 1 && !(msgs[0].flags & I2C_M_RD)) {
		if (msgs[0].len == 3 && msgs[0].buf[0] == 0x30 && msgs[0].buf[1] == 0x1a)
			scp_unlocked = 1;
		return 0;
	}
	if (n == 2 && (msgs[1].flags & I2C_M_RD)) {
		for (i = 0; i < msgs[0].len; i++)
			reg = (reg << 8) | msgs[0].buf[i];
		uint32_t val = 0;

		if (reg == 0x3107)
			val = scp_unlocked ? 0x9b : 0xcb;
		else if (reg == 0x3108)
			val = 0x3a;
		else if (reg == 0x801e)
			val = scp_2337 ? 0x01 : 0x00;
		for (i = 0; i < msgs[1].len; i++)
			msgs[1].buf[i] = (val >> (8 * (msgs[1].len - 1 - i))) & 0xff;
		return 0;
	}
	return 0;
}

static void test_probe_sc2336p_disambiguation(void)
{
	setup_probe("t23");
	mock_cpm[0x10 / 4] = 0x0740510D;
	mock_cpm[0x14 / 4] = 0x0640510D;
	mock_i2c_hook = scp_hook;

	scp_unlocked = 0;
	scp_2337 = 0;
	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "sc2336")); /* plain ID leads the table */
	{
		int seen_p = 0, seen_37 = 0;

		for (int i = 0; i < num_matches; i++) {
			if (!strcmp(sensor_db[match_idx[i]].name, "sc2336p"))
				seen_p = 1;
			if (!strcmp(sensor_db[match_idx[i]].name, "sc2337p"))
				seen_37 = 1;
		}
		CHECK(seen_p);
		CHECK(!seen_37);
	}

	scp_unlocked = 0;
	scp_2337 = 1;
	CHECK_EQ(probe_all(), 0);
	{
		int seen_p = 0, seen_37 = 0;

		for (int i = 0; i < num_matches; i++) {
			if (!strcmp(sensor_db[match_idx[i]].name, "sc2336p"))
				seen_p = 1;
			if (!strcmp(sensor_db[match_idx[i]].name, "sc2337p"))
				seen_37 = 1;
		}
		CHECK(!seen_p);
		CHECK(seen_37);
	}
	mock_i2c_hook = NULL;
}

static void test_probe_multibus(void)
{
	struct mock_chip *c;

	setup_probe("t31");
	bus_all = 1;
	mock_nbuses = 2;
	mock_buses[0] = 0;
	mock_buses[1] = 2;

	c = mock_chip_add(0, 0x37); /* gc2053 family on bus 0 */
	mock_chip_reg(c, 0xf0, 0x20);
	mock_chip_reg(c, 0xf1, 0x53);
	c = mock_chip_add(2, 0x31); /* gc5603 family on bus 2 */
	mock_chip_reg(c, 0x03f0, 0x56);
	mock_chip_reg(c, 0x03f1, 0x03);

	CHECK_EQ(probe_all(), 0);
	CHECK(report_contains("2 sensors found"));
	CHECK(report_contains("on bus 2 at 0x31"));
	silent_report();
	CHECK_EQ(num_devices, 2);
	{
		int bus0 = 0, bus2 = 0;

		for (int i = 0; i < num_matches; i++) {
			if (match_res[i].bus == 0)
				bus0++;
			if (match_res[i].bus == 2)
				bus2++;
		}
		CHECK(bus0 > 0);
		CHECK(bus2 > 0);
	}
}

/* ------------------------------------------------------------- GPIO / misc */

/* capture stderr around probe_all(); count occurrences of needle */
static int probe_stderr_count(const char *needle)
{
	char buf[16384] = {0};
	const char *p;
	int saved = dup(2);
	FILE *tmp = tmpfile();
	int n = 0;

	fflush(stderr);
	dup2(fileno(tmp), 2);
	probe_all();
	fflush(stderr);
	dup2(saved, 2);
	close(saved);
	rewind(tmp);
	fread(buf, 1, sizeof(buf) - 1, tmp);
	fclose(tmp);
	for (p = buf; (p = strstr(p, needle)); p += strlen(needle))
		n++;
	return n;
}

static void test_dead_probe_cache(void)
{
	unsigned i, j;
	int expect = 0;

	/* empty bus: one probe attempt per unique (addr, rate, class) combo;
	 * unlock-write classes cost two transfers before the NACK stops them */
	setup_probe("t31");
	CHECK_EQ(probe_all(), 0);

	for (i = 0; i < SENSOR_COUNT; i++) {
		const struct sensor_def *a = &sensor_db[i];
		int cls, first = 1;

		cur_soc = soc_by_name("t31");
		if (!sensor_matches_soc(a))
			continue;
		cls = quirk_class(a);
		for (j = 0; j < i; j++) {
			const struct sensor_def *b = &sensor_db[j];

			if (sensor_matches_soc(b) && b->i2c_addr == a->i2c_addr &&
			    b->clk == a->clk && quirk_class(b) == cls) {
				first = 0;
				break;
			}
		}
		if (first)
			expect += (cls == QUIRK_SC233XP || cls == QUIRK_SC3336P) ? 2 : 1;
	}
	CHECK_EQ(mock_i2c_xfers, expect);
	CHECK(expect < 80); /* the collapse is real: was ~600 transfers */

	/* a present chip is never veto'd by the cache */
	setup_probe("t31");
	struct mock_chip *c = mock_chip_add(0, 0x29);

	mock_chip_reg(c, 0x03f0, 0x46);
	mock_chip_reg(c, 0x03f1, 0x53);
	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "gc4653"));

	/* MCLK ends gated even though it stays up between entries now */
	CHECK_EQ((mock_cpm[0x7c / 4] >> 27) & 1, 1);
}

static void test_probe_progress(void)
{
	struct mock_chip *c;

	/* non-tty stderr: exactly one progress line per bus */
	setup_probe("t31");
	c = mock_chip_add(0, 0x29);
	mock_chip_reg(c, 0x03f0, 0x46);
	mock_chip_reg(c, 0x03f1, 0x53);
	verbose = 0;
	CHECK_EQ(probe_stderr_count("probing"), 1);

	setup_probe("t31");
	bus_all = 1;
	mock_nbuses = 2;
	mock_buses[0] = 0;
	mock_buses[1] = 1;
	CHECK_EQ(probe_stderr_count("probing"), 2);

	/* verbose mode has its own per-entry stream, no progress lines */
	setup_probe("t31");
	verbose = 1;
	CHECK_EQ(probe_stderr_count("sinfo: probing 290 sensor"), 0);
	verbose = 0;
}

static void test_report_scan_dump_verbose_only(void)
{
	struct mock_chip *c;

	/* matched chip: raw scan dump only with -v */
	setup_probe("t31");
	c = mock_chip_add(0, 0x29);
	mock_chip_reg(c, 0x03f0, 0x46);
	mock_chip_reg(c, 0x03f1, 0x53);
	CHECK_EQ(probe_all(), 0);
	verbose = 0;
	CHECK(!report_contains("I2C devices seen"));
	CHECK(!report_contains("run with -v"));
	CHECK(report_contains("gc4653 on bus 0 at 0x29 (MCLK 24 MHz)"));
	verbose = 1;
	CHECK(report_contains("I2C devices seen (1):"));
	CHECK(report_contains("gc4653 [matched]"));

	/* responder that matches nothing: default report points at -v,
	 * names the scanned bus, and verbose shows it collapsed once */
	setup_probe("t31");
	c = mock_chip_add(0, 0x29);
	mock_chip_reg(c, 0x03f0, 0x99);
	CHECK_EQ(probe_all(), 0);
	verbose = 0;
	CHECK(report_contains("no sensors detected on bus 0"));
	CHECK(report_contains("rerun with -v"));
	verbose = 1;
	CHECK(report_contains("bus 0 0x29: unmatched"));
	CHECK(report_contains("0x03F0=0x99"));
	verbose = 0;
}

static void test_report_quiet_and_multibus_scope(void)
{
	struct mock_chip *c;

	/* -q prints the name and nothing else */
	setup_probe("t31");
	c = mock_chip_add(0, 0x29);
	mock_chip_reg(c, 0x03f0, 0x46);
	mock_chip_reg(c, 0x03f1, 0x53);
	CHECK_EQ(probe_all(), 0);
	quiet = 1;
	CHECK(report_contains("gc4653"));
	CHECK(!report_contains("bus"));
	CHECK(!report_contains("MCLK"));
	quiet = 0;

	/* -b all no-match names every scanned bus */
	setup_probe("t31");
	bus_all = 1;
	mock_nbuses = 2;
	mock_buses[0] = 0;
	mock_buses[1] = 3;
	CHECK_EQ(probe_all(), 0);
	CHECK(report_contains("no sensors detected on any bus (0, 3)"));
	bus_all = 0;
}

static void test_gpio_claim_failure_warns_once(void)
{
	struct mock_chip *c;

	setup_probe("t31");
	reset_pin = 18;
	mock_export_fail = 1; /* a loaded sensor driver holds the pin */
	c = mock_chip_add(0, 0x29);
	mock_chip_reg(c, 0x03f0, 0x46);
	mock_chip_reg(c, 0x03f1, 0x53);

	CHECK_EQ(probe_all(), 0);
	CHECK(!strcmp(primary_name(), "gc4653"));
	CHECK_EQ(reset_warned, 1); /* once, not once per table entry */
	CHECK_EQ(pwdn_warned, 0);  /* pwdn disabled, nothing to warn about */
}

static void test_gpio_export_ownership(void)
{
	int owned = -1;

	mock_reset();
	use_soc("t31");

	/* pre-exported by someone else: piggyback, never unexport */
	mock_gpio_export(18);
	CHECK_EQ(gpio_claim(18, &owned), 0);
	CHECK_EQ(owned, 0);
	gpio_release(18, owned);
	CHECK(mock_gpio_exported(18));
	CHECK(!mock_sysfs_wrote("unexport=18"));

	/* not exported: ours to create and remove */
	mock_reset();
	owned = -1;
	CHECK_EQ(gpio_claim(19, &owned), 0);
	CHECK_EQ(owned, 1);
	CHECK(mock_gpio_exported(19));
	gpio_release(19, owned);
	CHECK(!mock_gpio_exported(19));
}

static void test_same_device_bus_aware(void)
{
	struct i2c_scan_result a, b;

	memset(&a, 0, sizeof(a));
	a.bus = 0;
	a.i2c_addr = 0x37;
	a.num_regs = 1;
	a.reg_addrs[0] = 0xf0;
	a.reg_values[0] = 0x20;
	b = a;
	CHECK(same_device(&a, &b));
	b.bus = 1; /* same chip model on another bus = another device */
	CHECK(!same_device(&a, &b));
	b = a;
	b.reg_values[0] = 0x21;
	CHECK(!same_device(&a, &b));
}

static void set_chipid(uint32_t cpuid, uint32_t type2)
{
	mock_efuse[0x2c / 4] = cpuid << 12;
	mock_sub[0x250 / 4] = type2 << 16;
}

static void test_soc_chipid_detect(void)
{
	static const struct {
		uint32_t cpuid;
		const char *family;
	} map[] = {
		{0x0005, "t10"}, {0x2000, "t20"}, {0x0021, "t21"},
		{0x0023, "t23"}, {0x0030, "t30"}, {0x0031, "t31"},
	};
	unsigned i;

	for (i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
		mock_reset();
		set_chipid(map[i].cpuid, 0);
		CHECK(soc_from_chipid() == soc_by_name(map[i].family));
	}

	/* unambiguous T41 SKUs */
	mock_reset();
	set_chipid(0x0040, 0xaaaa); /* t41nq */
	CHECK(soc_from_chipid() == soc_by_name("t41"));
	set_chipid(0x0040, 0x9999); /* t41lq */
	CHECK(soc_from_chipid() == soc_by_name("t41"));

	/* shared T40/T41 SKU code falls back to t40 (no cpuinfo here) */
	set_chipid(0x0040, 0x7777);
	CHECK(soc_from_chipid() == soc_by_name("t40"));

	/* unsupported and unknown families defer to the rest of the chain */
	set_chipid(0x0001, 0);
	CHECK(soc_from_chipid() == NULL);
	set_chipid(0x0033, 0);
	CHECK(soc_from_chipid() == NULL);
	set_chipid(0x0032, 0);
	CHECK(soc_from_chipid() == NULL);
	set_chipid(0xbeef, 0);
	CHECK(soc_from_chipid() == NULL);
}

static void test_dump(void)
{
	struct mock_chip *c;
	char buf[16384] = {0};
	int saved, rc;
	FILE *tmp;

	setup_probe("t31");
	c = mock_chip_add(0, 0x1a);
	mock_chip_reg(c, 0x3008, 0xa0);
	mock_chip_reg(c, 0x301e, 0xb2);

	saved = dup(1);
	tmp = tmpfile();
	fflush(stdout);
	dup2(fileno(tmp), 1);
	rc = do_dump(0x1a, 0x3000, 0x301f, 2, 1, 24000000);
	fflush(stdout);
	dup2(saved, 1);
	close(saved);
	rewind(tmp);
	fread(buf, 1, sizeof(buf) - 1, tmp);
	fclose(tmp);

	CHECK_EQ(rc, 0);
	CHECK(strstr(buf, "0x3008=0xA0") != NULL);
	CHECK(strstr(buf, "0x301E=0xB2") != NULL);
	CHECK(strstr(buf, "0x3000=0x00") != NULL); /* unmapped regs read zero */
	/* MCLK gated again afterwards */
	CHECK_EQ((mock_cpm[0x7c / 4] >> 27) & 1, 1);

	/* nothing at the address: no output, failure */
	setup_probe("t31");
	CHECK_EQ(do_dump(0x1a, 0x3000, 0x3001, 2, 1, 24000000), -1);
}

static void test_soc_lookup(void)
{
	CHECK(soc_by_name("t31") != NULL);
	CHECK(soc_by_name("T41") != NULL);
	CHECK(soc_by_name("bogus") == NULL);
	CHECK_EQ(soc_by_name("t40")->num_mclk, 3);
	CHECK_EQ(soc_by_name("t40")->mclk[2].cdr_off, 0x98);
}

static void test_table_invariants(void)
{
	for (unsigned i = 0; i < SENSOR_COUNT; i++) {
		const struct sensor_def *s = &sensor_db[i];

		CHECK(s->name && s->name[0]);
		CHECK(s->i2c_addr <= 0x7f);
		CHECK(s->clk > 0);
		CHECK(s->id_cnt >= 1 && s->id_cnt <= 4);
		CHECK(s->id_addr_len >= 1 && s->id_addr_len <= 4);
		CHECK(s->id_value_len >= 1 && s->id_value_len <= 4);
		CHECK(s->soc <= S_NOT_T41);
		for (unsigned j = i + 1; j < SENSOR_COUNT; j++) {
			const struct sensor_def *t = &sensor_db[j];

			if (strcmp(s->name, t->name) || s->i2c_addr != t->i2c_addr ||
			    s->soc != t->soc || s->id_cnt != t->id_cnt)
				continue;
			if (memcmp(s->id_addr, t->id_addr, sizeof(s->id_addr)) ||
			    memcmp(s->id_value, t->id_value, sizeof(s->id_value)))
				continue;
			fprintf(stderr, "duplicate table entry: %s@0x%02x (rows %u, %u)\n", s->name,
				s->i2c_addr, i, j);
			CHECK(0);
		}
	}
}

int main(void)
{
	test_pll_new_style();
	test_pll_t41_style();
	test_pll_old_style();
	test_mclk_divider_and_handshake();
	test_mclk_parent_fallback();
	test_probe_id_twins_and_grouping();
	test_probe_int8_regression();
	test_probe_t41_table_filter();
	test_probe_ov2735b_quirk();
	test_probe_sc2336p_disambiguation();
	test_probe_multibus();
	test_dead_probe_cache();
	test_probe_progress();
	test_report_scan_dump_verbose_only();
	test_report_quiet_and_multibus_scope();
	test_gpio_claim_failure_warns_once();
	test_gpio_export_ownership();
	test_same_device_bus_aware();
	test_dump();
	test_soc_chipid_detect();
	test_soc_lookup();
	test_table_invariants();

	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
