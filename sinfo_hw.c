// SPDX-License-Identifier: GPL-2.0
/*
 * sinfo_hw.c - real hardware access
 *
 * /dev/mem for CPM and GPIO registers, /dev/i2c-N for sensor I2C,
 * sysfs for GPIO control. Mappings and descriptors live for the
 * process lifetime; sinfo is a one-shot tool.
 */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "sinfo_hw.h"

#define CPM_PHYS 0x10000000

static volatile uint32_t *cpm_base;
static int i2c_fd = -1;

volatile uint32_t *hw_map_phys(uint32_t phys, size_t len)
{
	static int memfd = -1;
	void *p;

	if (memfd < 0) {
		memfd = open("/dev/mem", O_RDWR | O_SYNC);
		if (memfd < 0) {
			fprintf(stderr, "sinfo: [Error] cannot open /dev/mem: %s\n",
				strerror(errno));
			return NULL;
		}
	}
	p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, phys);
	if (p == MAP_FAILED) {
		fprintf(stderr, "sinfo: [Error] mmap 0x%08x failed: %s\n", phys, strerror(errno));
		return NULL;
	}
	return p;
}

int hw_cpm_init(void)
{
	if (cpm_base)
		return 0;
	cpm_base = hw_map_phys(CPM_PHYS, 0x1000);
	return cpm_base ? 0 : -1;
}

uint32_t hw_cpm_rd(uint32_t off)
{
	return cpm_base[off / 4];
}

void hw_cpm_wr(uint32_t off, uint32_t val)
{
	cpm_base[off / 4] = val;
}

static int i2c_cur_bus = -1;

int hw_i2c_open(int bus)
{
	char path[32];

	if (i2c_fd >= 0) {
		if (bus == i2c_cur_bus)
			return 0;
		close(i2c_fd);
		i2c_fd = -1;
	}
	snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
	i2c_fd = open(path, O_RDWR);
	if (i2c_fd < 0) {
		fprintf(stderr, "sinfo: [Error] cannot open %s: %s%s\n", path, strerror(errno),
			errno == ENOENT ? " (kernel needs CONFIG_I2C_CHARDEV=y)" : "");
		return -1;
	}
	i2c_cur_bus = bus;
	return 0;
}

int hw_i2c_xfer(struct i2c_msg *msgs, int n)
{
	struct i2c_rdwr_ioctl_data d = {.msgs = msgs, .nmsgs = n};

	return ioctl(i2c_fd, I2C_RDWR, &d) < 0 ? -1 : 0;
}

int hw_i2c_buses(int *buses, int max)
{
	DIR *d;
	struct dirent *e;
	int n = 0, i, j, t;

	d = opendir("/dev");
	if (!d)
		return 0;
	while (n < max && (e = readdir(d))) {
		if (sscanf(e->d_name, "i2c-%d", &t) == 1)
			buses[n++] = t;
	}
	closedir(d);
	for (i = 1; i < n; i++)
		for (j = i; j > 0 && buses[j - 1] > buses[j]; j--) {
			t = buses[j];
			buses[j] = buses[j - 1];
			buses[j - 1] = t;
		}
	return n;
}

int hw_sysfs_write(const char *path, const char *val)
{
	int fd, ret;

	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -1;
	ret = write(fd, val, strlen(val));
	close(fd);
	return ret < 0 ? -1 : 0;
}

int hw_path_writable(const char *path)
{
	return access(path, W_OK) == 0;
}

void hw_msleep(unsigned int ms)
{
	struct timespec ts = {.tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L};

	while (nanosleep(&ts, &ts) && errno == EINTR)
		;
}
