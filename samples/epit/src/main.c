/*
 * Copyright (c) 2018 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <counter.h>
#include "clock_freq.h"

static void counter_callback(struct device *dev, void *user_data)
{
	printk("timer tick\n");
}

void main(void)
{
	int status;
	struct device *epit;
	u32_t frequency;

	printk("EPIT sample app is starting...\n");

	epit = device_get_binding(EPIT_1_LABEL);

	if (!epit) {
		printk("EPIT not initialized\n");
		while (1) {
		}
	}

	status = counter_start(epit);
	printk("counter started %d\n", status);

	/* Get EPIT clock frequency */
	frequency = get_epit_clock_freq(EPIT1);
	printk("EPIT clock frequency is %u\n", frequency);

	status = counter_set_alarm(epit, counter_callback, frequency / 2, NULL);
	printk("counter set alarm %d\n", status);

	k_sleep(5001);

	status = counter_stop(epit);
	printk("counter stopped %d\n", status);

	while (1) {
	}
}
