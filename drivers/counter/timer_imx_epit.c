/*
 * Copyright (c) 2018 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <counter.h>
#include <device.h>
#include "epit.h"

#define TIMER_MAX_RELOAD	0xFFFFFFFF
#define EPIT(config)		((EPIT_Type *)config->base)

struct imx_epit_config {
	EPIT_Type *base;
	void (*irq_config_func)(struct device *dev);
};

struct imx_epit_data {
	volatile counter_callback_t callback;
	volatile void *user_data;
};

static void imx_epit_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);
	struct imx_epit_data *driver_data = dev->driver_data;

	EPIT_ClearStatusFlag(base);

	if (driver_data->callback != NULL) {
		driver_data->callback(dev, (void *)driver_data->user_data);
	}
}

static int imx_epit_init(struct device *dev)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);
	epit_init_config_t epit_config = {
		.freeRun     = true,
		.waitEnable  = true,
		.stopEnable  = true,
		.dbgEnable   = true,
		.enableMode  = true
	};

	EPIT_Init(base, &epit_config);
	config->irq_config_func(dev);

	return 0;
}

static int imx_epit_start(struct device *dev)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);

	/* Set EPIT clock source */
	EPIT_SetClockSource(base, epitClockSourcePeriph);

	/* Set prescaler divides by 1 */
	EPIT_SetPrescaler(base, 0);

	/* Set the counter reload value to a maximum */
	EPIT_SetCounterLoadValue(base, TIMER_MAX_RELOAD);

	/* Generate interrupt when the counter reaches zero */
	EPIT_SetOutputCompareValue(base, 0U);

	/* Start the counter */
	EPIT_Enable(base);

	return 0;
}

static int imx_epit_stop(struct device *dev)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);
	struct imx_epit_data *driver_data = dev->driver_data;

	/* Disable alarm */
	EPIT_SetIntCmd(base, false);
	driver_data->callback = NULL;
	driver_data->user_data = NULL;

	/* Disable EPIT */
	EPIT_Disable(base);

	return 0;
}

static u32_t imx_epit_read(struct device *dev)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);
	u32_t value;

	value = EPIT_GetCounterLoadValue(base) - EPIT_ReadCounter(base);

	return value;
}

static int imx_epit_set_alarm(struct device *dev,
			      counter_callback_t callback,
			      u32_t count, void *user_data)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);
	struct imx_epit_data *driver_data = dev->driver_data;

	if ((EPIT_CR_REG(base) & EPIT_CR_EN_MASK) == 0U) {
		/* EPIT is not started */
		return -ENOTSUP;
	}

	/* Disable EPIT Output Compare interrupt for consistency */
	EPIT_SetIntCmd(base, false);

	driver_data->callback = callback;
	driver_data->user_data = user_data;

	if (callback != NULL) {
		/* Set both reload and counter values to "count" */
		EPIT_SetOverwriteCounter(base, true);
		EPIT_SetCounterLoadValue(base, count);
		/* (Re)enable EPIT Output Compare interrupt */
		EPIT_SetIntCmd(base, true);
	} else {
		/* Reset reload value without affecting the actual count */
		EPIT_SetOverwriteCounter(base, false);
		EPIT_SetCounterLoadValue(base, TIMER_MAX_RELOAD);
	}

	return 0;
}

static u32_t imx_epit_get_pending_int(struct device *dev)
{
	const struct imx_epit_config *config = dev->config->config_info;
	EPIT_Type *base = EPIT(config);

	return EPIT_GetStatusFlag(base) ? 1U : 0U;
}

static const struct counter_driver_api imx_epit_driver_api = {
	.start = imx_epit_start,
	.stop = imx_epit_stop,
	.read = imx_epit_read,
	.set_alarm = imx_epit_set_alarm,
	.get_pending_int = imx_epit_get_pending_int
};

/* TIMER 1 */
#ifdef CONFIG_TIMER_IMX_EPIT_1
static void imx_epit_config_func_1(struct device *dev);

static const struct imx_epit_config imx_epit_1_config = {
	.base = (EPIT_Type *)CONFIG_TIMER_IMX_EPIT_1_BASE_ADDRESS,
	.irq_config_func = imx_epit_config_func_1,
};

static struct imx_epit_data imx_epit_1_data;

DEVICE_AND_API_INIT(epit_1, CONFIG_TIMER_IMX_EPIT_1_LABEL,
		    &imx_epit_init,
		    &imx_epit_1_data, &imx_epit_1_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &imx_epit_driver_api);

static void imx_epit_config_func_1(struct device *dev)
{
	IRQ_CONNECT(CONFIG_TIMER_IMX_EPIT_1_IRQ,
		    CONFIG_TIMER_IMX_EPIT_1_IRQ_PRI,
		    imx_epit_isr, DEVICE_GET(epit_1), 0);

	irq_enable(CONFIG_TIMER_IMX_EPIT_1_IRQ);
}
#endif /* CONFIG_TIMER_IMX_EPIT_1 */

/* TIMER 2 */
#ifdef CONFIG_TIMER_IMX_EPIT_2
static void imx_epit_config_func_2(struct device *dev);

static const struct imx_epit_config imx_epit_2_config = {
	.base = (EPIT_Type *)CONFIG_TIMER_IMX_EPIT_2_BASE_ADDRESS,
	.irq_config_func = imx_epit_config_func_2,
};

static struct imx_epit_data imx_epit_2_data;

DEVICE_AND_API_INIT(epit_2, CONFIG_TIMER_IMX_EPIT_2_LABEL,
		    &imx_epit_init,
		    &imx_epit_2_data, &imx_epit_2_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &imx_epit_driver_api);

static void imx_epit_config_func_2(struct device *dev)
{
	IRQ_CONNECT(CONFIG_TIMER_IMX_EPIT_2_IRQ,
		    CONFIG_TIMER_IMX_EPIT_2_IRQ_PRI,
		    imx_epit_isr, DEVICE_GET(epit_2), 0);

	irq_enable(CONFIG_TIMER_IMX_EPIT_2_IRQ);
}
#endif /* CONFIG_TIMER_IMX_EPIT_2 */
