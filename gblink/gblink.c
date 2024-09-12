// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#include <furi.h>
#include <furi_hal.h>
#include <stm32wbxx_ll_exti.h>
#include <stm32wbxx_ll_system.h>

#include <stdint.h>

#include <gblink/include/gblink.h>
#include "exti_workaround_i.h"
#include "clock_timer_i.h"

const struct gblink_pins common_pinouts[PINOUT_COUNT] = {
	/* Original */
	{
		&gpio_ext_pc3,
		&gpio_ext_pb3,
		&gpio_ext_pb2,
		&gpio_ext_pa4,
	},
	/* MALVEKE EXT1 */
	{
		&gpio_ext_pa6,
		&gpio_ext_pa7,
		&gpio_ext_pb3,
		&gpio_ext_pa4,
	},
};

struct gblink {
	const GpioPin *serin;
	const GpioPin *serout;
	const GpioPin *clk;
	const GpioPin *sd;
	gblink_mode mode;
	void (*callback)(void* cb_context, uint8_t in);
	void *cb_context;

	/* These two semaphores serve similar but distinct purposes. */
	/* The transfer semaphore is taken as soon as a transfer() request
	 * has been started. This is used in the function to wait until the
	 * transfer has been completed.
	 */
	FuriSemaphore *transfer_sem;
	/* The out byte semaphore is used to indicate that a byte transfer
	 * is in progress. This is used in the transfer function to not allow
	 * a transfer request if we're in the middle of sending a byte.
	 * The transfer semaphore is not used for that purpose since if the
	 * Flipper is in EXT clk mode, once a transfer() is started, there
	 * would be no way to both prevent transfer() from being called again
	 * as well as cancelling/changing what we're wanting to send. Using
	 * out byte semaphore means a transfer() can be called at any time,
	 * waited on synchronously for a timeout, and then re-called at a
	 * later time; while blocking that update if a byte is actually
	 * in the middle of being transmitted.
	 */
	FuriSemaphore *out_byte_sem;

	/* Used to lock out changing things after a certain point. Pinout,
	 * mode, etc.
	 * XXX: Might make more sense to use the mutex to protect a flag?
	 * Maybe a semaphore? Though I think that is the wrong use.
	 */
	FuriMutex *start_mutex;

	/* 
	 * The following should probably have the world stopped around them
	 * if not modified in an interrupt context.
	 */
	uint8_t in;
	uint8_t out;
	uint8_t shift;
	uint8_t nobyte;

	/* Should only be changed when not in middle of tx, will affect a lot */
	gblink_clk_source source;

	/* Can be changed at any time, will only take effect on the next
	 * transfer.
	 */
	gblink_speed speed;


	/* 
	 * The following is based on observing Pokemon trade data
	 *
	 * Clocks idle between bytes is nominally 430 us long for burst data,
	 * 15 ms for idle polling (e.g. waiting for menu selection), some oddball
	 * 2 ms gaps that appears between one 0xFE byte from the Game Boy every trade;
	 * clock period is nominally 122 us.
	 *
	 * Therefore, if we haven't seen a clock in 500 us, reset our bit counter.
	 * Note that, this should never actually be a concern, but it is an additional
	 * safeguard against desyncing.
	 */
	uint32_t time;
	uint32_t bitclk_timeout_us;

	void *exti_workaround_handle;
};

static inline bool gblink_transfer_in_progress(struct gblink *gblink)
{
	return !(furi_semaphore_get_count(gblink->out_byte_sem));
}

static void gblink_shift_in_isr(struct gblink *gblink)
{
	const uint32_t time_ticks = furi_hal_cortex_instructions_per_microsecond() * gblink->bitclk_timeout_us;

	if (gblink->source == GBLINK_CLK_INT)
		furi_hal_gpio_write(gblink->clk, 1);

	/* If we exceeded the bit clock timeout, reset all counters */
	if ((DWT->CYCCNT - gblink->time) > time_ticks) {
		gblink->in = 0;
		gblink->shift = 0;
	}
	gblink->time = DWT->CYCCNT;

	gblink->in <<= 1;
	gblink->in |= furi_hal_gpio_read(gblink->serin);
	gblink->shift++;
	/* If 8 bits transfered, reset shift counter, call registered
	 * callback, re-set nobyte in output buffer.
	 */
	if (gblink->shift == 8) {
	 	if (gblink->source == GBLINK_CLK_INT)
			clock_timer_stop();

		gblink->shift = 0;

		/* 
		 * Set up next out byte before calling the callback.
		 * This is in case the callback itself sets a new out
		 * byte which it will in most cases.
		 *
		 * The nobyte value is set in place as the next output byte,
		 * in case the flipper does not set a real byte before the next
		 * transfer starts.
		 */
		gblink->out = gblink->nobyte;
		furi_semaphore_release(gblink->out_byte_sem);

		/* 
		 * Call the callback, if set, and then release the semaphore
		 * in case a thread is waiting on TX to complete.
		 */
		if (gblink->callback)
			gblink->callback(gblink->cb_context, gblink->in);

		furi_semaphore_release(gblink->transfer_sem);
	}
}

static void gblink_shift_out_isr(struct gblink *gblink)
{
	furi_semaphore_acquire(gblink->out_byte_sem, 0);
	furi_hal_gpio_write(gblink->serout, !!(gblink->out & 0x80));
	gblink->out <<= 1;

	/* XXX: TODO: Check that this is the correct thing with open drain.
	 * does 0 value actually drive the line low, or high?
	 */
	if (gblink->source == GBLINK_CLK_INT)
		furi_hal_gpio_write(gblink->clk, 0);
}

static void gblink_clk_isr(void *context)
{
	furi_assert(context);
	struct gblink *gblink = context;
	bool out = false;

	/* 
	 * Whether we're shifting in or out is dependent on the clock source.
	 * If external, and the clock line is high, that means a posedge just
	 * occurred and we need to shift data in.
	 *
	 * If internal, and the clock line is high, that means we're about
	 * to drive a negedge and need to shift data out.
	 *
	 * The actual in/out functions drive the clock state at the right times
	 * if the clock is internal source.
	 */
	out = (furi_hal_gpio_read(gblink->clk) ==
	      (gblink->source == GBLINK_CLK_INT));

	if (out)
		gblink_shift_out_isr(gblink);
	else
		gblink_shift_in_isr(gblink);
}

/* 
 * Call to set up the clk pin modes to do the right thing based on if INT or
 * EXT clock source is configured.
 */
static void gblink_clk_configure(struct gblink *gblink)
{
	if (gblink->source == GBLINK_CLK_EXT) {
		furi_hal_gpio_init(gblink->clk, GpioModeInterruptRiseFall, GpioPullUp, GpioSpeedVeryHigh);
		/* furi_hal_gpio_init, while it sets interrupt settings on the GPIO,
		 * does not actually enable the EXTI interrupt.
		 */
		gblink_int_enable(gblink);
	} else {
		/* This will disable the EXTI interrupt for us */
		furi_hal_gpio_init(gblink->clk, GpioModeOutputOpenDrain, GpioPullUp, GpioSpeedVeryHigh);
	};
}

void gblink_clk_source_set(void *handle, gblink_clk_source source)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (source == gblink->source)
		return;
	
	/*
	 * NOTE:
	 * I'm not sure the best way to handle this at the moment. In theory,
	 * it should be safe to check that we're just not in the middle of a
	 * transfer and not worry about getting stuck.
	 * However, I'm not really sure how true that is, so for now this will
	 * always change the source and reset the current byte transfer.
	 * It is up to the callee to ensure that they are between bytes.
	 *
	 * One idea would be to get the semaphore, but wait the set timeout.
	 * if that is exceeded or the semaphore is acquired, then its probably
	 * safe to change the source and reset shift register.
	 */

	gblink->source = source;
	gblink->shift = 0;

	gblink_clk_configure(gblink);
}

void gblink_speed_set(void *handle, gblink_speed speed)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/*
	 * This does not need any protection, it will take effect at the start
	 * of the next byte.
	 */
	gblink->speed = speed;
}

/* default is set to 500 us */
void gblink_timeout_set(void *handle, uint32_t us)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	gblink->bitclk_timeout_us = us;
}

int gblink_pin_set(void *handle, gblink_bus_pins pin, const GpioPin *gpio)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	switch (pin) {
	case PIN_SERIN:
		gblink->serin = gpio;
		break;
	case PIN_SEROUT:
		gblink->serout = gpio;
		break;
	case PIN_CLK:
		gblink->clk = gpio;
		break;
	case PIN_SD:
		gblink->sd = gpio;
		break;
	default:
		furi_crash();
		break;
	}

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

int gblink_pin_set_default(void *handle, gblink_pinouts pinout)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	gblink->serin = common_pinouts[pinout].serin;
	gblink->serout = common_pinouts[pinout].serout;
	gblink->clk = common_pinouts[pinout].clk;
	gblink->sd = common_pinouts[pinout].sd;

	furi_mutex_release(gblink->start_mutex);

	return 0;
}


const GpioPin *gblink_pin_get(void *handle, gblink_bus_pins pin)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	switch (pin) {
	case PIN_SERIN:
		return gblink->serin;
	case PIN_SEROUT:
		return gblink->serout;
	case PIN_CLK:
		return gblink->clk;
	case PIN_SD:
		return gblink->sd;
	default:
		furi_crash();
		break;
	}

	return NULL;
}

int gblink_callback_set(void *handle, void (*callback)(void* cb_context, uint8_t in), void *cb_context)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	gblink->callback = callback;
	gblink->cb_context = cb_context;
	furi_mutex_release(gblink->start_mutex);

	return 0;
}

int gblink_mode_set(void *handle, gblink_mode mode)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	gblink->mode = mode;
	furi_mutex_release(gblink->start_mutex);

	return 0;
}


bool gblink_transfer(void *handle, uint8_t val)
{
	furi_assert(handle);
	struct gblink *gblink = handle;
	bool ret = false;


	/* Stop the world, this is to ensure we can safely set the next out byte */
	/*
	 * The reason for and therefore issue of setting the next byte has a few
	 * points to keep in mind.
	 *
	 * First, with EXT clock source, the first hint of the external device
	 * clocking in data is a negative edge where it would set data. This
	 * means that the next out byte needs to be set before that.
	 *
	 * Second, since the interrupt on the neg clock edge loads the next
	 * byte in to serout after grabbing the semaphore; we can stop the
	 * world right now, and set the byte if there is no transfer in
	 * progress. As soon as the world is resumed, the IRQ will fire, and
	 * the correct, new, data byte will start to be shifted out.
	 */
	FURI_CRITICAL_ENTER();

	/* If we're in the middle of a tranfer, don't let the byte be set. */
	if (!gblink_transfer_in_progress(gblink)) {
		gblink->out = val;
		ret = true;

		/*
		 * Now that we're this far, this means the byte we set will be
		 * transferred one way or another. Because of that, take the
		 * transfer semaphore. This gets released once a full byte has
		 * been transferred. This is for the TX wait function. We cannot
		 * use the out_byte_sem as if the wait is called immediately
		 * after the transfer, and no data has yet been shifted out,
		 * the TX wait function would incorrectly return immediately.
		 */
		furi_semaphore_acquire(gblink->transfer_sem, 0);
	}

	FURI_CRITICAL_EXIT();

	/* 
	 * If the out byte was successfully set, and we're driving the clock,
	 * turn on our timer for byte transfer.
	 */
	if (ret && gblink->source == GBLINK_CLK_INT)
		clock_timer_start(gblink_clk_isr, gblink, gblink->speed);

	return ret;

}

uint8_t gblink_transfer_tx_wait_complete(void *handle)
{
	struct gblink *gblink = handle;

	/* XXX: TODO: Think about how to implement this in a way that we can
	 * use the semaphore to see if there is a transfer waiting to happen,
	 * but not in a way that would incorrectly show a transfer waiting. e.g.
	 * if this takes the semaphore, then the semaphore is in the same state
	 * as if a transfer was in progress. Should this put back the semaphore
	 * after acquiring it? Is there a better way of handling it?
	 */

	furi_semaphore_acquire(gblink->transfer_sem, FuriWaitForever);

	return gblink->in;
}

void gblink_nobyte_set(void *handle, uint8_t val)
{
	struct gblink *gblink = handle;

	/*
	 * This is safe to run at any time. It is only copied in after a byte
	 * transfer is completed.
	 */
	gblink->nobyte = val;
}

void gblink_int_enable(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/*
	 * NOTE: This is currently safe to run even with the exti workaround
	 * in effect. It just enables the root EXTI interrupt source of the
	 * given pin.
	 */
	furi_hal_gpio_enable_int_callback(gblink->clk);
}

void gblink_int_disable(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/*
	 * NOTE: This is currently safe to run even with the exti workaround
	 * in effect. It just disables the root EXTI interrupt source of the
	 * given pin.
	 */
	furi_hal_gpio_disable_int_callback(gblink->clk);
}

void *gblink_alloc(void)
{
	struct gblink *gblink;

	/* Allocate and zero struct */
	gblink = malloc(sizeof(struct gblink));
	//gblink->spec = malloc(sizeof(struct gblink_spec));

	gblink->transfer_sem = furi_semaphore_alloc(1, 1);
	gblink->out_byte_sem = furi_semaphore_alloc(1, 1);
	gblink->start_mutex = furi_mutex_alloc(FuriMutexTypeNormal);

	/* Set defaults */
	gblink_pin_set_default(gblink, PINOUT_ORIGINAL);
	gblink_mode_set(gblink, GBLINK_MODE_GBC);
	gblink_clk_source_set(gblink, GBLINK_CLK_EXT);
	gblink_speed_set(gblink, GBLINK_SPD_8192HZ);
	gblink_timeout_set(gblink, 500);

	/* Set current time to start timeout calculations */
	gblink->time = DWT->CYCCNT;

	return gblink;
}

void gblink_start(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/* XXX: Check callback is valid */

	furi_mutex_acquire(gblink->start_mutex, FuriWaitForever);

	/* Set up pins */
	/* TODO: Set up a list of pins that are not safe to use with interrupts.
	 * I do believe the main FURI GPIO struct has this data baked in so that
	 * could be used. For now though, we're only checking for the MALVEKE
	 * pinout which uses a clk pin that has its IRQ shared with the Okay
	 * button.
	 * See the work done in pokemon trade tool custom pinout selection for
	 * an idea of how to check all that.
	 */
	furi_hal_gpio_write(gblink->serout, false);
	furi_hal_gpio_init(gblink->serout, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
	furi_hal_gpio_write(gblink->serin, false);
	furi_hal_gpio_init(gblink->serin, GpioModeInput, GpioPullUp, GpioSpeedVeryHigh);

	/* Set up interrupt on clock pin */
	if (gblink->clk == &gpio_ext_pb3) {
		/* The clock pin is on a pin that is not safe to set an interrupt
		 * on, so we do a gross workaround to get an interrupt enabled
		 * on that pin in a way that can be undone safely later with
		 * no impact to the shared IRQ.
		 */
		gblink->exti_workaround_handle = exti_workaround(gblink->clk, gblink_clk_isr, gblink);
	} else {
		/* This may not be needed after NFC refactor */
		furi_hal_gpio_remove_int_callback(gblink->clk);
		furi_hal_gpio_add_int_callback(gblink->clk, gblink_clk_isr, gblink);
	}

	/* The above immediately enables the interrupt, we don't want
	 * that just yet and we want configure to handle it.
	 */
	gblink_int_disable(gblink);

	gblink_clk_configure(gblink);
}

void gblink_stop(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/* If we can acquire the mutex, that means start was never actually
	 * called. Crash.
	 * XXX: Probably a bit harsh to just crash, can it gracefully recover
	 * without too much effort?
	 */
	if (furi_mutex_acquire(gblink->start_mutex, 0) == FuriStatusOk) {
		furi_crash();
		return;
	}

	if (gblink->clk == &gpio_ext_pb3) {
		/* This handles switching the IVT back and putting the EXTI
		 * regs and pin regs in a valid state for normal use.
		 */
		exti_workaround_undo(gblink->exti_workaround_handle);
	} else {
		/* Remove interrupt, set IO to sane state */
		furi_hal_gpio_remove_int_callback(gblink->clk);
	}
	furi_hal_gpio_init_simple(gblink->serin, GpioModeAnalog);
	furi_hal_gpio_init_simple(gblink->serout, GpioModeAnalog);
	furi_hal_gpio_init_simple(gblink->clk, GpioModeAnalog);

	furi_mutex_release(gblink->start_mutex);
}

void gblink_free(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/* If we cannot acquire the mutex, that means the link was never properly
	 * stopped. Crash.
	 * XXX: Can this be gracefully handled?
	 */
	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk) {
		furi_crash();
		return;
	}
	furi_mutex_release(gblink->start_mutex);
	furi_mutex_free(gblink->start_mutex);
	furi_semaphore_free(gblink->transfer_sem);
	furi_semaphore_free(gblink->out_byte_sem);
	free(gblink);
}
