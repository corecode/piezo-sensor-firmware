#define AC_CALLBACK_MODE false

#include <system.h>
#include <system_interrupt.h>
#include <ac.h>
#include <adc.h>
#include <port.h>
#include <events.h>
#include <usart.h>
#include <i2c_master.h>
#include <usb.h>
#include <usb_protocol_cdc.h>
#include <udi_cdc.h>
#include <udc.h>
#include <udd.h>


#define USE_ACCEL 0


enum {
        PIN_POWER_LED = PIN_PA07,
        PIN_ACCEL_LED = PIN_PA10,
        PIN_PIEZO_LED = PIN_PA06,
        PIN_UART_TXO = PIN_PA22,
        PIN_UART_RXI = PIN_PA23,
        PIN_SIGNAL = PIN_PA27,
        PIN_ACCEL_INT = PIN_PA16,
        PIN_I2C_SDA = PIN_PA14,
        PIN_I2C_SCL = PIN_PA15,
        ADC_PIEZO = ADC_POSITIVE_INPUT_PIN1,
        AC_PIEZO = AC_CHAN_CHANNEL_0,
};

enum {
        SYSTICK_FREQ = 100,
};

enum {
        LATCH_TIME_MS = 50,
};

enum {
        VDD_MV = 3300,
        PIEZO_THRESHOLD_MV = 400,
};

enum {
        ACCEL_ADDRESS = 0b0011101,
};

enum {
        ACCEL_THRESHOLD = 8500,
};

void system_board_init(void);



static struct ac_module ac;
static struct usart_module usart;
static struct i2c_master_module i2c;
static struct adc_module adc;

static int cdc_enabled, cdc_ready;
static volatile uint32_t time_ms;

static volatile uint32_t piezo_time, accel_time;


static void
delay_us(uint32_t us)
{
        uint32_t start = SysTick->VAL;

        const uint32_t ticks_us = system_gclk_chan_get_hz(0) / 1000000;
        uint32_t duration = ticks_us * us;
        uint32_t load = SysTick->LOAD;
        uint32_t end, limit;

        if (duration <= start)
                end = start - duration;
        else
                end = start + load - duration;

        if (end >= load / 2)
                limit = load;
        else
                limit = end + load / 2;

        uint32_t now;
        do
                now = SysTick->VAL;
        while (now > end && now < limit);
}

void
AC_Handler(void)
{
        ac_chan_get_status(&ac, AC_PIEZO);
        if (piezo_time == 0)
                piezo_time = time_ms;
        ac_chan_clear_status(&ac, AC_PIEZO);
}

bool
piezo_cdc_enable(void)
{
        cdc_enabled = true;
        return true;
}

void
piezo_cdc_disable(void)
{
        cdc_enabled = false;
}

void
piezo_cdc_dtr(int set)
{
        cdc_ready = set;
}

void
SysTick_Handler(void)
{
        time_ms += 1000 / SYSTICK_FREQ;
}

static int
accel_read(uint8_t reg, void *buf, size_t len)
{
	struct i2c_master_packet packet = {
		.address     = ACCEL_ADDRESS,
		.data_length = 1,
		.data        = &reg,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait_no_stop(&i2c, &packet) != STATUS_OK) {
                for(;;);
	}
        packet.data_length = len;
        packet.data = buf;
	while (i2c_master_read_packet_wait(&i2c, &packet) != STATUS_OK) {
                for(;;);
	}
        return (0);
}

static int
accel_write(void *buf, size_t len)
{
	struct i2c_master_packet packet = {
		.address     = ACCEL_ADDRESS,
		.data_length = len,
		.data        = buf,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c, &packet) != STATUS_OK) {
                for(;;);
	}
        return (0);
}

static void
configure_ac(void)
{
        struct ac_config config_ac;
        struct ac_chan_config config_chan;

        ac_get_config_defaults(&config_ac);
	config_ac.dig_source_generator = GCLK_GENERATOR_1;
        ac_init(&ac, AC, &config_ac);
        AC->CTRLA.reg |= AC_CTRLA_LPMUX;

        ac_chan_get_config_defaults(&config_chan);
        config_chan.output_mode = AC_CHAN_OUTPUT_SYNCHRONOUS;

        config_chan.vcc_scale_factor = (VDD_MV/2 + PIEZO_THRESHOLD_MV + VDD_MV / 64 - 1) * 64 / VDD_MV;
        config_chan.interrupt_selection = AC_CHAN_INTERRUPT_SELECTION_RISING;
        ac_chan_set_config(&ac, AC_PIEZO, &config_chan);
        ac_chan_enable(&ac, AC_PIEZO);

        ac.hw->INTENSET.reg = AC_INTFLAG_COMP0;
        system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_AC);

        ac_enable(&ac);
}


static void
configure_uart(void)
{
	struct usart_config config_usart;

	usart_get_config_defaults(&config_usart);

	config_usart.baudrate    = 1000000;
        config_usart.receiver_enable = false;
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_PA22C_SERCOM1_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA23C_SERCOM1_PAD1;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;

	while (usart_init(&usart, SERCOM1, &config_usart) != STATUS_OK) {
	}

        usart_enable(&usart);
}

static void
configure_i2c(void)
{
        /* clear bus */
	struct port_config pin;

        port_get_config_defaults(&pin);
        pin.input_pull = PORT_PIN_PULL_NONE;
        for (int pulse = 0; pulse < 9; pulse++) {
                pin.direction = PORT_PIN_DIR_OUTPUT;
                port_pin_set_config(PIN_I2C_SCL, &pin);
                delay_us(5);
                pin.direction = PORT_PIN_DIR_INPUT;
                port_pin_set_config(PIN_I2C_SCL, &pin);
                delay_us(5);
        }
        /* stop condition */
        pin.direction = PORT_PIN_DIR_OUTPUT;
        port_pin_set_config(PIN_I2C_SDA, &pin);
        delay_us(5);
        pin.direction = PORT_PIN_DIR_INPUT;
        port_pin_set_config(PIN_I2C_SDA, &pin);
        delay_us(5);

	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

        config_i2c_master.pinmux_pad0 = PINMUX_PA14C_SERCOM0_PAD0;
        config_i2c_master.pinmux_pad1 = PINMUX_PA15C_SERCOM0_PAD1;
	i2c_master_init(&i2c, SERCOM0, &config_i2c_master);
	i2c_master_enable(&i2c);
}

static void
configure_adc(void)
{
        struct adc_config config_adc;
        adc_get_config_defaults(&config_adc);
        config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV512;
        config_adc.freerunning = true;
        config_adc.positive_input = ADC_PIEZO;
        config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2;
        config_adc.reference = ADC_REFERENCE_INTVCC1;
        config_adc.resolution = ADC_RESOLUTION_CUSTOM;
        config_adc.divide_result = ADC_DIVIDE_RESULT_2;
        config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_2;
        adc_init(&adc, ADC, &config_adc);
        adc_enable(&adc);
}

static void
configure_systick(void)
{
        SysTick_Config(system_gclk_chan_get_hz(0) / SYSTICK_FREQ);
	NVIC_EnableIRQ(SysTick_IRQn);
}

static void
configure_accel(void)
{
        uint8_t whoami;
        accel_read(0x0f, &whoami, 1);
        delay_us(20);           /* weird race bug prevents NAK */

        uint8_t ctrl[] = {0x20, 0b01101100, 0, 1, 0b00110100};
        accel_write(&ctrl, sizeof(ctrl));
}

void
system_board_init(void)
{
	struct port_config pin;

        port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_POWER_LED, &pin);
	port_pin_set_config(PIN_ACCEL_LED, &pin);
	port_pin_set_config(PIN_PIEZO_LED, &pin);
	port_pin_set_config(PIN_SIGNAL, &pin);
        port_pin_set_output_level(PIN_POWER_LED, 1);
        port_pin_set_output_level(PIN_ACCEL_LED, 0);
        port_pin_set_output_level(PIN_PIEZO_LED, 0);
        port_pin_set_output_level(PIN_PIEZO_LED, 0);
        pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_ACCEL_INT, &pin);
}

static void
print_number(int prefix, int16_t num)
{
        if (!cdc_ready || !cdc_enabled)
                return;

        char out[10];
        char *const end = out + sizeof(out);
        char *pos = end;

        *--pos = '\n';

        for (int16_t val = abs(num); val != 0; val /= 10)
                *--pos = '0' + val % 10;
        if (num == 0)
                *--pos = '0';
        if (num < 0)
                *--pos = '-';
        *--pos = ' ';
        *--pos = prefix;

        size_t len = end - pos;
        if (udi_cdc_get_free_tx_buffer() >= len)
                udi_cdc_write_buf(pos, len);
}

static void
signal_output(int triggered)
{
        port_pin_set_output_level(PIN_SIGNAL, triggered);
}

int
main(void)
{
        system_init();
        configure_systick();
        configure_uart();
        configure_ac();
        configure_adc();
        configure_i2c();
        configure_accel();

        udc_start();
        adc_start_conversion(&adc);

        for(;;) {
                if (piezo_time != 0) {
                        if (time_ms - piezo_time < LATCH_TIME_MS) {
                                port_pin_set_output_level(PIN_PIEZO_LED, 1);
                        } else {
                                port_pin_set_output_level(PIN_PIEZO_LED, 0);
                                piezo_time = 0;
                        }
                }
#if USE_ACCEL
                if (port_pin_get_input_level(PIN_ACCEL_INT)) {
                        int16_t zaccel;
                        accel_read(0x2c, &zaccel, 2);
                        if (zaccel > ACCEL_THRESHOLD && accel_time == 0)
                                accel_time = time_ms;
                        print_number(accel_time ? 'A' : 'a', zaccel);
                }
#endif
                uint16_t piezo;
                enum status_code status;
                while ((status = adc_read(&adc, &piezo)) != STATUS_BUSY) {
                        if (status == STATUS_OK) {
                                int16_t piezo_rel = piezo - 0x800;
                                print_number(piezo_time ? 'P' : 'p', piezo_rel);
                        }
                }
                if (piezo_time != 0) {
                        if (time_ms - piezo_time < LATCH_TIME_MS) {
                                port_pin_set_output_level(PIN_PIEZO_LED, 1);
                        } else {
                                port_pin_set_output_level(PIN_PIEZO_LED, 0);
                                piezo_time = 0;
                        }
                }
                signal_output(piezo_time != 0 || accel_time != 0);
#if USE_ACCEL
                if (accel_time != 0) {
                        if (time_ms - accel_time < LATCH_TIME_MS) {
                                port_pin_set_output_level(PIN_ACCEL_LED, 1);
                        } else {
                                port_pin_set_output_level(PIN_ACCEL_LED, 0);
                                accel_time = 0;
                        }
                }
#endif
                //__WFI();
        }
}
