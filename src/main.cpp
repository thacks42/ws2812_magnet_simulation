#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>

#include "physics.hpp"

// maximum is at about 4000
#define LED_COUNT 114

int _write(int file, char *ptr, int len);

static void rcc_setup_16mhz_hse8mhz(void) {
  rcc_periph_clock_enable(RCC_PWR);
  pwr_set_vos_scale(PWR_SCALE2); /* medium */

  /* switch Flash to 1 WS: */
  flash_64bit_enable();
  flash_prefetch_enable();
  flash_set_ws(1);

  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);

  rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);
  rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);
  rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);

  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);
  rcc_wait_for_sysclk_status(RCC_HSE);

  rcc_osc_off(RCC_PLL);
  rcc_set_pll_configuration(RCC_HSE, RCC_CFGR_PLLMUL_MUL12, RCC_CFGR_PLLDIV_DIV3);
  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);
  
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
  rcc_wait_for_sysclk_status(RCC_PLL);

  rcc_ahb_frequency = 32000000;
  rcc_apb1_frequency = 32000000;
  rcc_apb2_frequency = 32000000;
}

static void clock_setup(void)
{
	//rcc_clock_setup_in_hse_8mhz_out_72mhz();
  //rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_setup_16mhz_hse8mhz();
    
	/* Enable clocks for GPIO ports */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable TIM2 Periph */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA1);

	rcc_periph_clock_enable(RCC_USART1);
}

#define TICK_NS (1000/32)
#define WS0 (350 / TICK_NS)
#define WS1 (800 / TICK_NS)
#define WSP (1300 / TICK_NS)
#define WSL (20000 / TICK_NS)

#define DMA_BANK_SIZE (40 * 8 * 4)
#define DMA_SIZE (DMA_BANK_SIZE*2)
static uint8_t dma_data[DMA_SIZE];
static volatile uint32_t led_data[LED_COUNT];
static volatile uint32_t led_cur = 0;

static void pwm_setup(void) {
	rcc_periph_reset_pulse(RST_TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_disable_oc_output(TIM2, TIM_OC3);
	timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
	timer_disable_oc_clear(TIM2, TIM_OC3);
	timer_set_oc_value(TIM2, TIM_OC3, 0);
	timer_enable_oc_preload(TIM2, TIM_OC3);
	timer_set_oc_polarity_high(TIM2, TIM_OC3);
	timer_enable_oc_output(TIM2, TIM_OC3);

	timer_set_dma_on_update_event(TIM2);
	timer_enable_irq(TIM2, TIM_DIER_UDE); // in fact, enable DMA on update

	timer_enable_preload(TIM2);
	timer_continuous_mode(TIM2);
	timer_set_period(TIM2, WSP);

	timer_enable_counter(TIM2);
}

static void populate_dma_data(uint8_t *dma_data_bank) {
	for(int i=0; i<DMA_BANK_SIZE;) {
		led_cur = led_cur % (LED_COUNT+4);
		if(led_cur < LED_COUNT) {
			uint32_t v = led_data[led_cur];
			for(int j=0; j<32; j++) {
				dma_data_bank[i++] = (v & 0x800000) ? WS1 : WS0;
				v <<= 1;
			}
		} else {
			for(int j=0; j<32; j++) {
				dma_data_bank[i++] = 0;
			}
		}
		led_cur = led_cur + 1;
	}
}


static void dma_int_enable(void) {
	nvic_set_priority(NVIC_DMA1_CHANNEL2_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
}

/* Not used in this example
static void dma_int_disable(void) {
 	nvic_disable_irq(NVIC_DMA1_CHANNEL2_IRQ);
}
*/

static int timer_dma(uint8_t *tx_buf, int tx_len)
{
	dma_int_enable();	

	/* Reset DMA channels*/
	dma_channel_reset(DMA1, DMA_CHANNEL2);

	/* Set up tx dma */
	dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&TIM_CCR3(TIM2));
	dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)tx_buf);
	dma_set_number_of_data(DMA1, DMA_CHANNEL2, tx_len);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL2);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);

	dma_enable_circular_mode(DMA1, DMA_CHANNEL2);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
	dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL2);
	dma_enable_channel(DMA1, DMA_CHANNEL2);

	return 0;
}

/* SPI transmit completed with DMA */
void dma1_channel2_isr(void)
{
	if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
		DMA1_IFCR = DMA1_IFCR | DMA_IFCR_CTCIF2;
		populate_dma_data(&dma_data[DMA_BANK_SIZE]);
	}
	if ((DMA1_ISR & DMA_ISR_HTIF2) != 0) {
		DMA1_IFCR = DMA1_IFCR | DMA_IFCR_CHTIF2;
		populate_dma_data(dma_data);
	}
}

#define LIS3DH_READ(reg) (reg | 0x80)
#define LIS3DH_READ_INC(reg) (reg | 0x80 | 0x40)
#define LIS3DH_WRITE(reg) (reg)
#define LIS3DH_WRITE_INC(reg) (reg | 0x40)
#define LIS3DH_STATUS_REG_AUX 0x07
#define LIS3DH_STATUS_REG_AUX_321OR (1<<7)
#define LIS3DH_STATUS_REG_AUX_3OR (1<<6)
#define LIS3DH_STATUS_REG_AUX_2OR (1<<5)
#define LIS3DH_STATUS_REG_AUX_1OR (1<<4)
#define LIS3DH_STATUS_REG_AUX_321DA (1<<3)
#define LIS3DH_STATUS_REG_AUX_3DA (1<<2)
#define LIS3DH_STATUS_REG_AUX_2DA (1<<1)
#define LIS3DH_STATUS_REG_AUX_1DA (1<<0)
#define LIS3DH_OUT_ADC1_L 0x08
#define LIS3DH_OUT_ADC1_H 0x09
#define LIS3DH_OUT_ADC2_L 0x0A
#define LIS3DH_OUT_ADC2_H 0x0B
#define LIS3DH_OUT_ADC3_L 0x0C
#define LIS3DH_OUT_ADC3_H 0x0D
#define LIS3DH_WHO_AM_I 0x0F
#define LIS3DH_CTRL_REG0 0x1E
#define LIS3DH_CTRL_REG0_SDO_PU_DISC (1<<7)
#define LIS3DH_TEMP_CFG_REG 0x1F
#define LIS3DH_TEMP_CFG_REG_TEMP_EN (1<<6)
#define LIS3DH_TEMP_CFG_REG_ADC_EN (1<<7)
#define LIS3DH_CTRL_REG1 0x20
#define LIS3DH_CTRL_REG1_ODR(rate) (rate<<4)
#define LIS3DH_CTRL_REG1_LPREN (1<<3)
#define LIS3DH_CTRL_REG1_ZEN (1<<2)
#define LIS3DH_CTRL_REG1_YEN (1<<1)
#define LIS3DH_CTRL_REG1_XEN (1<<0)
#define LIS3DH_CTRL_REG2 0x21
#define LIS3DH_CTRL_REG2_HPM(v) (v<<6)
#define LIS3DH_CTRL_REG2_HPCF(v) (v<<4)
#define LIS3DH_CTRL_REG2_FDS (1<<3)
#define LIS3DH_CTRL_REG2_HPCLICK (1<<2)
#define LIS3DH_CTRL_REG2_HP_IA2 (1<<1)
#define LIS3DH_CTRL_REG2_HP_IA1 (1<<0)
#define LIS3DH_CTRL_REG3 0x22
#define LIS3DH_CTRL_REG3_I1_CLICK (1<<7)
#define LIS3DH_CTRL_REG3_I1_IA1 (1<<6)
#define LIS3DH_CTRL_REG3_I1_IA2 (1<<5)
#define LIS3DH_CTRL_REG3_I1_ZYXDA (1<<4)
#define LIS3DH_CTRL_REG3_I1_321DA (1<<3)
#define LIS3DH_CTRL_REG3_I1_WTM (1<<2)
#define LIS3DH_CTRL_REG3_I1_OVERRUN (1<<1)
#define LIS3DH_CTRL_REG4 0x23
#define LIS3DH_CTRL_REG4_BDU (1<<7)
#define LIS3DH_CTRL_REG4_BLE (1<<6)
#define LIS3DH_CTRL_REG4_FS(v) (v<<4)
#define LIS3DH_CTRL_REG4_HR (1<<3)
#define LIS3DH_CTRL_REG4_ST(v) (v<<1)
#define LIS3DH_CTRL_REG4_SIM (1<<0)
#define LIS3DH_CTRL_REG5 0x24
#define LIS3DH_CTRL_REG5_BOOT (1<<7)
#define LIS3DH_CTRL_REG5_FIFO_EN (1<<6)
#define LIS3DH_CTRL_REG5_LIR_INT1 (1<<3)
#define LIS3DH_CTRL_REG5_D4D_INT1 (1<<2)
#define LIS3DH_CTRL_REG5_LIR_INT2 (1<<1)
#define LIS3DH_CTRL_REG5_D4D_INT2 (1<<0)
#define LIS3DH_CTRL_REG6 0x25
#define LIS3DH_CTRL_REG6_I2_CLICK (1<<7)
#define LIS3DH_CTRL_REG6_I2_IA1 (1<<6)
#define LIS3DH_CTRL_REG6_I2_IA2 (1<<5)
#define LIS3DH_CTRL_REG6_I2_BOOT (1<<4)
#define LIS3DH_CTRL_REG6_I2_ACT (1<<3)
#define LIS3DH_CTRL_REG6_INT_POLARITY (1<<1)
#define LIS3DH_REFERENCE 0x26
#define LIS3DH_STATUS_REG 0x27
#define LIS3DH_STATUS_REG_ZYXOR (1<<7)
#define LIS3DH_STATUS_REG_ZOR (1<<6)
#define LIS3DH_STATUS_REG_YOR (1<<5)
#define LIS3DH_STATUS_REG_XOR (1<<4)
#define LIS3DH_STATUS_REG_ZYXDA (1<<3)
#define LIS3DH_STATUS_REG_ZDA (1<<2)
#define LIS3DH_STATUS_REG_YDA (1<<1)
#define LIS3DH_STATUS_REG_XDA (1<<0)
#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_L 0x2C
#define LIS3DH_OUT_Z_H 0x2D
#define LIS3DH_FIFO_CTRL_REG 0x2E
#define LIS3DH_FIFO_CTRL_REG_FM(v) (v<<6)
#define LIS3DH_FIFO_CTRL_REG_TR (1<<5)
#define LIS3DH_FIFO_CTRL_REG_FTH(v) (v<<0)
#define LIS3DH_FIFO_SRC_REG 0x2F
#define LIS3DH_FIFO_SRC_REG_WTM (1<<7)
#define LIS3DH_FIFO_SRC_REG_OVRN_FIFO (1<<6)
#define LIS3DH_FIFO_SRC_REG_EMPTY (1<<5)
#define LIS3DH_FIFO_SRC_REG_FSS(v) (v<<0)
#define LIS3DH_FIFO_SRC_REG_WTM (1<<7)
#define LIS3DH_INT1_CFG 0x30
#define LIS3DH_INT1_CFG_AOI (1<<7)
#define LIS3DH_INT1_CFG_6D (1<<6)
#define LIS3DH_INT1_CFG_ZHIE (1<<5)
#define LIS3DH_INT1_CFG_ZLIE (1<<4)
#define LIS3DH_INT1_CFG_YHIE (1<<3)
#define LIS3DH_INT1_CFG_YLIE (1<<2)
#define LIS3DH_INT1_CFG_XHIE (1<<1)
#define LIS3DH_INT1_CFG_XLIE (1<<0)
#define LIS3DH_INT1_SRC 0x31
#define LIS3DH_INT1_IA (1<<6)
#define LIS3DH_INT1_ZH (1<<5)
#define LIS3DH_INT1_ZL (1<<4)
#define LIS3DH_INT1_YH (1<<3)
#define LIS3DH_INT1_YL (1<<2)
#define LIS3DH_INT1_XH (1<<1)
#define LIS3DH_INT1_XL (1<<0)
#define LIS3DH_INT1_THS 0x32
#define LIS3DH_INT1_DURATION 0x33
#define LIS3DH_INT2_CFG 0x34
#define LIS3DH_INT2_CFG_AOI (1<<7)
#define LIS3DH_INT2_CFG_6D (1<<6)
#define LIS3DH_INT2_CFG_ZHIE (1<<5)
#define LIS3DH_INT2_CFG_ZLIE (1<<4)
#define LIS3DH_INT2_CFG_YHIE (1<<3)
#define LIS3DH_INT2_CFG_YLIE (1<<2)
#define LIS3DH_INT2_CFG_XHIE (1<<1)
#define LIS3DH_INT2_CFG_XLIE (1<<0)
#define LIS3DH_INT2_SRC 0x35
#define LIS3DH_INT2_IA (1<<6)
#define LIS3DH_INT2_ZH (1<<5)
#define LIS3DH_INT2_ZL (1<<4)
#define LIS3DH_INT2_YH (1<<3)
#define LIS3DH_INT2_YL (1<<2)
#define LIS3DH_INT2_XH (1<<1)
#define LIS3DH_INT2_XL (1<<0)
#define LIS3DH_INT2_THS 0x36
#define LIS3DH_INT2_DURATION 0x37
#define LIS3DH_CLICK_CFG 0x38
#define LIS3DH_CLICK_CFG_ZD (1<<5)
#define LIS3DH_CLICK_CFG_ZS (1<<4)
#define LIS3DH_CLICK_CFG_YD (1<<3)
#define LIS3DH_CLICK_CFG_YS (1<<2)
#define LIS3DH_CLICK_CFG_XD (1<<1)
#define LIS3DH_CLICK_CFG_XS (1<<0)
#define LIS3DH_CLICK_SRC 0x39
#define LIS3DH_CLICK_SRC_IA (1<<6)
#define LIS3DH_CLICK_SRC_DCLICK (1<<5)
#define LIS3DH_CLICK_SRC_SCLICK (1<<4)
#define LIS3DH_CLICK_SRC_SIGN (1<<3)
#define LIS3DH_CLICK_SRC_Z (1<<2)
#define LIS3DH_CLICK_SRC_Y (1<<1)
#define LIS3DH_CLICK_SRC_X (1<<0)
#define LIS3DH_CLICK_THS 0x3A
#define LIS3DH_CLICK_THS_LIR_CLICK (1<<7)
#define LIS3DH_CLICK_THS_THS(v) (v<<0)
#define LIS3DH_TIME_LIMIT 0x3B
#define LIS3DH_TIME_LATENCY 0x3C
#define LIS3DH_TIME_WINDOW 0x3D
#define LIS3DH_ACT_THS 0x3E
#define LIS3DH_ACT_DUR 0x3F

#define ACC_CS_HIGH gpio_set(GPIOA,GPIO3)
#define ACC_CS_LOW gpio_clear(GPIOA,GPIO3)

int filter(int v) {
  static int p = 0;
  static int buf[16];
  const int coeff[16] = { 2086, 2676, 3312, 3957, 4562, 5075, 5449, 5647, 5647, 5449, 5075, 4562, 3957, 3312, 2676, 2086 };
  buf[p] = v;
  int ret = 0;
  for(int i=0; i<16; i++) {
    ret += (coeff[i] * buf[(p-i) % 16]);
  }
  p = (p+1) % 16;
  return ret;
}

int level = 0;
void exti0_isr(void) {
  ACC_CS_LOW;
  (void) spi_xfer(SPI1, LIS3DH_READ_INC(LIS3DH_OUT_X_L));
  int16_t x = (int16_t)(spi_xfer(SPI1, 0xFF) | (spi_xfer(SPI1, 0xFF) << 8));
  int16_t y = (int16_t)(spi_xfer(SPI1, 0xFF) | (spi_xfer(SPI1, 0xFF) << 8));
  int16_t z = (int16_t)(spi_xfer(SPI1, 0xFF) | (spi_xfer(SPI1, 0xFF) << 8));
  ACC_CS_HIGH;

  (void)x;
  (void)z;

  gpio_set(GPIOB, 1<<level);
  level = abs(filter(y)) / (2048 * 65536);
  if(level < 0) level = 0;
  if(level > 15) level = 15;
  gpio_clear(GPIOB, 1<<level);

  // TODO: do something with the data
  exti_reset_request(EXTI0);
}

static void acc_init(void) {
  unsigned res;
  //rcc_periph_reset_pulse(RCC_SPI1);
  rcc_periph_clock_enable(RCC_SPI1);
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable(SPI1);
  for(int tries=100; tries>0; tries++) {
    for(int delay=0; delay<400000; delay++) { __asm__("nop"); } /* Accelerometer needs 5ms starting at power-up for booting */
    ACC_CS_LOW;
    (void) spi_xfer(SPI1, LIS3DH_READ(LIS3DH_WHO_AM_I));
    res = spi_xfer(SPI1, 0xFF);
    ACC_CS_HIGH;
    if(res == 0x33) goto accelerometer_up;
  }
  gpio_toggle(GPIOB, GPIO15);
  __asm__("bkpt 0");

accelerometer_up:
  // enable EXTI
  nvic_enable_irq(NVIC_EXTI0_IRQ);
  nvic_set_priority(NVIC_EXTI0_IRQ, 0x10);
  exti_select_source(EXTI0, GPIOA);
  exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI0);

  // select HR bit in CTRL_REG4, set FS to 4g
  ACC_CS_LOW;
  (void) spi_xfer(SPI1, LIS3DH_WRITE(LIS3DH_CTRL_REG4));
  (void) spi_xfer(SPI1, LIS3DH_CTRL_REG4_HR | LIS3DH_CTRL_REG4_FS(0));
  ACC_CS_HIGH;

#if 0
  // high pass filter config
  ACC_CS_LOW;
  (void) spi_xfer(SPI1, LIS3DH_WRITE(LIS3DH_CTRL_REG2));
  (void) spi_xfer(SPI1, LIS3DH_CTRL_REG2_FDS | LIS3DH_CTRL_REG2_HPM(0) | LIS3DH_CTRL_REG2_HPCF(3));
  ACC_CS_HIGH;
#endif

  // enable interrupts
  ACC_CS_LOW;
  (void) spi_xfer(SPI1, LIS3DH_WRITE(LIS3DH_CTRL_REG3));
  (void) spi_xfer(SPI1, LIS3DH_CTRL_REG3_I1_ZYXDA);
  ACC_CS_HIGH;

  // select preferred ODR
  // enable axes - we're leaving all enabled, so nothing needed here
  ACC_CS_LOW;
  (void) spi_xfer(SPI1, LIS3DH_WRITE(LIS3DH_CTRL_REG1));
  (void) spi_xfer(SPI1, LIS3DH_CTRL_REG1_ODR(6) | LIS3DH_CTRL_REG1_ZEN | LIS3DH_CTRL_REG1_YEN | LIS3DH_CTRL_REG1_XEN);
  ACC_CS_HIGH;
}

void bad_sleep(){
    for(int i = 0; i < 4444; i++){
        __asm__("nop");
    }
}

template<size_t n>
void update_leds_from_sim(particle_simulation<n, LED_COUNT>& sim){
    sim.update_leds();
    for(int i = 0; i < LED_COUNT; i++){
        led_data[i] = (sim.leds[i].r << 8) | (sim.leds[i].b) | (sim.leds[i].g << 16);
    }
}

#define ALL_LEDS (GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8|GPIO9|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO15)
int main(void)
{

	clock_setup();

	// LED
  gpio_set(GPIOB, ALL_LEDS);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_400KHZ, ALL_LEDS);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ALL_LEDS);

  gpio_set(GPIOA, GPIO3); // CS for Acc
  gpio_set_af(GPIOA, GPIO_AF5, GPIO5|GPIO6|GPIO7); // SCK/MISO/MOSI
  gpio_set_af(GPIOA, GPIO_AF7, GPIO9|GPIO10); // TX/RX
  gpio_set_af(GPIOA, GPIO_AF1, GPIO2); // PWM OUT TIM2_CH3
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO5|GPIO7 | GPIO9 | GPIO3 | GPIO2);
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO5|GPIO6|GPIO7  |  GPIO9|GPIO10  /* |  GPIO11|GPIO12 */ );  // SPI1, USART1, USB

	memset(dma_data, 0, DMA_SIZE);
	memset((void*)led_data, 0x0a, LED_COUNT*sizeof(*led_data));
	//populate_dma_data(dma_data);
	//populate_dma_data(&dma_data[DMA_BANK_SIZE]);
    led_data[0] = 0xff00;

	timer_dma(dma_data, DMA_SIZE);
	pwm_setup();
    
    particle_simulation<10, LED_COUNT> sim;
    
  acc_init();
    

	while (1) {
		sim.step(make_fixed<int32_t, 16>(1));
        update_leds_from_sim(sim);
        bad_sleep();
	}
}
