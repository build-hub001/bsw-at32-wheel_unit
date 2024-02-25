#include "at32f413_wk_config.h"

#include "at32f413_board.h"
#include "at32f413_clock.h"


void _usart3_init(void);
void _tmr1_init(void);
void _gpio_config(void);


#define COUNTOF(a)                       (sizeof(a) / sizeof(*(a)))
#define USART3_TX_BUFFER_SIZE            (COUNTOF(usart3_tx_buffer) - 1)

uint8_t usart3_tx_buffer[11]  = {0};
uint8_t usart3_rx_buffer[11]  = {0};
uint8_t usart3_tx_counter     = 0x00;
uint8_t usart3_rx_counter     = 0x00;
uint8_t usart3_tx_buffer_size = USART3_TX_BUFFER_SIZE;
uint8_t usart3_rx_buffer_size = USART3_TX_BUFFER_SIZE;

gpio_init_type          gpio_init_struct = {0};
crm_clocks_freq_type    crm_clocks_freq_struct = {0};
tmr_output_config_type  tmr_output_struct;

uint16_t timer_period   = 0;
uint16_t channel1_pulse = 0;
uint16_t channel2_pulse = 0;
uint16_t channel3_pulse = 0;
uint16_t channel4_pulse = 0;


int main(void)
{
    wk_system_clock_config();
    wk_periph_clock_config();
    wk_nvic_config();
    
    at32_led_init(LED2);
    at32_led_init(LED3);
    at32_led_init(LED4);

    at32_led_off(LED2);
    at32_led_off(LED3);
    at32_led_off(LED4);

    // delay_init();
    uart_print_init(115200);

    _tmr1_init();
    _usart3_init();
    _gpio_config();

    while(1)
    {
        usart3_rx_counter=0;
        while(usart3_rx_counter <= usart3_rx_buffer_size)
        {
            while(usart_flag_get(USART3, USART_RDBF_FLAG) == RESET);
            usart3_rx_buffer[usart3_rx_counter] = usart_data_receive(USART3);
            usart3_rx_counter++;
        }

        for (int i = 0; i <= usart3_rx_buffer_size; i++)
        {
            printf("%02x ", usart3_rx_buffer[i]);
        }
        printf( "\n");
        at32_led_toggle(LED3);
        
        channel1_pulse = (uint16_t)(((uint32_t) usart3_rx_buffer[9] * (timer_period - 1)) / 255);
        channel2_pulse = (uint16_t)(((uint32_t) usart3_rx_buffer[10] * (timer_period - 1)) / 255);

        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel2_pulse);
        
    }
}

void _gpio_config(void)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* gpio output config */
  gpio_bits_reset(GPIOC, GPIO_DIR_1_PIN | GPIO_DIR_2_PIN);

  gpio_init_struct.gpio_drive_strength  = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type        = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode            = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins            = GPIO_DIR_1_PIN | GPIO_DIR_2_PIN;
  gpio_init_struct.gpio_pull            = GPIO_PULL_NONE;
  gpio_init(GPIOC, &gpio_init_struct);
}



void _tmr1_init(void)
{
  /* enable tmr1/gpioa/gpiob clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  /* timer1 output pin Configuration */
  gpio_init_struct.gpio_pins            = GPIO_PINS_8 | GPIO_PINS_10;
  gpio_init_struct.gpio_mode            = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type        = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull            = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength  = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_init_struct);


  /* tmr1 configuration generate 7 pwm signals with 4 different duty cycles:
   prescaler = 0, tmr1 counter clock = system_core_clock

   the objective is to generate 7 pwm signal at 17.57 khz:
     - tim1_period = (system_core_clock / 17570) - 1
   the channel 1 and channel 1n duty cycle is set to 50%
   the channel 2 and channel 2n duty cycle is set to 37.5%
   the channel 3 and channel 3n duty cycle is set to 25%
   the channel 4 duty cycle is set to 12.5%
   the timer pulse is calculated as follows:
     - channelxpulse = duty_cycle * (tim1_period - 1) / 100 */

  /* compute the value to be set in arr regiter to generate signal frequency at 17.57 khz */
  timer_period = (crm_clocks_freq_struct.sclk_freq / 1000 ) - 1;

  /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
  channel1_pulse = (uint16_t)(((uint32_t) 5 * (timer_period - 1)) / 10);

  /* compute ccr2 value to generate a duty cycle at 25.5%  for channel 2 and 2n */
  channel2_pulse = (uint16_t)(((uint32_t) 255 * (timer_period - 1)) / 1000);

  /* compute ccr3 value to generate a duty cycle at 25%  for channel 3 and 3n */
  channel3_pulse = (uint16_t)(((uint32_t) 75 * (timer_period - 1)) / 100);

  /* compute ccr4 value to generate a duty cycle at 12.5%  for channel 4 */
  channel4_pulse = (uint16_t)(((uint32_t) 125 * (timer_period- 1)) / 1000);

  tmr_base_init(TMR1, timer_period, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  /* channel 1, 2, 3 and 4 configuration in output mode */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode             = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state     = TRUE;
  tmr_output_struct.oc_polarity         = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.oc_idle_state       = TRUE;
  tmr_output_struct.occ_output_state    = TRUE;
  tmr_output_struct.occ_polarity        = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_idle_state      = FALSE;

  /* channel 1 */
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);

  /* channel 3 */
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
}

void _usart3_init(void)
{
  /* enable the usart3 and gpio clock */
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* configure the TX pin */
  gpio_init_struct.gpio_drive_strength  = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type        = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode            = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins            = GPIO_PINS_10;
  gpio_init_struct.gpio_pull            = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the RX pin */
  gpio_init_struct.gpio_drive_strength  = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type        = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode            = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins            = GPIO_PINS_11;
  gpio_init_struct.gpio_pull            = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);

  usart_hardware_flow_control_set(USART3, USART_HARDWARE_FLOW_NONE);

  usart_enable(USART3, TRUE);
}