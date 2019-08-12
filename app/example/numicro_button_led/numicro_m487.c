#include "aos_hal.h"
#include "board.h"

#define DEF_BUTTON_IDX_SW2 	16
#define DEF_BUTTON_IDX_SW3 	17
#define DEF_LED_RED			18
#define DEF_LED_YELLOW 		19
#define DEF_LED_GREEN 		20

extern gpio_dev_t board_gpio_table[];
static void key_process(input_event_t *eventinfo, void *priv_data)
{
    if (eventinfo->type != EV_KEY) {
        return;
    }

    //printf("[%d %d %d %d]\n", eventinfo->time, eventinfo->type, eventinfo->code, eventinfo->value);    
    if ( (eventinfo->code == DEF_BUTTON_IDX_SW2) || (eventinfo->code==DEF_BUTTON_IDX_SW3) )
    {
		printf("KeyCode=%d, ", eventinfo->code);
		switch(eventinfo->value)
		{
			case VALUE_KEY_CLICK:
		       printf("Shorter time pressing (<2 secs)\n");
			break;
			case VALUE_KEY_LTCLICK:
		       printf("Longer time pressing (<5 secs)\n");
			break;			
			case VALUE_KEY_LLTCLICK: 
		       printf("Longest time pressing (<5 secs)\n");
			break;
			default:
				return;
		}

        //LED_RED
        hal_gpio_output_toggle(&board_gpio_table[DEF_LED_RED]);
        //LED_YELLOW
        hal_gpio_output_toggle(&board_gpio_table[DEF_LED_YELLOW]);
        //LED_GREEN
        hal_gpio_output_toggle(&board_gpio_table[DEF_LED_GREEN]);
	}
}

static uint64_t   awss_time = 0;
static void key_poll_func(void *arg)
{
    uint32_t level;
    uint64_t diff;
    gpio_dev_t* psgpio;

    hal_gpio_input_get((gpio_dev_t*)arg, &level);
    psgpio = (gpio_dev_t*)arg;

    if (level == 0) { // still pressed
        aos_post_delayed_action(10, key_poll_func, arg);
    } else {		// released
        diff = aos_now_ms() - awss_time;
        if (diff > 5000) { 	/*long long press, 5 seconds */
            awss_time = 0;
            aos_post_event(EV_KEY, psgpio->port, VALUE_KEY_LLTCLICK);
        } else if (diff > 2000) { /* long press, 2 seconds */
            awss_time = 0;
            aos_post_event(EV_KEY, psgpio->port, VALUE_KEY_LTCLICK);
        } else if (diff > 40) { /* short press, 40 miliseconds */
            awss_time = 0;
            aos_post_event(EV_KEY, psgpio->port, VALUE_KEY_CLICK);
        } else {
            aos_post_delayed_action(10, key_poll_func, arg);
        }
    }
}

static void key_proc_work(void *arg)
{
    aos_schedule_call(key_poll_func, arg);
}

static void handle_awss_key(void *arg)
{
    uint32_t gpio_value;

    hal_gpio_input_get((gpio_dev_t*)arg, &gpio_value);
    if (gpio_value == 0 && awss_time == 0) {
        awss_time = aos_now_ms();
        aos_loop_schedule_work(0, key_proc_work, arg, NULL, NULL);
    }
}

static void board_gpio_init(void)
{
    hal_gpio_init(&board_gpio_table[DEF_BUTTON_IDX_SW2]);
    hal_gpio_init(&board_gpio_table[DEF_BUTTON_IDX_SW3]);
    hal_gpio_init(&board_gpio_table[DEF_LED_RED]);
    hal_gpio_init(&board_gpio_table[DEF_LED_YELLOW]);
    hal_gpio_init(&board_gpio_table[DEF_LED_GREEN]);
}

static void board_button_init(void)
{
    // SW2: enable irq
    hal_gpio_enable_irq(&board_gpio_table[DEF_BUTTON_IDX_SW2], IRQ_TRIGGER_FALLING_EDGE, handle_awss_key, (void*)&board_gpio_table[DEF_BUTTON_IDX_SW2]);

    // SW3: enable irq
    hal_gpio_enable_irq(&board_gpio_table[DEF_BUTTON_IDX_SW3], IRQ_TRIGGER_FALLING_EDGE, handle_awss_key, (void*)&board_gpio_table[DEF_BUTTON_IDX_SW3]);

    // register key event
    aos_register_event_filter(EV_KEY, key_process, NULL);
}

void board_leds_init(void)
{
    //Light on LED_RED by default
    hal_gpio_output_low(&board_gpio_table[DEF_LED_RED]);

    //Light on LED_YELLOW by default
	hal_gpio_output_low(&board_gpio_table[DEF_LED_YELLOW]);

    //Light on LED_GREEN by default    
    hal_gpio_output_low(&board_gpio_table[DEF_LED_GREEN]);
}

static void testcase_init()
{
    // GPIO pin initialize
    board_gpio_init();

    // Button SW2/SW3
    board_button_init();

    // LED
    board_leds_init();
}

static void app_delayed_action(void *arg)
{
    printf("hello world !!\r\n");
		// Register next callback function.
    aos_post_delayed_action(1000, app_delayed_action, NULL);
}

int application_start(int argc, char *argv[])
{
    int count = 0;

    printf("NuMicro LED & Button demo started.\n");

    // Initialize
    testcase_init();   

		// Register a timer callback function
		aos_post_delayed_action(1000, app_delayed_action, NULL);
	
    // Run
    aos_loop_run();
    return 0;
}
