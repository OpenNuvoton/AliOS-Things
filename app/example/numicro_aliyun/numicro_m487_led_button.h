#ifndef __NUMICRO_M487_LED_BUTTON_H__
#define __NUMICRO_M487_LED_BUTTON_H__

#define DEF_BUTTON_IDX_SW3 	16
#define DEF_BUTTON_IDX_SW2 	17
#define DEF_LED_RED			18
#define DEF_LED_YELLOW 		19
#define DEF_LED_GREEN 		20

typedef enum {
	e_LED_R=DEF_LED_RED,
	e_LED_Y=DEF_LED_YELLOW,	
	e_LED_G=DEF_LED_GREEN
} E_LED;

typedef enum {
	e_BTN_SW2=DEF_BUTTON_IDX_SW2,
	e_BTN_SW3=DEF_BUTTON_IDX_SW3
} E_UserEvent;

void led_control(E_LED eled, int on_off);
void led_button_start(void);
int handle_button_event(E_UserEvent eBtn);

#endif  // __NUMICRO_M487_LED_BUTTON_H__
