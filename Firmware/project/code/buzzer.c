#include "buzzer.h"

void buzzer_init(void)
{
    gpio_init(C13, GPO, 0, GPO_PUSH_PULL);
}

//·äÃùÆ÷Ãù½Ð x ´Î£¬¼ä¸ô y ms
void alarm_times(uint8 x, uint16 y)
{
    while(x--)
    {
        gpio_set_level(C13, 1);
        vTaskDelay(y);
        gpio_set_level(C13, 0);
        vTaskDelay(y);
    }
}




