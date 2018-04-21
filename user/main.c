#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "port.h"
#include "uwb.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "DS TWR RESP v1.2"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(void)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    uwb_init();
    /* Loop forever responding to ranging requests. */
    while (1)
    {
        uwb_task();
    }
}

