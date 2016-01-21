/* Name:        hd44780
 * Author:      Alex Ferm <aferm@petropower.com>
 * Platform:    Linux 3.X+ (Requires Device Tree Support, Tested on 4.1.10)
 * 
 * 
 * Description: A kernel level Linux platform device driver to provide read 
 *              and write access to an HD44780 LCD controller using pinctrl
 *              and gpio.
 * 
 * Relevant Documentation:
 *      HD44780 Datasheet: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 * 
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/gpio.h>         // linux gpio interface

#include <linux/pinctrl/consumer.h>     // linux pinctrl interface
#include <linux/of_device.h>    // linux openfirmware device interface
#include <linux/platform_device.h>      // linux platform device
#include <linux/of_gpio.h>

#include <linux/delay.h>
#include <linux/string.h>

#include "hd44780.h"

/********* Device Structures *************/

static struct class_compat *hd44780_class;

/************* LCD Structure *************/

LCD_INFO lcd_info = {
    .current_mode = LCD_MODE_UNSET,
    .display_state = DISPLAY_OFF,
    .cursor_mode = CURSOR_OFF,
    .cursor_direction = RIGHT,
    .display_autoshift = false,
    .has_gpio = false,
};

/************ Core Functions *************/

/* Function: lcd_pin_setup
 * 
 * Description:
 *      Set up a GPIO pin for the LCD in the specified direction.
 * 
 * Input:
 *      pin_number: GPIO number of the pin to setup
 *      gpio_direction: INPUT_PIN/OUTPUT_PIN
 * 
 * Output:
 *      int: 0 on success, other on error
 * 
 */
static int lcd_pin_setup(unsigned int pin_number, PIN_DIRECTION gpio_direction)
{
    int ret;

    // Request GPIO allocation
    ret =
        devm_gpio_request(&lcd_info.pdev->dev, pin_number, "GPIO pin request");
    if (ret != 0) {
        printk(KERN_ERR "HD44780: Failed to request GPIO pin %d.\n",
               pin_number);
        return ret;
    }
    // Set GPIO pin direction
    switch (gpio_direction) {
    case OUTPUT_PIN:
        // Set the direction to output with an initial value of 0.
        ret = gpio_direction_output(pin_number, 0);
        break;
    case INPUT_PIN:
        ret = gpio_direction_input(pin_number);
    }

    if (ret != 0) {
        printk(KERN_ERR
               "HD44780: Failed to set GPIO pin direction %d.\n", pin_number);
        return ret;
    }
    // Return value when there is no error
    return 0;
}

/* Function: lcd_pin_release_all
 * 
 * Description:
 *      Release all used GPIO by calling devm_gpio_free
 * 
 */
static void lcd_pin_release_all(void)
{
    if (lcd_info.has_gpio) {
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.rs_gpio);
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.rw_gpio);
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.e_gpio);

        if (lcd_info.bus_width == 8) {
            devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db0_gpio);
            devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db1_gpio);
            devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db2_gpio);
            devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db3_gpio);
        }

        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db4_gpio);
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db5_gpio);
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db6_gpio);
        devm_gpio_free(&lcd_info.pdev->dev, lcd_info.db7_gpio);
    }
    lcd_info.has_gpio = false;
}

/* Function: lcd_set_rw
 * 
 * Description:
 *      Set pincrtl/gpio state
 * 
 * Input:
 *      mode: LCD_MODE (LCD_READ/LCD_WRITE) defined in hd44780.h
 * 
 * Output:
 *      int: 0 on success, other on error
 * 
 */
static int lcd_set_rw(LCD_MODE mode)
{
    int ret;

    lcd_pin_release_all();
    switch (mode) {
    case LCD_READ:
        ret = pinctrl_select_state(lcd_info.p, lcd_info.read_state);
        if (ret < 0) {
            printk(KERN_ERR "HD44780: Error selecting read pinctrl state.\n");
            lcd_info.current_mode = LCD_MODE_UNSET;
            return ret;
        }

        ret = 0;

        // OR all return codes. In order to speed up the error checking process and reduce code duplication.
        ret |= lcd_pin_setup(lcd_info.e_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.rw_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.rs_gpio, OUTPUT_PIN);

        if (lcd_info.bus_width == 8) {
            ret |= lcd_pin_setup(lcd_info.db0_gpio, INPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db1_gpio, INPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db2_gpio, INPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db3_gpio, INPUT_PIN);
        }

        ret |= lcd_pin_setup(lcd_info.db4_gpio, INPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db5_gpio, INPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db6_gpio, INPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db7_gpio, INPUT_PIN);

        if (ret != 0) {
            lcd_pin_release_all();
            lcd_info.current_mode = LCD_MODE_UNSET;
            return ret;
        }

        gpio_set_value(lcd_info.rw_gpio, 1);

        break;
    case LCD_WRITE:
        ret = pinctrl_select_state(lcd_info.p, lcd_info.write_state);
        if (ret < 0) {
            printk(KERN_ERR "HD44780: Error selecting write pinctrl state.\n");
            lcd_info.current_mode = LCD_MODE_UNSET;
            return ret;
        }

        ret = 0;

        ret |= lcd_pin_setup(lcd_info.e_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.rw_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.rs_gpio, OUTPUT_PIN);

        if (lcd_info.bus_width == 8) {
            ret |= lcd_pin_setup(lcd_info.db0_gpio, OUTPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db1_gpio, OUTPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db2_gpio, OUTPUT_PIN);
            ret |= lcd_pin_setup(lcd_info.db3_gpio, OUTPUT_PIN);
        }

        ret |= lcd_pin_setup(lcd_info.db4_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db5_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db6_gpio, OUTPUT_PIN);
        ret |= lcd_pin_setup(lcd_info.db7_gpio, OUTPUT_PIN);

        if (ret != 0) {
            lcd_pin_release_all();
            lcd_info.current_mode = LCD_MODE_UNSET;
            return ret;
        }

        gpio_set_value(lcd_info.rw_gpio, 0);

        break;
    default:
        printk(KERN_ERR "HD44780: Error invalid mode specified.\n");
        return -1;
    }
    lcd_info.current_mode = mode;
    lcd_info.has_gpio = true;
    return 0;
}

/* Function: lcd_read
 * 
 * Description:
 *      Read data/(busy flag + address) from the HD44780 LCD controller
 * 
 * Input:
 *      data_mode: flag specifying the value of the RS pin. (DATA_MODE/COMMAND_MODE)
 *              ( COMMAND_MODE is used to read the current address and busy flag )
 *
 * Output:
 *      1 byte of data cast as an unsigned char.
 * 
 */
unsigned char lcd_read(LCD_RS_MODE data_mode)
{

    unsigned char c;
    int counter;
    unsigned char data_bits[8];

    // Make sure LCD is in READ mode
    if (lcd_info.current_mode != LCD_READ) {
        if (lcd_set_rw(LCD_READ)) {
            printk(KERN_ERR "HD44780: Failed to set LCD to read mode.\n");
            return 0x80;
        }
    }

    switch (lcd_info.bus_width) {
    case 8:
        // Set to data mode
        gpio_set_value(lcd_info.rs_gpio, data_mode);
        udelay(15);

        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);

        // Grab the entire byte from the data bus
        data_bits[7] = gpio_get_value(lcd_info.db7_gpio);
        data_bits[6] = gpio_get_value(lcd_info.db6_gpio);
        data_bits[5] = gpio_get_value(lcd_info.db5_gpio);
        data_bits[4] = gpio_get_value(lcd_info.db4_gpio);
        data_bits[3] = gpio_get_value(lcd_info.db3_gpio);
        data_bits[2] = gpio_get_value(lcd_info.db2_gpio);
        data_bits[1] = gpio_get_value(lcd_info.db1_gpio);
        data_bits[0] = gpio_get_value(lcd_info.db0_gpio);

        gpio_set_value(lcd_info.e_gpio, 0);
        udelay(15);

        break;

    case 4:
        // Set to data mode
        gpio_set_value(lcd_info.rs_gpio, data_mode);
        udelay(15);

        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);

        // Grab the most significant nibble from the data bus
        data_bits[7] = gpio_get_value(lcd_info.db7_gpio);
        data_bits[6] = gpio_get_value(lcd_info.db6_gpio);
        data_bits[5] = gpio_get_value(lcd_info.db5_gpio);
        data_bits[4] = gpio_get_value(lcd_info.db4_gpio);

        gpio_set_value(lcd_info.e_gpio, 0);

        // Delay for t(cycE) to ensure acurate reads
        udelay(1);

        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);

        // Grab the least significant nibble from the data bus
        data_bits[3] = gpio_get_value(lcd_info.db7_gpio);
        data_bits[2] = gpio_get_value(lcd_info.db6_gpio);
        data_bits[1] = gpio_get_value(lcd_info.db5_gpio);
        data_bits[0] = gpio_get_value(lcd_info.db4_gpio);

        gpio_set_value(lcd_info.e_gpio, 0);

        break;
    }

    // Combine bits
    c = 0x00;
    for (counter = 0; counter < 8; counter++) {
        c |= ((data_bits[counter] << counter) & (0x1 << counter));
    }

    return c;
}

/* Function: lcd_read_data
 * 
 * Description:
 *      Read 1 byte of data from the HD44780 LCD controller.
 *
 * Output:
 *      1 byte of data cast as an unsigned char.
 * 
 */
unsigned char lcd_read_data(void)
{

    return lcd_read(DATA_MODE);

}

/* Function: lcd_check_busy
 * 
 * Description:
 *      Check and return the busy flag.
 * 
 * Output:
 *      busy_flag: 1 = busy, 0 = ready
 * 
 */
int lcd_check_busy(void)
{

    return (lcd_read(COMMAND_MODE) & LCD_BUSY_FLAG) > 0;

}

/* Funciont: lcd_busy_wait
 * 
 * Description:
 *      Wait until the LCD is done writing
 * 
 */
void lcd_busy_wait(void)
{
    int i;

    // Sleep 15 microseconds at a time until the LCD is ready, or maximum try count is exceeded.
    for (i = 0; (lcd_check_busy() == 1) && (i < MAX_BUSY_WAIT); i++) {
        // Short sleep to make sure we aren't completely blocking the CPU
        udelay(15);
    }

    if (i > 0) {
        printk(KERN_DEBUG
               "HD44780: Waited on LCD for %dus, checked busy flag %d(MAX=%d) times.\n",
               i * 15, i + 1, MAX_BUSY_WAIT);
    }
}

/* Function: lcd_get_addr
 * 
 * Description:
 *      Get the current address the cursor is pointed to.
 * 
 * Output:
 *      address: the address the 
 * 
 */
unsigned char lcd_get_addr(void)
{
    // Read the current address and mask out the busy flag
    return lcd_read(COMMAND_MODE) & LCD_ADDRESS;

}

/* Function: lcd_send
 * 
 * Description:
 *      Send data/command to the HD44780 LCD controller
 * 
 * Input:
 *      data: Character/Command to send to the LCD controller
 *      bus_width: bus width for command
 *              ( Specifying 8 when lcd_info.bus_width = 4 will result in 
 *              only the most significant nibble being sent. This is useful
 *              during initialization. )
 *      data_mode: flag specifying the value of the RS pin. (DATA_MODE/COMMAND_MODE)
 * 
 */
static void lcd_send(unsigned char data, int bus_width, LCD_RS_MODE data_mode)
{

    unsigned char data_bits[8];
    int counter = 0;

    // Make sure LCD is in WRITE mode
    if (lcd_info.current_mode != LCD_WRITE) {
        if (lcd_set_rw(LCD_WRITE)) {
            printk(KERN_ERR "HD44780: Failed to set LCD to write mode.\n");
            return;
        }
    }
    // Break up the data into bits
    for (counter = 0; counter < 8; counter++) {
        data_bits[counter] = ((data) & (0x1 << counter)) >> (counter);
    }

    switch (bus_width) {
    case 8:
        // Most significant nibble (from bit 7 to bit 4)
        gpio_set_value(lcd_info.db7_gpio, data_bits[7]);
        gpio_set_value(lcd_info.db6_gpio, data_bits[6]);
        gpio_set_value(lcd_info.db5_gpio, data_bits[5]);
        gpio_set_value(lcd_info.db4_gpio, data_bits[4]);

        // Least significant nibble (from bit 3 to bit 0), if the hardware allows
        if (bus_width == lcd_info.bus_width) {
            // Put the bits on the line

            gpio_set_value(lcd_info.db3_gpio, data_bits[3]);
            gpio_set_value(lcd_info.db2_gpio, data_bits[2]);
            gpio_set_value(lcd_info.db1_gpio, data_bits[1]);
            gpio_set_value(lcd_info.db0_gpio, data_bits[0]);

        }
        // Set mode, (Command/Data)
        gpio_set_value(lcd_info.rs_gpio, data_mode);
        udelay(15);

        // Produce clock trigger
        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);
        gpio_set_value(lcd_info.e_gpio, 0);
        break;

    case 4:
        // Most significant nibble (from bit 7 to bit 4)
        gpio_set_value(lcd_info.db7_gpio, data_bits[7]);
        gpio_set_value(lcd_info.db6_gpio, data_bits[6]);
        gpio_set_value(lcd_info.db5_gpio, data_bits[5]);
        gpio_set_value(lcd_info.db4_gpio, data_bits[4]);

        // Set mode, (Command/Data)
        gpio_set_value(lcd_info.rs_gpio, data_mode);
        udelay(15);

        // Produce clock trigger
        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);
        gpio_set_value(lcd_info.e_gpio, 0);

        udelay(15);

        // Part 2. Lower 4 bit data (from bit 3 to bit 0)
        gpio_set_value(lcd_info.db7_gpio, data_bits[3]);
        gpio_set_value(lcd_info.db6_gpio, data_bits[2]);
        gpio_set_value(lcd_info.db5_gpio, data_bits[1]);
        gpio_set_value(lcd_info.db4_gpio, data_bits[0]);

        // Produce clock trigger
        gpio_set_value(lcd_info.e_gpio, 1);
        udelay(15);
        gpio_set_value(lcd_info.e_gpio, 0);
        break;
    default:
        printk(KERN_ERR "HD44780: Invalid bus width, %d.\n", bus_width);
    }

    lcd_busy_wait();
}

/* Function: lcd_instruction
 * 
 * Description:
 *      Send a command to the HD44780 LCD controller.
 *
 * Input:
 *      command: command to be sent to the LCD controller.
 * 
 */
static void lcd_instruction(char command)
{
    lcd_send(command, lcd_info.bus_width, COMMAND_MODE);
}

/* Function: lcd_set_addr
 * 
 * Description:
 *      Set the address counter to the specified address in CGRAM or DDRAM
 * 
 * Input:
 *      address: Address to set the address counter to.
 *      memspace: Which memory space to go to. (DDRAM/CGRAM)
 * 
 */
static void lcd_set_addr(unsigned char address, MEMORY_SPACE memspace)
{
    unsigned char command = 0x00;
    switch (memspace) {
    case CGRAM:
        command = LCD_SET_CGRAM_ADDR;
        break;
    case DDRAM:
        command = LCD_SET_DDRAM_ADDR;
        break;
    default:
        printk(KERN_ERR "HD44780: Invalid memory space.\n");
        return;
    }
    command |= address & LCD_ADDRESS;
    lcd_instruction(command);
    return;
}

/* Function: lcd_clear
 * 
 * Description:
 *      Clear the display on the LCD
 * 
 */
static void lcd_clear(void)
{
    /* Clear Display */
    /* Instruction 00000001b */
    lcd_instruction(LCD_CLEAR);
    printk(KERN_DEBUG "HD44780: Display cleared.\n");
}

/* Function: lcd_home
 * 
 * Description:
 *      Set address counter to 0x00 and unshift display
 * 
 */
static void lcd_home(void)
{
    /* Clear Display */
    /* Instruction 00000010b */
    lcd_instruction(LCD_HOME);
    printk(KERN_DEBUG "HD44780: Homed display.\n");
}

/* Function: lcd_shift_cursor
 * 
 * Description:
 *      Shift the cursor one cell in the specified direction.
 * 
 * Input:
 *      dir: Direction to shift (LEFT/RIGHT)
 * 
 * Instruction 0001(S/C)(R/L)--b
 * 
 */
static void lcd_shift_cursor(LCD_DIR dir)
{
    unsigned char command = LCD_SHIFT | SHIFT_CURSOR;
    switch (dir) {
    case LEFT:
        command |= SHIFT_LEFT;
        break;
    case RIGHT:
        command |= SHIFT_RIGHT;
        break;
    default:
        return;
    }
    lcd_instruction(command);
}

/* Function: lcd_shift_display
 * 
 * Description:
 *      Shift the display one cell in the specified direction.
 * 
 * Input:
 *      dir: Direction to shift (LEFT/RIGHT)
 * 
 * Instruction 0001(S/C)(R/L)--b
 * 
 */
static void lcd_shift_display(LCD_DIR dir)
{
    unsigned char command = LCD_SHIFT | SHIFT_DISPLAY;
    switch (dir) {
    case LEFT:
        command |= SHIFT_LEFT;
        break;
    case RIGHT:
        command |= SHIFT_RIGHT;
        break;
    default:
        return;
    }
    lcd_instruction(command);
}

/* Function: lcd_set_cursor_movement
 * 
 * Description:
 *      Shift the display one cell in the specified direction.
 * 
 * Input:
 *      dir: Direction to shift (LEFT/RIGHT)
 *      disp: Automaticly shift display? (true/false)
 * 
 * Instruction 000001(I/D)Sb
 * 
 */
static void lcd_set_cursor_movement(LCD_DIR dir, uint8_t disp)
{
    unsigned char command = LCD_ENTRY_MODE;
    switch (dir) {
    case LEFT:
        command |= MOVE_LEFT;
        break;
    case RIGHT:
        command |= MOVE_RIGHT;
        break;
    default:
        printk(KERN_ERR "HD44780: Invalid cursor direction.");
        return;
    }
    if (disp) {
        command |= MOVE_DISPLAY;
    }
    lcd_instruction(command);
    lcd_info.cursor_direction = dir;
    lcd_info.display_autoshift = disp;
}

/* Function: lcd_display_on
 * 
 * Description:
 *      Turn on the screen itself and set the cursor to the specified mode.
 * 
 * Input:
 *      cursor_mode: specify if the cursor should be on/off/blinking
 *              (CURSOR_ON/CURSOR_OFF/CURSOR_BLINK)
 * 
 */
static void lcd_display_on(int cursor_mode)
{
    /* Display On/off Control */
    /* Instruction 00001DCBb  
     * Set D= 1, or Display on
     * Set C= (0/1), or Cursor (off/on)
     * Set B= (0/1), or Blinking (off/on)
     * 
     */
    unsigned char command = LCD_DISPLAY_CONTROL | DISPLAY_ON;
    switch (cursor_mode) {
    case CURSOR_OFF:
        command |= CURSOR_OFF;
        printk(KERN_DEBUG "HD44780: Cursor Mode = OFF\n");
        break;
    case CURSOR_ON:
        printk(KERN_DEBUG "HD44780: Cursor Mode = ON\n");
        command |= CURSOR_ON;
        break;
    case CURSOR_BLINK:
        printk(KERN_DEBUG "HD44780: Cursor Mode = BLINK\n");
        command |= CURSOR_BLINK;
        break;
    default:
        printk(KERN_ERR "HD44780: Invalid cursor mode.\n");
        return;
    }

    lcd_instruction(command);
    printk(KERN_DEBUG "HD44780: LCD cursor mode set.\n");
    lcd_info.display_state = DISPLAY_ON;
}

/* Function: lcd_display_off
 * 
 * Description:
 *      Turn off the LCD display. Currently called upon module exit
 * 
 */
static void lcd_display_off(void)
{
    /* Display On/off Control */
    /* Instruction 00001DCBb  
     * Set D= 0, or Display off
     * Set C= 0, or Cursor off
     * Set B= 0, or Blinking off
     * 
     */
    lcd_instruction(LCD_DISPLAY_CONTROL | DISPLAY_OFF);
    printk(KERN_DEBUG "HD44780: Display off.\n");
    lcd_info.display_state = DISPLAY_OFF;
}

/* Function: lcd_write_data
 * 
 * Description:
 *      Send a 1-byte ASCII character data to the HD44780 LCD controller.
 * 
 * Input:
 *      data: 1-byte of data to be sent to the LCD controller.
 * 
 */
static void lcd_write_data(char data)
{

    lcd_send(data, lcd_info.bus_width, DATA_MODE);

}

/* Function: lcd_read_chunk
 * 
 * Description:
 *      Reads a chunk of data from the specified address 
 *      in CGRAM or DDRAM to the buffer.
 * 
 * Input:
 *      buffer: Data output buffer
 *      memspace: Which memory space to go to. (DDRAM/CGRAM)
 *      offset: Address at which to start reading.
 *      count: Number of bytes to read.
 *      
 */
static void lcd_read_chunk(char *buffer, MEMORY_SPACE memspace, uint8_t offset,
                           uint8_t count)
{
    int i;
    lcd_set_addr(offset, memspace);
    for (i = 0; i < count; i++) {
        buffer[i] = lcd_read_data();
    }
    return;
}

/* Function: lcd_write_chunk
 * 
 * Description:
 *      Writes a chunk of data from the buffer of specified size to
 *      the specified address in CGRAM or DDRAM.
 * 
 * Input:
 *      buffer: Data input buffer
 *      memspace: Which memory space to go to. (DDRAM/CGRAM)
 *      offset: Address at which to start writing.
 *      count: Number of bytes to write.
 *      
 */
static void lcd_write_chunk(char *buffer, MEMORY_SPACE memspace, uint8_t offset,
                            uint8_t count)
{
    int i;
    lcd_set_addr(offset, memspace);
    for (i = 0; i < count; i++) {
        lcd_write_data(buffer[i]);
    }
    return;
}

/* Function: lcd_initialize
 * 
 * Description:
 *      Initialize the LCD as described on the HD44780 LCD controller datasheet.
 * 
 */
static void lcd_initialize(void)
{
    unsigned char command = 0x00;

    /* Function Set/Wake */
    /* Instruction 001(DL)NF--b  
     * Set DL= 1, or 8-bit mode
     * Can't set N (controller currently in 8-bit mode)
     * Can't set F (controller currently in 8-bit mode)
     * 
     */
    // Use lcd_send rather than lcd_instruction in order to force 8-bit mode
    lcd_send(LCD_FUNCTION_SET | BIT_MODE_8 | LINES_1 | FONT_5_X_8, 8,
             COMMAND_MODE);

    usleep_range(5 * 1000, 6 * 1000);   // wait for more than 4.1 ms

    /* Function Set/Wake */
    /* Instruction 001(DL)NF--b  
     * Set DL= 1, or 8-bit mode
     * Can't set N (controller currently in 8-bit mode)
     * Can't set F (controller currently in 8-bit mode)
     * 
     */
    // Use lcd_send rather than lcd_instruction in order to force 8-bit mode
    lcd_send(LCD_FUNCTION_SET | BIT_MODE_8 | LINES_1 | FONT_5_X_8, 8,
             COMMAND_MODE);

    /* Function Set/Wake */
    /* Instruction 001(DL)NF--b  
     * Set DL= 1, or 8-bit mode
     * Can't set N (controller currently in 8-bit mode)
     * Can't set F (controller currently in 8-bit mode)
     * 
     */
    // Use lcd_send rather than lcd_instruction in order to force 8-bit mode
    lcd_send(LCD_FUNCTION_SET | BIT_MODE_8 | LINES_1 | FONT_5_X_8, 8,
             COMMAND_MODE);

    /* Function Set */
    /* Instruction 001(DL)NF--b  
     * Set DL= 0, or 4-bit mode
     * Can't set N (controller currently in 8-bit mode)
     * Can't set F (controller currently in 8-bit mode)
     * 
     */
    // Use lcd_send rather than lcd_instruction in order to force 8-bit mode
    if (lcd_info.bus_width == 4) {
        lcd_send(LCD_FUNCTION_SET | BIT_MODE_4 | LINES_1 | FONT_5_X_8, 8,
                 COMMAND_MODE);
    }

    /* Function Set */
    /* Instruction 001(DL)NF--b  
     * Set DL= 0/1, or 4-bit mode
     * Set N = 0/1, or 1/2-lines
     * Set F = 0, or 5x8 font
     * 
     */
    command = LCD_FUNCTION_SET | FONT_5_X_8;

    if (lcd_info.bus_width == 8) {
        command |= BIT_MODE_8;
    } else {
        command |= BIT_MODE_4;
    }

    if (lcd_info.rows >= 2) {
        command |= LINES_2;
    } else {
        command |= LINES_1;
    }

    lcd_instruction(command);

    /* Display off */
    lcd_display_off();

    /* Display clear */
    lcd_clear();

    /* Set Cursor Movement */
    lcd_set_cursor_movement(lcd_info.cursor_direction,
                            lcd_info.display_autoshift);

    // Initialization complete, now enabling display
    // Turn on display with cursor off
    lcd_display_on(CURSOR_OFF);

}

/************ SYSFS Attributes ************/
/* Function: show_nothing
 * 
 * Description:
 *      Write only attribute. This is just a stub that returns 0.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_nothing(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
    //Write only attribute. This is just a stub that returns 0.
    return 0;
}

/* Function: store_nothing
 * 
 * Description:
 *      Read only attribute. This is just a stub that returns count.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_nothing(struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count)
{
    //Read only attribute. This is just a stub that returns count.
    return count;
}

/* Function: show_rows
 * 
 * Description:
 *      Return number of rows on the lcd.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_rows(struct device *dev, struct device_attribute *attr,
                         char *buf)
{
    //Print out number of rows.
    return sprintf(buf, "%d\n", lcd_info.rows);
}

static DEVICE_ATTR(rows, S_IRUGO, show_rows, store_nothing);

/* Function: show_columns
 * 
 * Description:
 *      Return number of columns on the lcd.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_columns(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
    //Print out number of columns.
    return sprintf(buf, "%d\n", lcd_info.columns);
}

static DEVICE_ATTR(columns, S_IRUGO, show_columns, store_nothing);

/* Function: show_bus_width
 * 
 * Description:
 *      Return width of the data bus.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_bus_width(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
    //Print out the bus width.
    return sprintf(buf, "%d-bit\n", lcd_info.bus_width);
}

static DEVICE_ATTR(bus_width, S_IRUGO, show_bus_width, store_nothing);

/* Function: store_clear
 * 
 * Description:
 *      Clears screen and returns count
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_clear(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_clear();
    mutex_unlock(&lcd_info.lcd_mutex);
    //This is just a stub that returns count.
    return count;
}

static DEVICE_ATTR(clear, S_IRUGO | S_IWUSR | S_IWGRP, show_nothing,
                   store_clear);

/* Function: store_home
 * 
 * Description:
 *      Homes screen and returns count
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_home(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_home();
    mutex_unlock(&lcd_info.lcd_mutex);
    //This is just a stub that returns count.
    return count;
}

static DEVICE_ATTR(home, S_IRUGO | S_IWUSR | S_IWGRP, show_nothing, store_home);

/* Function: store_cursor_shift
 * 
 * Description:
 *      Shifts cursor left in the given direction and returns count.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_cursor_shift(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "RIGHT", 4) == 0) {
        lcd_shift_cursor(RIGHT);
    } else if (strncasecmp(buf, "LEFT", 4) == 0) {
        lcd_shift_cursor(LEFT);
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid shift direction. (%s)\n",
               buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(cursor_shift, S_IRUGO | S_IWUSR | S_IWGRP, show_nothing,
                   store_cursor_shift);

/* Function: store_display_shift
 * 
 * Description:
 *      Shifts display in the given direction and returns count.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_display_shift(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "RIGHT", 4) == 0) {
        lcd_shift_display(RIGHT);
    } else if (strncasecmp(buf, "LEFT", 4) == 0) {
        lcd_shift_display(LEFT);
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid shift direction. (%s)\n",
               buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(display_shift, S_IRUGO | S_IWUSR | S_IWGRP, show_nothing,
                   store_display_shift);

/* Function: store_initialize
 * 
 * Description:
 *      (Re)Initializes the HD44780 and returns count
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Length of data in buf.
 * 
 * Output:
 *      ssize_t: Size of written data.
 * 
 */
static ssize_t store_initialize(struct device *dev,
                                struct device_attribute *attr, const char *buf,
                                size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_initialize();
    mutex_unlock(&lcd_info.lcd_mutex);
    //This is just a stub that returns count.
    return count;
}

static DEVICE_ATTR(initialize, S_IRUGO | S_IWUSR | S_IWGRP, show_nothing,
                   store_initialize);

/* Function: show_cursor
 * 
 * Description:
 *      Return the current cursor mode.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_cursor(struct device *dev, struct device_attribute *attr,
                           char *buf)
{
    switch (lcd_info.cursor_mode) {
    case CURSOR_ON:
        return sprintf(buf, "ON\n");
    case CURSOR_OFF:
        return sprintf(buf, "OFF\n");
    case CURSOR_BLINK:
        return sprintf(buf, "BLINK\n");
    default:
        return 0;
    }
}

/* Function: store_cursor
 * 
 * Description:
 *      Device attribute implementation of setting the cursor state.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Size to write
 * 
 * Output:
 *      ssize_t: Size of stored data.
 * 
 */
static ssize_t store_cursor(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "BLINK", 5) == 0) {
        lcd_info.cursor_mode = CURSOR_BLINK;
    } else if (strncasecmp(buf, "ON", 2) == 0) {
        lcd_info.cursor_mode = CURSOR_ON;
    } else if (strncasecmp(buf, "OFF", 3) == 0) {
        lcd_info.cursor_mode = CURSOR_OFF;
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid cusor mode. (%s)\n", buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    if (lcd_info.display_state == DISPLAY_ON) {
        lcd_display_on(lcd_info.cursor_mode);
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(cursor, S_IRUGO | S_IWUSR | S_IWGRP, show_cursor,
                   store_cursor);

/* Function: show_display
 * 
 * Description:
 *      Return current display state.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_display(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
    switch (lcd_info.display_state) {
    case DISPLAY_ON:
        return sprintf(buf, "ON\n");
    case DISPLAY_OFF:
        return sprintf(buf, "OFF\n");
    default:
        return 0;
    }
}

/* Function: store_display
 * 
 * Description:
 *      Device attribute implementation of setting the display state.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Size to write
 * 
 * Output:
 *      ssize_t: Size of stored data.
 * 
 */
static ssize_t store_display(struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "ON", 2) == 0) {
        lcd_display_on(lcd_info.cursor_mode);
    } else if (strncasecmp(buf, "OFF", 3) == 0) {
        lcd_display_off();
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid display mode. (%s)\n", buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(display, S_IRUGO | S_IWUSR | S_IWGRP, show_display,
                   store_display);

/* Function: show_cursor_direction
 * 
 * Description:
 *      Return current cursor direction.
 *      RIGHT=Increment
 *      LEFT=Decrement
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_cursor_direction(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    switch (lcd_info.cursor_direction) {
    case LEFT:
        return sprintf(buf, "LEFT\n");
    case RIGHT:
        return sprintf(buf, "RIGHT\n");
    default:
        return 0;
    }
}

/* Function: store_cursor_direction
 * 
 * Description:
 *      Device attribute implementation of setting the cursor direction.
 *      RIGHT=Increment
 *      LEFT=Decrement
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Size to write
 * 
 * Output:
 *      ssize_t: Size of stored data.
 * 
 */
static ssize_t store_cursor_direction(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "RIGHT", 4) == 0) {
        lcd_set_cursor_movement(RIGHT, lcd_info.display_autoshift);
    } else if (strncasecmp(buf, "LEFT", 4) == 0) {
        lcd_set_cursor_movement(LEFT, lcd_info.display_autoshift);
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid cursor direction. (%s)\n",
               buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(cursor_direction, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_cursor_direction, store_cursor_direction);

/* Function: show_display_autoshift
 * 
 * Description:
 *      Return current setting for display_autoshift.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_display_autoshift(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{

    switch (lcd_info.display_autoshift) {
    case true:
        return sprintf(buf, "ON\n");
    case false:
        return sprintf(buf, "OFF\n");
    default:
        return 0;
    }
}

/* Function: store_display_autoshift
 * 
 * Description:
 *      Device attribute implementation of setting the cursor direction.
 *      RIGHT=Increment
 *      LEFT=Decrement
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Size to write
 * 
 * Output:
 *      ssize_t: Size of stored data.
 * 
 */
static ssize_t store_display_autoshift(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    if (strncasecmp(buf, "ON", 2) == 0) {
        lcd_set_cursor_movement(lcd_info.cursor_direction, true);
    } else if (strncasecmp(buf, "OFF", 3) == 0) {
        lcd_set_cursor_movement(lcd_info.cursor_direction, false);
    } else {
        printk(KERN_ERR "HD44780 (sysfs): Invalid autoshift parameter. (%s)\n",
               buf);
        mutex_unlock(&lcd_info.lcd_mutex);
        return -EINVAL;
    }
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

static DEVICE_ATTR(display_autoshift, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_display_autoshift, store_display_autoshift);

/* Function: show_address_counter
 * 
 * Description:
 *      Read and display the address counter in hex.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 * 
 * Output:
 *      ssize_t: Size of shown data.
 * 
 */
static ssize_t show_address_counter(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    //Write only attribute. This is just a stub that returns 0.
    return sprintf(buf, "0x%02x\n", lcd_get_addr());
}

/* Function: store_address_counter
 * 
 * Description:
 *      Device attribute implementation of setting the address counter.
 * 
 * Inputs:
 *      dev: Device pointer
 *      attr: Pointer to device_attribute
 *      buf: Buffer containing the new value.
 *      count: Size to write
 * 
 * Output:
 *      ssize_t: Size of stored data.
 * 
 */
static ssize_t store_address_counter(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    uint8_t addr;
    int ret;
    ret = kstrtou8(buf, 16, &addr);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Invalid address. (%s)\n", buf);
        return ret;
    }
    if (addr > LCD_ADDRESS) {
        printk(KERN_ERR "HD44780 (sysfs): Address too large. (0x%02x)\n", addr);
        return -EINVAL;
    }

    mutex_lock(&lcd_info.lcd_mutex);
    lcd_set_addr(addr, DDRAM);
    mutex_unlock(&lcd_info.lcd_mutex);

    printk(KERN_DEBUG "HD44780 (sysfs): Address counter set to 0x%02x.\n",
           addr);
    return count;
}

static DEVICE_ATTR(address_counter, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_address_counter, store_address_counter);

/* Function: ddram_write
 * 
 * Description:
 *      SYSFS bin_attribute write handler.
 * 
 */
static ssize_t write_ddram(struct file *ddram_file, struct kobject *kobj,
                           struct bin_attribute *attr, char *buffer,
                           loff_t offset, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_write_chunk(buffer, DDRAM, offset, count);
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

/* Function: ddram_read
 * 
 * Description:
 *      SYSFS bin_attribute read handler.
 * 
 */
static ssize_t read_ddram(struct file *ddram_file, struct kobject *kobj,
                          struct bin_attribute *attr, char *buffer,
                          loff_t offset, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_read_chunk(buffer, DDRAM, offset, count);
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

BIN_ATTR(ddram, S_IRUGO | S_IWUSR | S_IWGRP, read_ddram, write_ddram, 128);

/* Function: cgram_write
 * 
 * Description:
 *      SYSFS bin_attribute write handler.
 * 
 */
static ssize_t write_cgram(struct file *cgram_file, struct kobject *kobj,
                           struct bin_attribute *attr, char *buffer,
                           loff_t offset, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_write_chunk(buffer, CGRAM, offset, count);
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

/* Function: cgram_read
 * 
 * Description:
 *      SYSFS bin_attribute read handler.
 * 
 */
static ssize_t read_cgram(struct file *cgram_file, struct kobject *kobj,
                          struct bin_attribute *attr, char *buffer,
                          loff_t offset, size_t count)
{
    mutex_lock(&lcd_info.lcd_mutex);
    lcd_read_chunk(buffer, CGRAM, offset, count);
    mutex_unlock(&lcd_info.lcd_mutex);
    return count;
}

BIN_ATTR(cgram, S_IRUGO | S_IWUSR | S_IWGRP, read_cgram, write_cgram, 64);

/************ SYSFS Setup Functions ************/

/* Function: hd44780_sysfs_setup
 * 
 * Description:
 *      Create SYSFS objects.
 * 
 * Created Objects:
 *      bus_width
 *      rows
 *      columns
 *      initialize
 *      clear
 *      home
 *      display
 *      cursor
 *      cursor_direction
 *      display_autoshift
 *      address_counter
 *      cursor_shift
 *      display_shift
 *      ddram
 *      cgram
 *      class link
 * 
 * Input:
 *      dev: device sysfs files will be linked to.
 */
static int hd44780_sysfs_setup(struct device *dev)
{
    int ret = 0;

    // Setup sysfs properties

    // bus_width
    ret = device_create_file(dev, &dev_attr_bus_width);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create bus_width file.\n");
        return ret;
    }
    // rows
    ret = device_create_file(dev, &dev_attr_rows);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create rows file.\n");
        return ret;
    }
    // columns
    ret = device_create_file(dev, &dev_attr_columns);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create columns file.\n");
        return ret;
    }
    // initialize
    ret = device_create_file(dev, &dev_attr_initialize);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address initialize file.\n");
        return ret;
    }
    // clear
    ret = device_create_file(dev, &dev_attr_clear);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create clear file.\n");
        return ret;
    }
    // home
    ret = device_create_file(dev, &dev_attr_home);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create home file.\n");
        return ret;
    }
    // display
    ret = device_create_file(dev, &dev_attr_display);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create display file.\n");
        return ret;
    }
    // cursor
    ret = device_create_file(dev, &dev_attr_cursor);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create cusor file.\n");
        return ret;
    }
    // cursor_direction
    ret = device_create_file(dev, &dev_attr_cursor_direction);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address cursor_direction file.\n");
        return ret;
    }
    // display_autoshift
    ret = device_create_file(dev, &dev_attr_display_autoshift);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address display_autoshift file.\n");
        return ret;
    }
    // address_counter
    ret = device_create_file(dev, &dev_attr_address_counter);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address counter file.\n");
        return ret;
    }
    // cursor_shift
    ret = device_create_file(dev, &dev_attr_cursor_shift);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address cursor_shift file.\n");
        return ret;
    }
    // display_shift
    ret = device_create_file(dev, &dev_attr_display_shift);
    if (ret) {
        printk(KERN_ERR
               "HD44780 (sysfs): Failed to create address display_shift file.\n");
        return ret;
    }
    // ddram
    ret = device_create_bin_file(dev, &bin_attr_ddram);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create ddram file.\n");
        return ret;
    }
    // cgram
    ret = device_create_bin_file(dev, &bin_attr_cgram);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to create cgram file.\n");
        return ret;
    }
    // class link
    ret = class_compat_create_link(hd44780_class, dev, NULL);
    if (ret) {
        printk(KERN_ERR "HD44780 (sysfs): Failed to link to class.\n");
        return ret;
    }
    return 0;
}

/* Function: hd44780_sysfs_cleanup
 * 
 * Description:
 *      Remove sysfs objects.
 * 
 * Removed Objects:
 *      bus_width
 *      rows
 *      columns
 *      initialize
 *      clear
 *      home
 *      display
 *      cursor
 *      cursor_direction
 *      display_autoshift
 *      address_counter
 *      cursor_shift
 *      display_shift
 *      ddram
 *      cgram
 *      class link
 * 
 * Input:
 *      dev: device sysfs files are linked to.
 */
static void hd44780_sysfs_cleanup(struct device *dev)
{
    // clean up sysfs

    // bus_width
    device_remove_file(dev, &dev_attr_bus_width);

    // rows
    device_remove_file(dev, &dev_attr_rows);

    // columns
    device_remove_file(dev, &dev_attr_columns);

    // initialize
    device_remove_file(dev, &dev_attr_initialize);

    // clear
    device_remove_file(dev, &dev_attr_clear);

    // home
    device_remove_file(dev, &dev_attr_home);

    // display
    device_remove_file(dev, &dev_attr_display);

    // cursor
    device_remove_file(dev, &dev_attr_cursor);

    // cursor_direction
    device_remove_file(dev, &dev_attr_cursor_direction);

    // display_autoshift
    device_remove_file(dev, &dev_attr_display_autoshift);

    // address_counter
    device_remove_file(dev, &dev_attr_address_counter);

    // cursor_shift
    device_remove_file(dev, &dev_attr_cursor_shift);

    // display_shift
    device_remove_file(dev, &dev_attr_display_shift);

    // ddram
    device_remove_bin_file(dev, &bin_attr_ddram);

    // cgram
    device_remove_bin_file(dev, &bin_attr_cgram);

    // class link
    class_compat_remove_link(hd44780_class, dev, NULL);

    return;
}

/************ Platform Driver Setup ************/

/* Struct: hd44780_dt_ids
 * 
 * Description:
 *      A list of comatible LCD devices.
 * 
 */
static const struct platform_device_id hd44780_devtype[] = {
    {.name = "hitachi-hd44780",.driver_data = 0},
    { /* Requiered NULL ENTRY */ }
};

MODULE_DEVICE_TABLE(platform, hd44780_devtype);

/* Struct: lcd_dt_ids
 * 
 * Description:
 *      A list of comatible LCD devices.
 */
static const struct of_device_id hd44780_dt_ids[] = {
    {
     .compatible = "hitachi,hd44780",
     .data = &hd44780_devtype[0]},
    { /* Requiered NULL ENTRY */ }
};

MODULE_DEVICE_TABLE(of, hd44780_dt_ids);

/* Struct: hd44780_driver
 * 
 * Description:
 *      Collection of necesary driver information
 */
static struct platform_driver hd44780_driver = {
    .probe = hd44780_probe,
    .remove = hd44780_remove,
    .driver = {
               .name = "hd44780",
               .of_match_table = hd44780_dt_ids,
               .owner = THIS_MODULE},
};

/* Function: hd44780_probe
 * 
 * Description:
 *      This function is called when the platform driver is probed.
 *      It is responsible for checking the compatability of the
 *      probed device, and performing inital setup.
 * 
 * Resources Allocated:
 *      SYSFS Files (see hd44780_sysfs_setup)
 *      GPIO (varies based on bus width)
 *      pinctrl
 *      mutex
 * 
 * Input:
 *      pdev: pointer to the probed platform_device
 * 
 * Output:
 *      int: 0 on success, other on error
 * 
 */
static int hd44780_probe(struct platform_device *pdev)
{
    const void *value;
    int ret;
    const char *mode_string;
    const struct of_device_id *of_id =
        of_match_device(hd44780_dt_ids, &pdev->dev);

    lcd_info.pdev = pdev;
    lcd_info.of_node = pdev->dev.of_node;

    if (of_id) {
        printk(KERN_DEBUG
               "HD44780: Compatible device probed. Loading information from the device tree.\n");

        value = of_get_property(lcd_info.of_node, "rows", NULL);
        if (value) {
            lcd_info.rows = be32_to_cpup(value);
            printk(KERN_INFO "HD44780: Found LCD height. (%d)\n",
                   lcd_info.rows);
        } else {
            printk(KERN_ERR "HD44780: 'rows' property missing.\n");
            goto fail;
        }

        value = of_get_property(lcd_info.of_node, "columns", NULL);
        if (value) {
            lcd_info.columns = be32_to_cpup(value);
            printk(KERN_INFO "HD44780: Found LCD width. (%d)\n",
                   lcd_info.columns);
        } else {
            printk(KERN_ERR "HD44780: 'columns' property missing.\n");
            goto fail;
        }

        ret = of_property_read_string(lcd_info.of_node, "mode", &mode_string);
        if (ret == 0) {

            if (strncasecmp(mode_string, "4-bit", 5) == 0) {
                lcd_info.bus_width = 4;
            } else if (strncasecmp(mode_string, "8-bit", 5) == 0) {
                lcd_info.bus_width = 8;
            } else {
                printk(KERN_ERR "HD44780: Invalid 'mode' property. (%s)\n",
                       mode_string);
                goto fail;
            }
            printk(KERN_INFO "HD44780: Found LCD bus width. (%s)\n",
                   mode_string);
        } else {
            printk(KERN_ERR "HD44780: 'mode' property missing.\n");
            goto fail;
        }

        lcd_info.rs_gpio = of_get_named_gpio(lcd_info.of_node, "rs-gpio", 0);
        lcd_info.rw_gpio = of_get_named_gpio(lcd_info.of_node, "rw-gpio", 0);
        lcd_info.e_gpio = of_get_named_gpio(lcd_info.of_node, "e-gpio", 0);

        if (IS_ERR((void *)lcd_info.rs_gpio) || IS_ERR((void *)lcd_info.rw_gpio)
            || IS_ERR((void *)lcd_info.e_gpio)) {
            printk(KERN_ERR "HD44780: Failed to claim GPIO.\n");
            goto fail;
        }

        if (lcd_info.bus_width == 8) {
            lcd_info.db0_gpio =
                of_get_named_gpio(lcd_info.of_node, "db0-gpio", 0);
            lcd_info.db1_gpio =
                of_get_named_gpio(lcd_info.of_node, "db1-gpio", 0);
            lcd_info.db2_gpio =
                of_get_named_gpio(lcd_info.of_node, "db2-gpio", 0);
            lcd_info.db3_gpio =
                of_get_named_gpio(lcd_info.of_node, "db3-gpio", 0);

            if (IS_ERR((void *)lcd_info.db0_gpio)
                || IS_ERR((void *)lcd_info.db1_gpio)
                || IS_ERR((void *)lcd_info.db2_gpio)
                || IS_ERR((void *)lcd_info.db3_gpio)) {
                printk(KERN_ERR "HD44780: Failed to claim GPIO.\n");
                goto fail;
            }
        } else {
            lcd_info.db0_gpio = 0;
            lcd_info.db1_gpio = 0;
            lcd_info.db2_gpio = 0;
            lcd_info.db3_gpio = 0;
        }

        lcd_info.db4_gpio = of_get_named_gpio(lcd_info.of_node, "db4-gpio", 0);
        lcd_info.db5_gpio = of_get_named_gpio(lcd_info.of_node, "db5-gpio", 0);
        lcd_info.db6_gpio = of_get_named_gpio(lcd_info.of_node, "db6-gpio", 0);
        lcd_info.db7_gpio = of_get_named_gpio(lcd_info.of_node, "db7-gpio", 0);

        if (IS_ERR((void *)lcd_info.db4_gpio)
            || IS_ERR((void *)lcd_info.db5_gpio)
            || IS_ERR((void *)lcd_info.db6_gpio)
            || IS_ERR((void *)lcd_info.db7_gpio)) {
            printk(KERN_ERR "HD44780: Failed to claim GPIO.\n");
            goto fail;
        }

        if (lcd_info.bus_width == 8) {
            printk(KERN_INFO
                   "HD44780: RS: %d RW: %d E: %d DB0: %d DB1: %d DB2: %d DB3: %d DB4: %d DB5: %d DB6: %d DB7: %d\n",
                   lcd_info.rs_gpio, lcd_info.rw_gpio, lcd_info.e_gpio,
                   lcd_info.db0_gpio, lcd_info.db1_gpio, lcd_info.db2_gpio,
                   lcd_info.db3_gpio, lcd_info.db4_gpio, lcd_info.db5_gpio,
                   lcd_info.db6_gpio, lcd_info.db7_gpio);
        } else {
            printk(KERN_INFO
                   "HD44780: RS: %d RW: %d E: %d DB4: %d DB5: %d DB6: %d DB7: %d\n",
                   lcd_info.rs_gpio, lcd_info.rw_gpio, lcd_info.e_gpio,
                   lcd_info.db4_gpio, lcd_info.db5_gpio, lcd_info.db6_gpio,
                   lcd_info.db7_gpio);
        }

        lcd_info.p = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR((void *)lcd_info.p)) {
            printk(KERN_ERR "HD44780: Failed to get pinctrl.\n");
            goto fail_pinctrl;
        }

        lcd_info.write_state = pinctrl_lookup_state(lcd_info.p, "write");
        if (IS_ERR((void *)lcd_info.p)) {
            printk(KERN_ERR
                   "HD44780: Failed to get pinctrl state \"write\".\n");
            goto fail_pinctrl;
        }

        lcd_info.read_state = pinctrl_lookup_state(lcd_info.p, "read");
        if (IS_ERR((void *)lcd_info.p)) {
            printk(KERN_ERR "HD44780: Failed to get pinctrl state \"read\".\n");
            goto fail_pinctrl;
        }
        // Initialize MUTEX lock
        mutex_init(&lcd_info.lcd_mutex);

        // Setup SYSFS files
        ret = hd44780_sysfs_setup(&pdev->dev);
        if (ret) {
            printk(KERN_ERR "HD44780: Failed to setup SYSFS files.\n");
            goto fail_sysfs;
        }
        // initialize LCD
        mutex_lock(&lcd_info.lcd_mutex);
        lcd_initialize();
        mutex_unlock(&lcd_info.lcd_mutex);

        printk(KERN_DEBUG "HD44780: Probe Complete.\n");

    } else {
        printk(KERN_ERR "HD44780: Incompatible device probed.\n");
        goto fail;
    }

    return 0;

 fail_sysfs:
    // clean up sysfs
    hd44780_sysfs_cleanup(&pdev->dev);
 fail_pinctrl:
    // Free pinctrl
    devm_pinctrl_put(lcd_info.p);
 fail:
    printk(KERN_ERR "HD44780: Probe error.\n");
    return -EINVAL;

}

/* Function: hd44780_remove
 * 
 * Description:
 *      This function is called when the bound platform device is removed,
 *      or the platform driver is unregistered. All resources allocated by
 *      hd44780_probe are cleaned up.
 * 
 * Resources Cleaned:
 *      SYSFS Files (see hd44780_sysfs_cleanup)
 *      GPIO (see lcd_pin_release_all)
 *      pinctrl
 *      mutex
 * 
 * Input:
 *      pdev: pointer to the removed platform_device
 * 
 */
static int hd44780_remove(struct platform_device *pdev)
{
    const struct of_device_id *of_id =
        of_match_device(hd44780_dt_ids, &pdev->dev);

    printk(KERN_DEBUG "HD44780: Remove called.\n");

    if (of_id) {
        // Clean up sysfs
        hd44780_sysfs_cleanup(&pdev->dev);

        // Clear LCD display
        lcd_clear();

        // Curn off LCD display
        lcd_display_off();

        // Releasse GPIO pins
        lcd_pin_release_all();

        // Free pinctrl
        devm_pinctrl_put(lcd_info.p);

        // Destroy mutex lock
        mutex_destroy(&lcd_info.lcd_mutex);

        printk(KERN_INFO "HD44780: Compatible device removed.\n");
    } else {
        printk(KERN_ERR "HD44780: Incompatible device removed.\n");
    }

    return 0;
}

/************* Module Operation Functions ****************/

/* Function: hd44780_init
 * 
 * Description:
 *      Initialize the device module. Registered resources are
 *      unregistered by hd44780_exit.
 * 
 * Registered Resources:
 *      platform driver
 *      class compat
 * 
 */
static int __init hd44780_init(void)
{
    int r = 0;

    printk(KERN_DEBUG "HD44780: Init called.\n");

    // register class compat
    hd44780_class = class_compat_register(CLASS_NAME);
    if (IS_ERR((void *)hd44780_class)) {
        printk(KERN_ERR "HD44780: Failed to create class structure.\n");
        return PTR_ERR(hd44780_class);
    }

    printk(KERN_DEBUG "HD44780: Registering platform driver.\n");

    // register platform driver
    r = platform_driver_register(&hd44780_driver);
    if (r < 0) {
        class_compat_unregister(hd44780_class);
        printk(KERN_ERR "HD44780: Failed to register platform driver.\n");
        return r;
    }

    printk(KERN_DEBUG "HD44780: Init complete.\n");

    return 0;
}

/* Function: hd44780_exit
 * 
 * Description:
 *      Unregister resources registered in hd44780_init
 * 
 */
static void __exit hd44780_exit(void)
{
    printk(KERN_DEBUG "HD44780: Exit called.\n");

    // unregister platform driver
    platform_driver_unregister(&hd44780_driver);

    // unregister class compat
    class_compat_unregister(hd44780_class);
    printk(KERN_INFO "HD44780: Driver Exited.\n");
}

/************* Module Setup Macros ********************/
module_init(hd44780_init);
module_exit(hd44780_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:hd44780");
