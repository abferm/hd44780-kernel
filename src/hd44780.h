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

#ifndef HD44780_H_
#define HD44780_H_

/********* Linux driver Constants ************/

#define DRIVER_AUTHOR           "Alex Ferm <aferm@petropower.com>"
#define DRIVER_DESC             "A bit-banged HD44780 LCD controller driver"
#define CLASS_NAME              "hd44780"

#define MAX_BUSY_WAIT           1000    // maximum number of times to check the busy flag

#define MAX_BUF_LENGTH          50      // maximum length of a buffer to copy between user space and kernel space

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/********* LCD OP CODES **********/

#define LCD_CLEAR               0x01
#define LCD_HOME                (0x01 << 1)
#define LCD_ENTRY_MODE          (0x01 << 2)
#define LCD_DISPLAY_CONTROL     (0x01 << 3)
#define LCD_SHIFT               (0x01 << 4)
#define LCD_FUNCTION_SET        (0x01 << 5)
#define LCD_SET_CGRAM_ADDR      (0x01 << 6)
#define LCD_SET_DDRAM_ADDR      (0x01 << 7)

/********* OP CODE OPTIONS **********/
// LCD_ENTRY_MODE
#define MOVE_LEFT               (LEFT << 1)
#define MOVE_RIGHT              (RIGHT << 1)
#define MOVE_DISPLAY            (0x01 << 0)

// LCD_DISPLAY_CONTROL
#define DISPLAY_OFF             (0x00 << 2)
#define DISPLAY_ON              (0x01 << 2)
#define CURSOR_OFF              (0x00 << 1)
#define CURSOR_ON               (0x01 << 1)
#define CURSOR_BLINK            (0x01 << 0)

// LCD_SHIFT
#define SHIFT_CURSOR            (0x00 << 3)
#define SHIFT_DISPLAY           (0x01 << 3)
#define SHIFT_LEFT              (LEFT << 2)
#define SHIFT_RIGHT             (RIGHT << 2)

// LCD_FUNCTION_SET
#define BIT_MODE_4              (0x00 << 4)
#define BIT_MODE_8              (0x01 << 4)
#define LINES_1                 (0x00 << 3)
#define LINES_2                 (0x01 << 3)
#define FONT_5_X_8              (0x00 << 2)
#define FONT_5_X_11             (0x01 << 2)

/********* BIT MASKS **********/

#define LCD_ADDRESS             0X7F
#define LCD_BUSY_FLAG           0x80

/********* Enumerations ***********/
// GPIO Pin Directions
typedef enum pin_dir {
    INPUT_PIN = 0,
    OUTPUT_PIN = 1
} PIN_DIRECTION;

// LCD Modes
typedef enum lcd_modes {
    LCD_READ = 0,
    LCD_WRITE = 1,
    LCD_MODE_UNSET = -1
} LCD_MODE;

// LCD RS Modes
typedef enum lcd_rs_modes {
    COMMAND_MODE = 0,
    DATA_MODE = 1
} LCD_RS_MODE;

// LCD Memory Spaces
typedef enum lcd_memspaces {
    DDRAM = 0,
    CGRAM = 1
} MEMORY_SPACE;

// LCD Directions
typedef enum lcd_dirs {
    LEFT = 0,
    RIGHT = 1
} LCD_DIR;

/************* LCD Structure *************/
typedef struct {
    // Platform device
    struct platform_device *pdev;
    // Device tree node
    struct device_node *of_node;

    // Pinctrl and states
    struct pinctrl *p;
    struct pinctrl_state *write_state, *read_state;

    // Mutex lock
    struct mutex lcd_mutex;

    // Tracking of LCD state (READ/WRITE/UNSET)
    // Should be initialized as LCD_MODE_UNSET
    LCD_MODE current_mode;

    // Cursor mode CURSOR_(ON/OFF/BLINK)
    int cursor_mode;

    // Display state DISPLAY_(ON/OFF)
    int display_state;

    // Cursor direction (LEFT/RIGHT)
    LCD_DIR cursor_direction;

    // Automatically shift display? (true/false)
    int display_autoshift;

    // Are the gpio pins currently claimed? (true/false)
    int has_gpio;

    // LCD dimensions
    int rows;
    int columns;

    // Number of wires for the data bus (4/8)
    int bus_width;

    // Control line gpio numbers
    int rs_gpio;
    int rw_gpio;
    int e_gpio;

    // Data bus gpio numbers
    int db0_gpio;
    int db1_gpio;
    int db2_gpio;
    int db3_gpio;
    int db4_gpio;
    int db5_gpio;
    int db6_gpio;
    int db7_gpio;

} LCD_INFO;

/********* Function Prototypes **********/

// Pin Setup/Release Functions
static int lcd_pin_setup(unsigned int pin_number, PIN_DIRECTION gpio_direction);

static void lcd_pin_release_all(void);

static int lcd_set_rw(LCD_MODE mode);

// LCD Write-Mode Functions
static void lcd_send(unsigned char data, int bus_width, LCD_RS_MODE data_mode);

static void lcd_instruction(char command);
static void lcd_write_data(char data);

static void lcd_initialize(void);
static void lcd_write_chunk(char *buffer, MEMORY_SPACE memspace, uint8_t offset,
                            uint8_t count);

static void lcd_set_cursor_movement(LCD_DIR dir, uint8_t disp);
static void lcd_set_addr(unsigned char address, MEMORY_SPACE memspace);
static void lcd_clear(void);

static void lcd_home(void);

static void lcd_shift_cursor(LCD_DIR dir);
static void lcd_shift_display(LCD_DIR dir);

static void lcd_display_off(void);
static void lcd_display_on(int cursor_mode);

// LCD Read-Mode Functions
unsigned char lcd_read(LCD_RS_MODE data_mode);

unsigned char lcd_get_addr(void);

int lcd_check_busy(void);
void lcd_busy_wait(void);

unsigned char lcd_read_data(void);
static void lcd_read_chunk(char *buffer, MEMORY_SPACE memspace, uint8_t offset,
                           uint8_t count);

// Platform Driver Operation Functions
static int hd44780_probe(struct platform_device *pdev);
static int hd44780_remove(struct platform_device *pdev);

// SYSFS Setup Functions
static int hd44780_sysfs_setup(struct device *dev);
static void hd44780_sysfs_cleanup(struct device *dev);

// Module Operation Functions
static int __init hd44780_init(void);
static void __exit hd44780_exit(void);

#endif
