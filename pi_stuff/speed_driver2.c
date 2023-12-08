#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>

/* Button and LED GPIO Descriptors */
static struct gpio_desc *led_gpio;
static struct gpio_desc *button_gpio;

unsigned long t1 = 0;
unsigned long t2;
unsigned long elapsedTime;

// Adds elapsed_ms to a parameters file under sys/modules/parameters
module_param(elapsedTime, long, S_IRUGO);

unsigned long get_time_us(void) {
    return ktime_to_us(ktime_get_real());
}

/*
 * Adopted from Lines 11-12 from https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c#L11
 * "variable contains pin number o interrupt controller to which GPIO 17 is mapped to" in our case 
*/
static unsigned int irq_number;

/*
 * Funcion adopted from Lines 15-20 from https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c#L15
 * "@brief Interrupt service routine is called, when interrupt is triggered"
*/
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    
    t2 = get_time_us();
    elapsedTime = t2 - t1;
    t1 = t2;

    // Print elapsed time in microseconds as an integer
    printk(KERN_INFO "Speed Encoder: %lu us.\n", elapsedTime);
    
    // Print a message to kernel stating that the button has been pressed.
    printk(KERN_INFO "The Button HAS been PRESSED!\n");
    return (irq_handler_t) IRQ_HANDLED;
}

/*
 * This functions loads modules and sets up ISR for button by
 * accessing led and button gpio descriptors. It prints outputs
 * to the kernel to signal the module successfully loaded.
 *
 * This function has code adopted from Linux Driver Tutorial repo
 * on Github at the mentioned parts within the function.
 */
static int led_probe(struct platform_device *pdev)
{
    // Signal to kernel log that module is loaded successfully.
    printk(KERN_INFO "MODULE LOADED.\n");

    t1 = get_time_us();

    //Get the Button DPIO Device Managed Descriptor.
    button_gpio = devm_gpiod_get(&pdev->dev, "button", GPIOD_IN);

    gpiod_set_debounce(button_gpio, 1000000);

    /*
    * Adopted from Lines 42-48 from https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c#42
    * @brief Interrupt service routine is called, when interrupt is triggered
    */
    irq_number = gpiod_to_irq(button_gpio);
    if (request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_FALLING, "noah,gpio", NULL) != 0) {
                printk("Error!\n Can not request interrupt nr.: %d\n", irq_number);
        return -1;
    }

    // Successful return.
    return 0;
}

/* This function cleans up the device driver by freeing associated IRQ.
 * It also logs the removal to the kernel and exits successfully.
 */
static int led_remove(struct platform_device *pdev)
{
    // Free Associated IRQ to clean up driver.
    free_irq(irq_number, NULL);

    // Log to the kernel of removal.
    printk(KERN_INFO "The LED has been removed!\n");

    // Exit successfully.
    return 0;
}

// Matches compatible gpio device.
static struct of_device_id matchy_match[] = {
    { .compatible = "noah,gpio" }, // Change as needed
    {/* leave alone - keep this here (end node) */},
};

// Responsible for running the functions to prove and remove.
static struct platform_driver adam_driver = {
    .probe   = led_probe,
    .remove  = led_remove,
    .driver  = {
        .name           = "The Rock: this name doesn't even matter",
        .owner          = THIS_MODULE,
        .of_match_table = matchy_match,
    },
};

// Runs the probe and remove of driver.
module_platform_driver(adam_driver);
MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");



// #include <linux/module.h>
// #include <linux/of_device.h>
// #include <linux/kernel.h>
// #include <linux/gpio/consumer.h>
// #include <linux/platform_device.h>
// #include <linux/interrupt.h>
// #include <linux/of.h>
// #include <linux/ktime.h>

// static int irq_number;
// static struct gpio_desc *gpio_speed;
// unsigned long t1 = 0;
// unsigned long t2;

// unsigned long get_time_us(void) {
//     return ktime_to_us(ktime_get_real());
// }


// static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct
// pt_regs *regs) {
//     unsigned long elapsedTime;
//     t2 = get_time_us();
//     elapsedTime = t2 - t1;
//     t1 = t2;

//     // Print elapsed time in microseconds as an integer
//     printk(KERN_INFO "Speed Encoder: %lu us.\n", elapsedTime);

//     return (irq_handler_t) IRQ_HANDLED;
// }

// static int led_probe(struct platform_device *pdev) {
//     printk(KERN_INFO "LED module inserted :)\n");
//     t1 = get_time_us();
//     gpio_speed = devm_gpiod_get(&pdev->dev, "speed", GPIOD_IN);
    
//     gpiod_set_debounce(gpio_speed, 1000000);
//     irq_number = gpiod_to_irq(gpio_speed);
//     if (request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "encoder-gpio,gpio", &pdev->dev) != 0) {
//         printk("Failed to request IRQ.\n");
//         return -1;
//     }

//     printk("Successfully requested IRQ.\n");

//     return 0;
// }

// static int led_remove(struct platform_device *pdev) {
//     free_irq(irq, &pdev->dev);
//     printk(KERN_INFO "Free irq\n");
//     return 0;
// }

// static struct of_device_id matchy_match[] = {
//     {.compatible = "encoder-gpio,gpio",},
//     {/* sentinel */},
// };

// static struct platform_driver adam_driver = {
//     .probe = led_probe,
//     .remove = led_remove,
//     .driver = {
//         .name = "The Rock: this name doesn't even matter",
//         .owner = THIS_MODULE,
//         .of_match_table = matchy_match,
//     },
// };

// module_platform_driver(adam_driver);
// MODULE_DESCRIPTION("424's finest");
// MODULE_AUTHOR("GOAT");
// MODULE_LICENSE("GPL v2");
// MODULE_ALIAS("platform:adam_driver");
