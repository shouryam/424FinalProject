#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>

//Inturupt handler number
int irq_number;

// //Refers to LED pin P9-27
// struct gpio_desc *led;

//refers to button pin P8_14
struct gpio_desc *button;

ktime_t curr_time, previous_time;
int diff;
module_param(diff, int, S_IRUGO);

/**
 * Interrupt service routine is called, when interrupt is triggered
 * Got this code from https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c 
 */
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	printk("Speed encoder triggered.\n");
	//gpiod_set_value(led, (gpiod_get_value(led) + 1) %2);
	//clock_t new_time = clock();
	// if last_time == NULL {
	// 	last_time = new_time;
	// } else {
	curr_time = ktime_get();
        int temp = ktime_to_ns(curr_time - previous_time) / 1000000;
        if(temp > 1) { diff = temp; }
        printk("Difference between triggers: %d\n", diff);
        previous_time = curr_time;
	// 	last_time = current;
    //     int millseconds = diff * 1000/CLOCKS_PER_SEC;
	// 	printk("Milliseconds: %d", millseconds);
	// 	if (millseconds > 1){
	// 		FILE *fptr;
	// 		fptr = fopen("speed.txt", "w");
	// 		fprintf(fptr, "%d", millseconds);
	// 		fclose(fptr);
	// 	}
//	je = jiffies;
  //      printk("This is the previous time (js): %ld\n", js);
    //    printk("This is the current time (je): %ld\n", je);
//	diffj = js - je;
//	unsigned int check;
//	check  = jiffies_to_usecs(diffj);
//	if (check > 1000){
//		diff = check;
//	}
//	js = je;
//	printk("Difference in rotations is now: %d\n", diff);
	return (irq_handler_t) IRQ_HANDLED; 
}

// probe function, takes in the platform device that allows us to get the references to the pins
static int led_probe(struct platform_device *pdev)
{
	printk("gpiod_driver has been inserted.\n");
    //Get the pins based on what we called them in the device tree file:  "name"-gpios
	//led = devm_gpiod_get(&pdev->dev, "led", GPIOD_OUT_LOW);
	button = devm_gpiod_get(&pdev->dev, "userbutton", GPIOD_IN);

    //Set waiting period before next input
	gpiod_set_debounce(button, 1000000);

    //Pair the button pin with an irq number
	irq_number = gpiod_to_irq(button);

    //Pair the number with the function that handles it
	if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0){
		printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
		free_irq(irq_number, NULL);
		return -1;
	}
	return 0;
}

// remove function ,takes in the platform device that allows us to get the references to the pins
static int led_remove(struct platform_device *pdev)
{
	printk("Custom gpiod_driver module has been removed and irq has been freed\n");
	irq_number = gpiod_to_irq(button);
	//TBH not sure what goes in the second argument
    free_irq(irq_number, NULL);
	return 0;
}

static struct of_device_id matchy_match[] = {
	{.compatible = "gpiod_driver"},
	{/* leave alone - keep this here (end node) */},
};

// platform driver object
static struct platform_driver adam_driver = {
	.probe  = led_probe,
	.remove  = led_remove,
	.driver  = {
       		.name  = "The Rock: this name doesn't even matter",
       		.owner = THIS_MODULE,
       		.of_match_table = matchy_match,
	},
};

module_platform_driver(adam_driver);
MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
