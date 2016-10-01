#ifndef _HW_RGB_H_
#define _HW_RGB_H_

/* Describe the RGB chip layout,
	Maybe will be change to fit qcom's design:
	 1. Using platform_device + platform_data
	 2.	Using device tree
*/
typedef struct {
	int gpio_CTRL;
}rgb_chip_layout;

int hw_requestGPIO(const rgb_chip_layout *);
int hw_freeGPIO(void);

int hw_wakeup(void);
int hw_sendData(
	const int REGISTER,  // addr: 0~6
	const unsigned char data);

int hw_getReg(const int REGISTER);

int hw_trySuspend(void);
int hw_resume(void);


#endif // end of file.
