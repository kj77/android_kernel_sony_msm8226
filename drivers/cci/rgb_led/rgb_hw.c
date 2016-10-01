#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "rgb_hw.h"
#include "myLog.h"

static struct {
	int CTRL;
	unsigned int reg[9];
}mHW;

static void _dumpReg(void) {
    int i = 0;
    Ftr();
    for( ; i < sizeof(mHW.reg)/sizeof(mHW.reg[0]); i++) {
        printk("%d ",mHW.reg[i]);
    }
    Ftr();
}

int hw_getReg(const int REG){
	Fin();
	if (REG<0 || REG>8) {
		Ftr("No such reg:%d", (unsigned int)REG);
		return -1;
	}
	Fout("reg:%d has %X", REG, mHW.reg[REG]);
	return mHW.reg[REG];
}



static DEFINE_SPINLOCK(my_lock);

/* Init the rgb chip.
Actually, I do not need any lock her!
Because there are only least delay time, no delay limitation.
*/
int hw_requestGPIO(const rgb_chip_layout *layout) {
	int res = 0;
	Fin("CTRL:%d", layout->gpio_CTRL);
	mHW.CTRL = layout->gpio_CTRL;
	memset(mHW.reg,0, sizeof(mHW.reg));

    res = gpio_request(mHW.CTRL, "rgb_ctrl");
    if (res<0) {
        Fout("Error 4");
        return -1;
    }

	Fout("Success");
	return 1;
}

#if 1
#define SEND_LOW() do {\
	gpio_set_value(mHW.CTRL,0); \
		udelay(40);\
	gpio_set_value(mHW.CTRL,1); \
		udelay(10);\
} while(0)

#define SEND_HIGH() do {\
    gpio_set_value(mHW.CTRL,0); \
		udelay(10);\
    gpio_set_value(mHW.CTRL,1); \
		udelay(40);\
} while(0)
#endif

int hw_sendData(
		const int  REG,
		const unsigned char data_) {
	unsigned long int_flags;

	struct timeval tv1,tv2;
	unsigned char data = data_;

	Fin();
	if (REG<0 || REG>8) {
		Ftr();
		return -1;
	}

	Ftr(" echo %d > %d", data, REG);


	if (REG == 0) {
		// 1110 0111
		data &= 0xe7;
		// To disable unstable flags, vendor suggests it.
	}

	do_gettimeofday(&tv1);
	spin_lock_irqsave(&my_lock, int_flags);
	mHW.reg[REG] = data;

	gpio_set_value(mHW.CTRL, 1);
	udelay(20); // TDS: data start. typical 10 us
	// fill the address.
	if (REG&0x08) SEND_HIGH(); else SEND_LOW();
	if (REG&0x04) SEND_HIGH(); else SEND_LOW();
	if (REG&0x02) SEND_HIGH(); else SEND_LOW();
	if (REG&0x01) SEND_HIGH(); else SEND_LOW();
	// fill the data.
	if (data&0x80) SEND_HIGH(); else SEND_LOW();
	if (data&0x40) SEND_HIGH(); else SEND_LOW();
	if (data&0x20) SEND_HIGH(); else SEND_LOW();
	if (data&0x10) SEND_HIGH(); else SEND_LOW();
    if (data&0x08) SEND_HIGH(); else SEND_LOW();
    if (data&0x04) SEND_HIGH(); else SEND_LOW();
    if (data&0x02) SEND_HIGH(); else SEND_LOW();
    if (data&0x01) SEND_HIGH(); else SEND_LOW();
	// fill EOD
	gpio_set_value(mHW.CTRL,0);
	udelay(8); //typical val: 2 us
	gpio_set_value(mHW.CTRL,1);
	udelay(400); //typical val: 350 us
	spin_unlock_irqrestore(&my_lock, int_flags);
	do_gettimeofday(&tv2);
	
	Fout("at %d, tv: (%d)us", REG, tv2.tv_usec - tv1.tv_usec);
	return 1;
}


int hw_wakeup(void) {
	unsigned long int_flags;
    struct timeval tv1,tv2;
	//long artv[3];
	Fin();
	do_gettimeofday(&tv1);

    spin_lock_irqsave(&my_lock, int_flags);
	
	udelay(200);
	gpio_set_value(mHW.CTRL, 1);
	udelay(100);
	
    gpio_set_value(mHW.CTRL, 0); //min: 10 us
	udelay(15);

    gpio_set_value(mHW.CTRL, 1); //min: 400 us, max:600 us
	udelay(450);

	spin_unlock_irqrestore(&my_lock, int_flags);
	do_gettimeofday(&tv2);
	Fout("tv:%d", tv2.tv_usec - tv1.tv_usec);
	return 0;
}

int hw_trySuspend(void) {
    _dumpReg();

	if (0) {
		unsigned *p = &mHW.reg[0];
	    if (p[6]!=0 || p[7]!=0 || p[8]!=0)
    	    return -EBUSY;
	    if (p[4]!=0)
    	    return -EBUSY;
	}
    Ftr("Entery suspend mode.");
    gpio_free(mHW.CTRL);

    return 0;
}

/* No need to implement state machine handling suspend/resume mode.
** By check the gpio#0 state, if gpio#0 can not be re-request in resume procedure,
**   the suspend() does not free gpio#0 yet.
*/
int hw_resume(void) {
    int res = 0;
    res = gpio_request(mHW.CTRL, "rgb_ctrl");
    if (res<0) {
        Ftr("Can not request CTRL pin.");
        return res;
    }
    hw_wakeup();
    return 0;
}


int hw_freeGPIO(void) {
    Ftr();
    gpio_free(mHW.CTRL);
	return 0;
}
