#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>

#include "rgb_interface.h"
#include "rgb_hw.h"
#include "myLog.h"

static struct class mClass;
/*
    ssize_t (*show)(struct class *class, struct class_attribute *attr,
            char *buf);
    ssize_t (*store)(struct class *class, struct class_attribute *attr,
            const char *buf, size_t count);
*/
int if_create() {
	int res = 0;
	Fin();
	//mClass = class_create(THIS_MODULE, "illumination");
	res = class_register(&mClass);
	if (IS_ERR_VALUE(res)) {
		Fout("class_register failed: %d", res);
		return res;
	}
	/*if (IS_ERR_OR_NULL(mClass) ) {
		Fout("create_class failed: %d", PTR_ERR(mClass) );
		return PTR_ERR(mClass);
	}*/
	 /*
	res = class_create_file(mClass, mClassAttrs);
	if (IS_ERR_VALUE(res)) {
		Fout("class_create_file failed: %d", res);
		return res;
	}
	*/


	Fout("Success");
	return 0;
}

static struct class_attribute mClassAttrs[];
static struct class mClass = {
	.name = "illumination",
	.owner = THIS_MODULE,
	.class_attrs = mClassAttrs,

};

static ssize_t showAttr(
	struct class *,
	struct class_attribute *,
	char *);
static ssize_t storeAttr(
	struct class*,
	struct class_attribute *,
	const char *buf, size_t count);

static struct class_attribute mClassAttrs[] = {
	__ATTR(0,0644, showAttr, storeAttr),
	__ATTR(1,0644, showAttr, storeAttr),
	__ATTR(2,0644, showAttr, storeAttr),
	__ATTR(3,0644, showAttr, storeAttr),
	__ATTR(4,0644, showAttr, storeAttr),
	__ATTR(5,0644, showAttr, storeAttr),
	__ATTR(6,0644, showAttr, storeAttr),
	__ATTR(7,0644, showAttr, storeAttr),
	__ATTR(8,0644, showAttr, storeAttr),
	__ATTR_NULL,
};

static ssize_t showAttr(
		struct class *c,
		struct class_attribute *a,
		char *buf) {
	char ch = a->attr.name[0];
	int res = 0;
	unsigned int reg;
	Fin();
	if (ch<'0' || ch>'8') {
		Ftr("Register out of range, %c", ch);
		return -EINVAL;
	}
	reg = ch-'0';
	res = hw_getReg(reg);
	res = res & 0x000000ff;
	snprintf(buf, PAGE_SIZE, "0x%X", res);
	Fout();
	return strlen(buf);
}
static ssize_t storeAttr(
		struct class* c,
		struct class_attribute *a,
		const char *buf, size_t count) {
	char ch = a->attr.name[0];
	unsigned int reg;
	long ldata;
	int res = -1;
	Fin("name:%c, buf:%s", ch, buf);
	if (ch<'0' || ch>'8') {
		Fout("reg: 0~8 only");
		return -EINVAL;
	}
	if (strlen(buf)==0) {
		Fout("no buffered data");
		return -EINVAL;
	}
	if (buf[0] == '0') {
		Ftr();
		if (buf[1]=='x' || buf[1]=='X')
			res = kstrtol(buf+2,16,&ldata);
		if ('0'<=buf[1] && buf[1]<='8')
			res = kstrtol(buf+1,8, &ldata);
		if (buf[1]==0x0a || buf[1]==0x0d ||buf[1]==0)
			res = kstrtol(buf,8, &ldata);
	}
	else {
		res = kstrtol(buf, 10, &ldata);
	}
	Ftr("data parsed: %ld", ldata);
	if (res!=0) {
		Fout("Parsing buffer failed, res:%d", res);
		return res;
	}
	if (ldata<-1 || ldata>0x00ff) {
		Fout("Unacceptable value: %d", ldata);
		return -EINVAL;
	}
	
	reg =ch-'0';
	Ftr();
	hw_sendData(reg, ldata);
	Fout("ok");
	return count;
}


void if_release() {
	Fin();
	if (!IS_ERR(&mClass))
		class_destroy(&mClass);
	Fout();
	return;

}
