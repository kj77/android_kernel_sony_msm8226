/*===========================================================================
File: kernel/include/linux/suspend_info.h
Description: Central suspend/resume debug mechanism for peripheral devices (gpio/regulator/wake-up) 

Revision History:
when       	who				what, where, why
--------   	---				---------------------------------------------------------
04/16/12   	Changhan Tsai	Initial version
06/19/12	KevinA_Lin		add suspned_info_is_enable() function, be used to return suspend_info_enable
===========================================================================*/

#ifndef _LINUX_SUSPEND_INFO_H
#define _LINUX_SUSPEND_INFO_H

#include <linux/list.h>

struct suspend_info_ops {
	struct list_head node;
	int (*suspend)(void);
	void (*resume)(void);
	void	(*dbg_show)(struct seq_file *s);
};

extern void register_suspend_info_ops(struct suspend_info_ops *ops);
extern void unregister_suspend_info_ops(struct suspend_info_ops *ops);
extern int suspend_info_suspend(void);
extern void suspend_info_resume(void);
extern void suspend_info_shutdown(void);
extern int suspned_info_is_enable(void);
#endif
