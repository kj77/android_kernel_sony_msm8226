/*===========================================================================
File: kernel/kernel/power/suspend_info.c
Description: Central suspend/resume debug mechanism for peripheral devices (gpio/regulator/wake-up) 

Revision History:
when       	who				what, where, why
--------   	---				---------------------------------------------------------
04/16/12	Changhan Tsai	Initial version
06/19/12	KevinA_Lin		add suspned_info_is_enable() function, be used to return suspend_info_enable
===========================================================================*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/suspend_info.h>
#include <linux/syscore_ops.h>
#include <linux/time.h>

static LIST_HEAD(suspend_info_ops_list);
static DEFINE_MUTEX(suspend_info_ops_lock);

static struct dentry *suspend_info_debugfs_dir;
static unsigned int suspend_info_enable = 0;

/**
 * register_suspend_info_ops - Register a set of system core operations.
 * @ops: System core operations to register.
 */
void register_suspend_info_ops(struct suspend_info_ops *ops)
{
	mutex_lock(&suspend_info_ops_lock);
	list_add_tail(&ops->node, &suspend_info_ops_list);
	mutex_unlock(&suspend_info_ops_lock);
}
EXPORT_SYMBOL_GPL(register_suspend_info_ops);

/**
 * unregister_suspend_info_ops - Unregister a set of system core operations.
 * @ops: System core operations to unregister.
 */
void unregister_suspend_info_ops(struct suspend_info_ops *ops)
{
	mutex_lock(&suspend_info_ops_lock);
	list_del(&ops->node);
	mutex_unlock(&suspend_info_ops_lock);
}
EXPORT_SYMBOL_GPL(unregister_suspend_info_ops);

static int get_enable(void *data, u64 * val)
{
	int ret;

	ret = suspend_info_enable;
	*val = ret;
	return 0;
}

static int set_enable(void *data, u64 val)
{
	suspend_info_enable = (unsigned int)val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, get_enable, set_enable, "%llu\n");

int suspned_info_is_enable(void)
{
	return suspend_info_enable;
}
EXPORT_SYMBOL_GPL(suspned_info_is_enable);

static int suspend_info_show(struct seq_file *s, void *unused)
{
	struct suspend_info_ops *ops;

	list_for_each_entry(ops, &suspend_info_ops_list, node)
		if (ops->dbg_show) {
			if (initcall_debug)
				pr_info("%s(): Calling %pF\n", __func__, ops->dbg_show);
			ops->dbg_show(s);
		}

	return 0;
}

static int suspend_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, suspend_info_show, NULL);
}

static const struct file_operations suspend_info_fops = {
	.open	= suspend_info_open,
	.read		= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static int suspend_info_syscore_suspend(void)
{
	struct suspend_info_ops *ops;
	int ret = 0;

	if(suspend_info_enable)
	{
		list_for_each_entry_reverse(ops, &suspend_info_ops_list, node)
			if (ops->suspend) {
				if (initcall_debug)
					pr_info("%s(): Calling %pF\n", __func__, ops->suspend);
				ret = ops->suspend();
				if (ret)
					goto err_out;
			}
	}
	return 0;

 err_out:
	pr_err("PM: System core suspend callback %pF failed.\n", ops->suspend);

	list_for_each_entry_continue(ops, &suspend_info_ops_list, node)
		if (ops->resume)
			ops->resume();

	return ret;
}

static void suspend_info_syscore_resume(void)
{
	struct suspend_info_ops *ops;

	if(suspend_info_enable)
	{
		list_for_each_entry(ops, &suspend_info_ops_list, node)
			if (ops->resume) {
				if (initcall_debug)
					pr_info("%s(): Calling %pF\n", __func__, ops->resume);
				ops->resume();
			}
	}
}

static struct syscore_ops suspend_info_syscore_ops = {
	.suspend = suspend_info_syscore_suspend,
	.resume = suspend_info_syscore_resume,
};

static int suspend_info_syscore_init(void)
{
	register_syscore_ops(&suspend_info_syscore_ops);

	suspend_info_debugfs_dir = debugfs_create_dir("suspend_info", NULL);
	if (IS_ERR_OR_NULL(suspend_info_debugfs_dir))
		pr_err("%s():Cannot create debugfs dir\n", __func__);

	debugfs_create_file("enable", 0644, suspend_info_debugfs_dir, NULL, &enable_fops);
	debugfs_create_file("suspend_info", 0644, suspend_info_debugfs_dir, NULL, &suspend_info_fops);
	pr_info("%s\n", __func__);

	return 0;
}

static void suspend_info_syscore_exit(void)
{
	debugfs_remove_recursive(suspend_info_debugfs_dir);
	unregister_syscore_ops(&suspend_info_syscore_ops);
	pr_info("%s\n", __func__);
}

late_initcall_sync(suspend_info_syscore_init); //kevin test
module_exit(suspend_info_syscore_exit);

MODULE_AUTHOR("Changhan Tsai <changhan_tsai@compalcomm.com>");
MODULE_DESCRIPTION("Central suspend/resume debug mechanism for peripheral devices");
MODULE_LICENSE("GPL v2");
