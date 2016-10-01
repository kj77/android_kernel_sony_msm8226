#include "myLog.h"

#define SHOW_ARGS() do {\
    va_list va;\
    const char *PSTR;\
    va_start(va, FUNC_N_FORMAT);\
    PSTR = va_arg(va, const char *);\
    vprintk(PSTR, va);\
    va_end(va);\
}while(0)


void showFinCallerFuncArgs(
    const int NARGS,
    const void *CALLER_ADDR,
    const char *FUNC_N_FORMAT, ...)
{
    printk(KERN_INFO "+++ %s +++ ", FUNC_N_FORMAT);
    if (NARGS)
        SHOW_ARGS();
    printk(" << %ps", CALLER_ADDR);
    printk("\n");
}

void showFoutCallerFuncArgs(
    const int NARGS,
    const void *CALLER_ADDR,
    const char *FUNC_N_FORMAT, ...)
{
    printk(KERN_INFO "--- %s --- ",FUNC_N_FORMAT);
    if (NARGS)
        SHOW_ARGS();
    printk("\n");
}

void showFuncLineArgs(
    const int NARGS,
    const unsigned long LINE_NUM,
    const char *FUNC_N_FORMAT, ...)
{
    printk(KERN_INFO "=== %s:%ld === ", FUNC_N_FORMAT, LINE_NUM);
    if (NARGS)
        SHOW_ARGS();
    printk("\n");
}

