/*
 *   Driver for KeyStream 11b/g wireless LAN cards.
 *   
 *   ks_debug.h
 *   $Id: ks_debug.h 623 2008-06-20 08:09:31Z sekine $
 *
 *   Copyright (c) 2005-2008 KeyStream Corp.
 *   All rights reserved.
 *
 */

#ifndef _KS_DEBUG_H
#define _KS_DEBUG_H

#include <linux/kernel.h>


#ifdef KS_WLAN_DEBUG
#define DPRINTK(n, fmt, args...) \
                 if (KS_WLAN_DEBUG>(n)) printk(KERN_NOTICE "%s: "fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(n, fmt, args...)
#endif

extern void print_buffer(unsigned char *p, int size);

#endif /* _KS_DEBUG_H */
