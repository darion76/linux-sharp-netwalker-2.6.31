/*
 *   Driver for KeyStream 11b/g wireless LAN cards.
 *   
 *   ks_debug.c
 *   $Id: ks_debug.c 623 2008-06-20 08:09:31Z sekine $
 *
 *   Copyright (c) 2005-2008 KeyStream Corp.
 *   All rights reserved.
 *
 */
#include "ks_wlan.h"
#include "ks_debug.h"

void print_buffer(unsigned char *p, int length)
{
#ifdef KS_WLAN_DEBUG
        int i;
#define HEX_OFFSET "\
           +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F"
        printk(HEX_OFFSET);
        for (i=0; i<length; i++) {
                if (i % 16 == 0) printk("\n%04X-%04X:", i, i+15);
                printk(" %02X", *(p+i));
        }
        printk("\n");
#endif
}
