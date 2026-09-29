/*
 * TEST HARNESS ONLY -- LD_PRELOAD shim for loopback_test.sh.
 *
 * A veth pair reports 10 Gbit/s. p-net v0.2.0 only maps 10/100/1000 Mbit/s
 * copper to a MAU type, so it decides there is "no local Ethernet port with
 * high enough speed" and aborts the connection at PrmEnd
 * (PNET_ERROR_CODE_2_ABORT_PDEV_CHECK_FAILED). A real PROFINET port is
 * 100 Mbit/s or 1 Gbit/s and never hits this.
 *
 * This reports 1000 Mbit/s full duplex for ONE interface, named in
 * MW_FAKE_LINK_SPEED_IFACE, and passes every other ioctl through untouched.
 * Never preload it outside the loopback test.
 *
 *   gcc -shared -fPIC -o fake_link_speed.so fake_link_speed.c -ldl
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

int ioctl (int fd, unsigned long request, ...)
{
   static int (*real_ioctl) (int, unsigned long, void *) = NULL;
   va_list ap;
   void * arg;

   va_start (ap, request);
   arg = va_arg (ap, void *);
   va_end (ap);

   if (real_ioctl == NULL)
   {
      real_ioctl = (int (*) (int, unsigned long, void *))dlsym (RTLD_NEXT, "ioctl");
   }
   int ret = real_ioctl (fd, request, arg);

   const char * target = getenv ("MW_FAKE_LINK_SPEED_IFACE");
   if (ret >= 0 && request == SIOCETHTOOL && target != NULL && arg != NULL)
   {
      struct ifreq * ifr = (struct ifreq *)arg;
      struct ethtool_cmd * cmd = (struct ethtool_cmd *)ifr->ifr_data;
      if (strncmp (ifr->ifr_name, target, IFNAMSIZ) == 0 && cmd != NULL &&
          cmd->cmd == ETHTOOL_GSET)
      {
         ethtool_cmd_speed_set (cmd, SPEED_1000);
         cmd->duplex = DUPLEX_FULL;
         cmd->port = PORT_TP;
      }
   }
   return ret;
}
