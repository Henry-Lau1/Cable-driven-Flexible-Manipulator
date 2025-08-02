#ifndef __USBCAN_
#define __USBCAN_

#include "controlcan.h"
#include <pthread.h>
#include <stdio.h>
#include <chrono>

namespace RxbHardwareInterface{

  void *receive_func(void *param); 

  void can_init(UCHAR Timing0, UCHAR Timing1);

  void can_close(); //can不用了记得关闭
 

}

#endif
