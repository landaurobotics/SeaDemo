//
// Created by yc on 2023/3/14.
//

#ifndef DOUBLEENCODER_ECATCONFIG_H
#define DOUBLEENCODER_ECATCONFIG_H

#include <stdlib.h>
#include <termio.h>
#include <iostream>
#include <errno.h>
#include "signal.h"
#include "stdio.h"
#include "sys/resource.h"
#include "sys/time.h"
#include "sys/types.h"
#include "unistd.h"
#include "sys/mman.h"
#include <pthread.h>
#include <cmath>

// EtherCAT header
#include "ecrt.h"

// CSV header
#include <cstring>
#include <vector>
#include <fstream>
#include <sstream>

#include <vector>

extern pthread_mutex_t mutex;
extern pthread_cond_t cond;

///////////// Extern Variable by Yang Luo ////////////////////

#include <iostream>
#include <stdlib.h>
#include <math.h>

#define CYCLE_TIME 1000 // 1kHz
#define RES_HIGH 262144 // 高速端分辨率 18bit
#define RES_LOW 65536   // 低速段分辨率 14bit
#define RATIO 100       // 减速比100:1
#define STIFFNESS 126.0507 // 扭簧刚度 2.2 Nm/°

#define IMP_CTRL 11
#define ZERO_FORCE_CTRL 22

#define STAT_STOPPED 0
#define STAT_RUNNING 1
#define STAT_ERROR   2

extern int ctrl_mode; //默认阻抗模式
extern bool enabled;      //默认启动
extern int g_status;    //默认状态 stopped

extern double imp_stiff;  // 默认阻抗刚度
extern double imp_damp;  // 默认阻抗阻尼

/////////////////////////////////////////////////////////////

#define TASK_FREQUENCY 1000.0 // Hz

/* NOTICE:
 * Change TASK_FREQUENCY to higher value (4kHz++) may cause losing heartbeats
 */

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

#define CSP 8
#define CSV 9
#define CST 10

#define ETHERCAT_STATUS_OP 0x08
#define STATUS_SERVO_ENABLE_BIT (0x04)

#define joint_num 1
#define active_num 1

//// EtherCAT port Mapping
// #define l_hip   3
// #define l_knee  4
// #define l_ankle 5
// #define r_hip   0
// #define r_knee  1
// #define r_ankle 2



#endif // DOUBLEENCODER_ECATCONFIG_H
