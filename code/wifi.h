# ifndef __WIFI_H
# define __WIFI_H

#include "zf_common_headfile.h"
#include "imu.h"
#include "pid.h"
#include "location.h"
#include "control_state.h"
#include "image.h"

#define WIFI_SSID_TEST "Hhhyt"
#define WIFI_PASSWORD_TEST "18273645"
#define TCP_TARGET_IP "192.168.137.1"
#define TCP_TARGET_PORT "8086"
#define WIFI_LOCAL_PORT "6666"

void wifi_init(void);
void wifi_send_data(void);
void wifi_send_image_orin(void);
void wifi_send_image_thre(void);

#endif