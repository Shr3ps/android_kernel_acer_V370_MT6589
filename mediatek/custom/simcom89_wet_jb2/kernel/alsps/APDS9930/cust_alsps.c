#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 3,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_LDO_VGP5,    /*LDO is not used*/
    .power_vol  = VOL_2800,          /*LDO is not used*/
    .i2c_addr   = {0x39, 0x39, 0x39, 0x39},
    .als_level  = { 20, 3000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,65535},
    .als_value  = { 15, 200, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    .ps_threshold_high = 53,
    .ps_threshold_low = 46,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

int APDS9930_CMM_PPCOUNT_VALUE = 0x04;
int APDS9930_CMM_CONTROL_VALUE = 0x60;  //50MA,1gain;  Again:0--0;1---8;2--16;3---120
int ZOOM_TIME=2247;