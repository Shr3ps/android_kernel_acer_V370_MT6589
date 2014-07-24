#include "tpd.h"

#include "tpd_custom_fts.h"
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
///
#ifdef FT5336_DOWNLOAD
#include "ft5336_download_lib.h"

static unsigned char CTPM_MAIN_FW[]=
{
	#include "ft_all.i"
};
#endif


/*punk*/
#ifdef TP_GESTURE_SUPPORT
#include "ft_gesture_lib.h"
static short pointnum = 0;
#if 0
unsigned long time_stamp = 0;
bool is_one_finger = true;
unsigned short data_xx[150] = {0};
unsigned short data_yy[150] = {0};
unsigned char data_xy[150] = {0};
#else
#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4
int gestrue_id = 0;
#endif
extern int tpd_chk_ges_en(void);
#ifdef TPD_SUPPORT_DOOV
static unsigned char tpd_hall_sensor_enable=0;
#endif
extern int fts_dma_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen);
//punk
extern u8 *I2CDMABuf_va;
extern volatile u32 I2CDMABuf_pa;
#endif

#ifdef TPD_SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "cust_gpio_usage.h"
  
#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect 			= 1;//0-->close ; 1--> far away
#endif

extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
 
 
static void tpd_eint_interrupt_handler(void);
// start:Here maybe need port to different playform,like MT6575/MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
// End:Here maybe need port to different playform,like MT6575/MT6577
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
//zengfl add start
//static void tpd_shutdown(struct i2c_client *client);
#ifdef USB_CHARGE_DETECT
 static void tpd_usb_plugin(int plugin);
 static struct delayed_work Judge_tp_delayed_work;
 static int b_usb_plugin = 0;
 static int b_tpd_suspend = 0;
#endif
//zengfl add end

//FANG Zhuo -- 2013/09/11
extern kal_bool upmu_is_chr_det(void);
void tpd_usb_plugin(int plugin)
{
	int ret = -1;
	int ret1 = 0,ret2 = 0;
	//int temp = upmu_is_chr_det();
	int temp = plugin;

	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &temp);
	//msleep(50);
	//i2c_smbus_read_i2c_block_data(i2c_client, 0x8B, 1, &ret1);
	//printk("%s,%d,ret1=%d,temp=%d\n",__func__, __LINE__, ret1,temp);
	if ( ret < 0 )
	{
		printk("Fts usb detect write err: %s.\n",__func__);
	}
}
EXPORT_SYMBOL(tpd_usb_plugin);

static int tpd_flag 					= 0;
static int tpd_halt						= 0;
static int point_num 					= 0;
static int p_point_num 					= 0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 							0

//register define
#define DEVICE_MODE 					0x00
#define GEST_ID 						0x01
#define TD_STATUS 						0x02
//point1 info from 0x03~0x08
//point2 info from 0x09~0x0E
//point3 info from 0x0F~0x14
//point4 info from 0x15~0x1A
//point5 info from 0x1B~0x20
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 			3
//extern int tpd_mstar_status ;  // compatible mstar and ft6306 chenzhecong

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
//static int tpd_keys_dim_local_fwvga[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_FWVGA;
//static int tpd_keys_dim_local_qhd[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_QHD;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 			10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 			10
#endif

#define TOUCH_IOC_MAGIC 				'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FTS_NAME,
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[TPD_MAX_POINTS];
    int x[TPD_MAX_POINTS];
    int p[TPD_MAX_POINTS];
    int id[TPD_MAX_POINTS];
#if 0//def TP_GESTURE_SUPPORT
    int xy[TPD_MAX_POINTS];
#endif
};
 
static const struct i2c_device_id ft5206_tpd_id[] = {{FTS_NAME,0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO(FTS_NAME, (0x70>>1))};
 
static struct i2c_driver tpd_i2c_driver = {
  	.driver = {
	 	.name 	= FTS_NAME,
	//	.owner 	= THIS_MODULE,
  	},
  	.probe 		= tpd_probe,
  	.remove 	= __devexit_p(tpd_remove),
  	.id_table 	= ft5206_tpd_id,
  	.detect 	= tpd_detect,
// 	.shutdown	= tpd_shutdown,
//  .address_data = &addr_data,
};
///////////////////////////////////////////////////////////////////////
#ifdef FT5336_DOWNLOAD

extern u8 *I2CDMABuf_va;
extern volatile u32 I2CDMABuf_pa;

int ft5x0x_download_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret,i;
#if 0
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },

		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "%s:i2c read error.\n", __func__);
	}

#else// for DMA I2c transfer
//TPD_DMESG("[FTS] test----1\n");
	if(writelen!=0)
	{
		//DMA Write
		if(writelen < 8  )
		{
//TPD_DMESG("[FTS] test----3\n");
#if (defined MT6589)
			i2c_client->ext_flag = i2c_client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);		
#endif			
			//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
			//for(i = 0 ; i < writelen; i++)
			{
				//TPD_DMESG("[FTS] test:\n");
			}
			ret= i2c_master_send(i2c_client, writebuf, writelen);
//ret = i2c_master_send(i2c_client, writebuf, writelen);
		}
		else
		{
//TPD_DMESG("[FTS] test----4\n");
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}
//TPD_DMESG("[FTS] test----5\n");
#if (defined MT6589)
			i2c_client->ext_flag = i2c_client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
#elif (defined MT6572)
			i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif			
			if((ret=i2c_master_send(i2c_client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&i2c_client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
			//return ret;
		}
	}
//TPD_DMESG("[FTS] test----6\n");
	//DMA Read 
	if(readlen!=0)
	{
		if (readlen <8) {
//TPD_DMESG("[FTS] test----7\n");
#if (defined MT6589)
			i2c_client->ext_flag = i2c_client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
#elif (defined MT6572)
			i2c_client->addr = client->addr & I2C_MASK_FLAG;
#endif			
			ret = i2c_master_recv(i2c_client, (unsigned char *)readbuf, readlen);
		}
		else
		{
//TPD_DMESG("[FTS] test----8\n");
#if (defined MT6589)
			i2c_client->ext_flag = i2c_client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
#elif (defined MT6572)
			i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif					
			ret = i2c_master_recv(i2c_client, (unsigned char *)I2CDMABuf_pa, readlen);
//TPD_DMESG("[FTS] test----9\n");
			for(i = 0; i < readlen; i++)
	        	{
				readbuf[i] = I2CDMABuf_va[i];
	        	}
		}
	}
//TPD_DMESG("[FTS] test----10\n");
	#endif
	
	return ret;
}
/*write data by i2c*/
int ft5x0x_download_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret,i;
#if 0
	struct i2c_msg msg[] = {
		{
		 .addr = 0x38,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&g_i2c_client->dev, "%s i2c write error.\n", __func__);

	#else
	if(writelen < 8)
	{
#if (defined MT6589)
		i2c_client->ext_flag = i2c_client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
#endif			
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(i2c_client, writebuf, writelen);
	}
	else
	{
		for(i = 0 ; i < writelen; i++)
		{
			I2CDMABuf_va[i] = writebuf[i];
		}
#if (defined MT6589)
		i2c_client->ext_flag = i2c_client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;	
#elif (defined MT6572)
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif
		if((ret=i2c_master_send(i2c_client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
			dev_err(&i2c_client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
	} 
	#endif
	return ret;
}

void ft5x0x_reset_tp(int HighOrLow)
{
	TPD_DEBUG("set tp reset pin to %d\n", HighOrLow);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, HighOrLow);
}
EXPORT_SYMBOL(ft5x0x_reset_tp);

int ft5336_Enter_Debug(void)
{
TPD_DMESG("[FTS] ft5336_Enter_Debug\n");
	ft5x0x_reset_tp(0);
	msleep(4);
	ft5x0x_reset_tp(1);
	return ft5336_Lib_Enter_Download_Mode();
}
//if return 0, main flash is ok, else download.
int ft5336_IsDownloadMain(void)
{
	//add condition to check main flash
	return -1;
}
int ft5336_DownloadMain(void)
{
TPD_DMESG("[FTS] ft5336_DownloadMain\n");
	unsigned short fwlen = 0;
	if (ft5336_Enter_Debug() < 0) {
		TPD_DMESG("-----enter debug mode failed\n");
		return -1;
	}
	fwlen = sizeof(CTPM_MAIN_FW);
	TPD_DMESG("----fwlen=%d\n", fwlen);
TPD_DMESG("[FTS] start ft5336_Lib_DownloadMain\n");
	//return ft6x06_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
	return ft5336_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
}
#endif


/////////////////////////////////////////////////////////////////////////

#ifdef USB_CHARGE_DETECT
static  void tpd_power_reset(void)
{
}

static void judge_tp_normal_delayed_work(struct work_struct *work)
{
	int ret = 0;
	char data[2] = {0};

	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xA6, 1, &(data[0]));
	if ( ret < 0 || data[0] == 0xA6 || data[0] == 0 )
	{
		printk("%s read i2c error ret = %d data[0] = %d", __FUNTION__,ret,data[0]);
		tpd_power_reset();
	}
	printk("tpd:%s plugin %d", __func__,b_usb_plugin);
	i2c_smbus_write_i2c_block_data(i2c_client, 0x90, 1, &b_usb_plugin);
	b_tpd_suspend = 0;
}

static void tpd_usb_plugin(int plugin)
{
	b_usb_plugin = plugin;

	printk("tpd: %s %d.\n",__func__,b_usb_plugin);

	if ( b_tpd_suspend ) return;
	
	i2c_smbus_write_i2c_block_data(i2c_client, 0x90, 1, &b_usb_plugin);
}
#endif

void get_tp_fw_id(unsigned char *data)
{
	i2c_smbus_read_i2c_block_data(i2c_client, FT6x06_REG_FW_VER, 1, data);
}

static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("D[%4d %4d %4d] ", x, y, p);
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif	
    {   
      tpd_button(x, y, 1);  
    }
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}
 
static  void tpd_up(int x, int y) {
	//input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif
    {   
       tpd_button(x, y, 0); 
    }   		 
}

#ifdef TP_GESTURE_SUPPORT
#if 0
static int tpd_gesture_handle2(struct touch_info *cinfo)
{

	int i = 0;
	int ret=0;
#if (TPD_MAX_POINTS!=2)
	char data[35] = {0};
#else
	char data[16] = {0};
#endif	
    u16 high_byte,low_byte;
	u8 report_rate =0;

	mutex_lock(&i2c_access);

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
#if (TPD_MAX_POINTS!=2)
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
#endif
	
	mutex_unlock(&i2c_access);
   
	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
		
	for(i = 0; i < point_num; i++)
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
     		cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;
	
		/*get the Y coordinate, 2 bytes*/
		
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;
#ifdef TP_GESTURE_SUPPORT
		cinfo->xy[i] =  data[3+6*i+4];
#endif
	}
	printk(" tpd_gesture_handle2 cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d, cinfo->xy[0]=%d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0],cinfo->xy[0]);	
	//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
	//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	

    if(point_num > 1)
    {
        is_one_finger = false;
    }

	printk(" tpd_gesture_handle2 pointnum=%d\n", pointnum);	

	//TPD_DEBUG("point_num = %d\n",point_num);
	if(point_num >0) 
	{
		for(i =0; i<point_num; i++)
		{				
			if(is_one_finger && pointnum < 150)
			{
	  			data_xx[pointnum] = (unsigned short)cinfo->x[i];
	  			data_yy[pointnum] = (unsigned short)cinfo->y[i];
	  			data_xy[pointnum] = (unsigned char)cinfo->xy[i];
	  			pointnum++;
			}
			else
			{
				is_one_finger = false;
			}
		}
	}
	else  
	{
		ret = fetch_object_sample(data_xx,data_yy,data_xy,pointnum,jiffies);
	    	TPD_DMESG(" %s,  -3.0 , call fetch_object_sample() ret=%d,pointnum=%d\n", __FUNCTION__, ret,pointnum);  

	        is_one_finger = true;
	        pointnum = 0;

	        memset(data_xx, 0, sizeof(unsigned short)*150);
	        memset(data_yy, 0, sizeof(unsigned short)*150);
	        memset(data_xy, 0, sizeof(unsigned char)*150);
	}

	switch (ret)
	{ 
		case 0x20:
			TPD_DMESG(" %s,  -3.1 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_LEFT, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_LEFT, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x21:
			TPD_DMESG(" %s,  -3.2 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_RIGHT, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_RIGHT, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x22:
			TPD_DMESG(" %s,  -3.3 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_UP, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_UP, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x23:
			TPD_DMESG(" %s,  -3.4 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_DOWN, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_DOWN, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x24:
			TPD_DMESG(" %s,  -3.5 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x30:
			TPD_DMESG(" %s,  -3.6 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_O, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_O, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x41:
			TPD_DMESG(" %s,  -3.7 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_Z, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_Z, 0);
			input_sync(tpd->kpd); 
			break;
		default:
			TPD_DMESG(" %s,  -3.8 , not detect key, key value=%x\n", __FUNCTION__,ret);  
			break;

	}

	return 0;

}
#else
static int tpd_gesture_handle2(struct touch_info *cinfo)
{

    unsigned char buf[2] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;

    ret = fts_dma_i2c_Read(i2c_client, buf, 1,(unsigned char *)I2CDMABuf_pa,FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0)
    {
        TPD_DMESG("%s read touchdata failed.\n", __func__);
        return ret;
    }

    if (0x24 == I2CDMABuf_va[0])
    {
        gestrue_id = 0x24;
	TPD_DMESG(" %s,  -3.5 \n", __FUNCTION__);  
        input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 1);
        input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 0);
	input_sync(tpd->kpd); 
        return 0;
    }

    pointnum = (short)(I2CDMABuf_va[1]) & 0xff;

	buf[0] = 0xd3;

    ret = fts_dma_i2c_Read(i2c_client, buf, 1,(unsigned char *)I2CDMABuf_pa,(pointnum * 4 + 2));
    if (ret < 0)
    {
     TPD_DMESG("%s error @  line 772 fts_dma_i2c_Read.\n", __func__);
        return ret;
    }
	
	
    gestrue_id = fetch_object_sample(I2CDMABuf_va,pointnum);
    TPD_DMESG(" %s,  -3.0 tpd_gesture_handle2 pointnum=%d,gestrue_id=0x%x\n", __FUNCTION__,pointnum,gestrue_id);  

    switch (gestrue_id)
    { 
		case 0x20:
			TPD_DMESG(" %s,  -3.1 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_LEFT, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_LEFT, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x21:
			TPD_DMESG(" %s,  -3.2 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_RIGHT, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_RIGHT, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x22:
			TPD_DMESG(" %s,  -3.3 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_UP, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_UP, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x23:
			TPD_DMESG(" %s,  -3.4 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_DOWN, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_DOWN, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x24:
			TPD_DMESG(" %s,  -3.5 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_DOUBLE_TAP, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x30:
			TPD_DMESG(" %s,  -3.6 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_O, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_O, 0);
			input_sync(tpd->kpd); 
			break;
		case 0x41:
			TPD_DMESG(" %s,  -3.7 \n", __FUNCTION__);  
            input_report_key(tpd->kpd, KEY_GESTURE_Z, 1);
            input_report_key(tpd->kpd, KEY_GESTURE_Z, 0);
			input_sync(tpd->kpd); 
			break;
		default:
			TPD_DMESG(" %s,  -3.8 , not detect key, key value=%x\n", __FUNCTION__,ret);  
			break;

	}

	return 0;

}
#endif
#endif

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
#if (TPD_MAX_POINTS!=2)
	char data[35] = {0};
#else
	char data[16] = {0};
#endif	
    u16 high_byte,low_byte;
	u8 report_rate =0;

	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
#if (TPD_MAX_POINTS!=2)
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
#endif
	
	mutex_unlock(&i2c_access);
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	//TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);
   
	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
		
	for(i = 0; i < point_num; i++)
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
     	cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;

		//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
	
		/*get the Y coordinate, 2 bytes*/
		
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;

		 //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
	}
	printk(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d, point_num=%d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0],point_num);	
	//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
	//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	return true;
};

#ifdef TPD_PROXIMITY
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	printk("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
	}else{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
	}

	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);		
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}	
				
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;	
}
#endif
#ifdef TPD_SUPPORT_DOOV
void tpd_enable_hallsensor_dov(u8 enable)
{
	u8 state=0;
	int ret = -1;

	if (enable){
		state |= 0x01;
	        ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xC1, 1, &state);
		tpd_hall_sensor_enable=1;
		TPD_DMESG(" %s, write i2c reg 0xc1 ,state=%d, ret=%d\n", __FUNCTION__,state,ret);  
	}else{
		tpd_hall_sensor_enable=0;
		state &= 0x00;	
	        ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xC1, 1, &state);
		TPD_DMESG(" %s, wrinte i2c reg0xc1 ,state=%d, ret=%d\n", __FUNCTION__,state,ret);  
	}

	return;
}
#endif
 static int touch_event_handler(void *unused)
 { 
   	struct touch_info cinfo, pinfo;
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	//FANG Zhuo -- 2013/09/11
	//kal_bool temp = upmu_is_chr_det();

#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
	u8 state;
#endif
 
	do
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;
			 
		set_current_state(TASK_RUNNING);

     		//FANG Zhuo -- 2013/09/11
		//if(temp)
		{
			tpd_usb_plugin(upmu_is_chr_det());
		}


#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
			
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

			if ((err = tpd_read_ps()))
			{
				TPD_PROXIMITY_DMESG("proxi_5206 read ps data 1156: %d\n", err);	
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
				TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			}
		}  
#endif
	#ifdef TP_GESTURE_SUPPORT
		if(tpd_chk_ges_en() &&(tpd_halt==1))
			tpd_gesture_handle2(&cinfo);
	#endif

		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
			    for(i =0; i<point_num; i++)//only support 3 point
			    {
			        // tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
				if(cinfo.y[i]>TPD_RES_Y) //button area
					tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
				else	//lcd area
					tpd_down(TPD_WARP_X(cinfo.x[i]), TPD_WARP_Y(cinfo.y[i]), cinfo.id[i]);
			    }
				
			    input_sync(tpd->dev);
			}
			else  
    		{
			    //tpd_up(cinfo.x[0], cinfo.y[0]);
				if(cinfo.y[0]>TPD_RES_Y) //button area
					tpd_up(cinfo.x[0], cinfo.y[0]);
				else	//lcd area
					tpd_up(TPD_WARP_X(cinfo.x[0]), TPD_WARP_Y(cinfo.y[0]));			    
        	    //TPD_DEBUG("release --->\n"); 
        	    //input_mt_sync(tpd->dev);
        	    input_sync(tpd->dev);
        		}
        	}

        	if(tpd_mode==12)
        	{
           //power down for desence debug
           //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
			hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
			hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
			hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif 
    		msleep(20);
    	}

 	}while(!kthread_should_stop());
 
	return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;

reset_proc:   
	i2c_client = client;

	//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#if 0 //def TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 


#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
#else
   	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(1);//1 longxuewei
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);

	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 

        //mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	msleep(100);
 #ifdef TP_GESTURE_SUPPORT
	init_para(720,1280,50,0,0); 
 #endif
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}

	tpd_load_status = 1;
 //   tpd_mstar_status =0 ;  // compatible mstar and ft6306 chenzhecong

	#ifdef FTS_APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
    #endif
	#ifdef TPD_SYSFS_DEBUG
	fts_create_sysfs(i2c_client);
	#endif

	#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		TPD_DMESG(TPD_DEVICE, "%s:[FTS] create fts control iic driver failed\n",__func__);
	#endif
	
	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif


	//get some register information
	i2c_smbus_read_i2c_block_data(i2c_client, FT6x06_REG_FW_VER, 1, &data);
	TPD_DMESG("[FTS] Firmware version = 0x%x\n", data);


	
	
#ifdef FT5336_DOWNLOAD
			FTS_I2c_Read_Function fun_i2c_read = ft5x0x_download_i2c_Read;
			FTS_I2c_Write_Function fun_i2c_write = ft5x0x_download_i2c_Write;
			Init_I2C_Read_Func(fun_i2c_read);
			Init_I2C_Write_Func(fun_i2c_write);
			 if(ft5336_IsDownloadMain() < 0) {
	 	#if 1
				TPD_DMESG("--------FTS---------download main\n");
				if(ft5336_DownloadMain()<0)
				{
					TPD_DMESG("---------FTS---------Download main failed\n");
				}
		#endif
			 } else
				TPD_DMESG("--------FTS---------no download main\n");
#endif
	
	
	#ifdef TPD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
	#endif
	
	
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	TPD_DMESG("FTS Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
#ifdef USB_CHARGE_DETECT
	INIT_DELAYED_WORK(&Judge_tp_delayed_work,judge_tp_normal_delayed_work);
#endif

#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
	
	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("proxi_fts attach fail = %d\n", err);
	}
	else
	{
		APS_ERR("proxi_fts attach ok = %d\n", err);
	}		
#endif
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
{

        #ifdef FTS_APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
   	#ifdef TPD_SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif

	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	
	TPD_DEBUG("TPD removed\n");
 
   	return 0;
}

static int tpd_local_init(void)
{
  	TPD_DMESG("FTS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
   	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("FTS unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("FTS add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
/*
	if(TPD_RES_Y > 854)
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_qhd);// initialize tpd button data
	}
	else
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_fwvga);// initialize tpd button data
	}
	*/
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  //char data;
  //FANG Zhuo -- 2013/09/11
  //kal_bool temp = upmu_is_chr_det();
  
#ifdef TP_GESTURE_SUPPORT
	u8 data = 0;
	int ret;
#endif

//#ifdef TP_GESTURE_SUPPORT
	/*Not check gesture status when resume*/
#if 0
	if(tpd_chk_ges_en())
	{
		/*Disable gesture when exit sleep*/
		ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data);  //TP enter sleep mode
		TPD_DMESG(" %s, wrinte i2c reg0xd0 ,data=%d, ret=%d\n", __FUNCTION__,data,ret);  
		/*read for check*/
		ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &data);  //TP enter sleep mode
		TPD_DMESG(" %s, read i2c reg0xd0 ,data=%d,  ret=%d \n", __FUNCTION__,data,ret);  
		//!!! 
		return;
	}
#endif

#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	
 
   	TPD_DMESG("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

#else
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM,"TP");
	msleep(30);
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM,VOL_2800,"TP");
	msleep(30);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(2);  
   // mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	msleep(150);//add this line  //longxuewei
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
#ifdef USB_CHARGE_DETECT
	schedule_delayed_work(&Judge_tp_delayed_work,msecs_to_jiffies(800));
#endif
//	msleep(30);
	tpd_halt = 0;
	/* for resume debug
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	}
	*/
	tpd_up(0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");

	 //FANG Zhuo -- 2013/09/11
	//if(temp)
	{
		tpd_usb_plugin(upmu_is_chr_det());
	}
#ifdef TPD_SUPPORT_DOOV
	if(tpd_hall_sensor_enable==1)
		tpd_enable_hallsensor_dov(1);
#endif
	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;
#ifdef TP_GESTURE_SUPPORT
	int ret;
#endif
 	 tpd_halt = 1;

#ifdef TP_GESTURE_SUPPORT

	if(tpd_chk_ges_en())
	{
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(10);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(150);

		/*Enable gesture when phone in suspend mode*/
		data = 0x01;
		ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data);
		TPD_DMESG(" %s, wrinte i2c reg0xd0 ,data=%d, ret=%d\n", __FUNCTION__,data,ret);  

		/*All gesture on*/
		data = 0x1f;
		ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xd1, 1, &data);
		TPD_DMESG(" %s, wrinte i2c reg0xd0 ,data=%d, ret=%d\n", __FUNCTION__,data,ret);  

		data = 0x1f;
		ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xd2, 1, &data);
		TPD_DMESG(" %s, wrinte i2c reg0xd0 ,data=%d, ret=%d\n", __FUNCTION__,data,ret);  

		return;
	}

#endif

	data = 0x3;

	//i2c_smbus_read_i2c_block_data(i2c_client, FT6x06_REG_FW_VER, 1, &data);
	//TPD_DMESG("[FTS] Firmware version = 0x%x\n", data);
	//data = 0x3;

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif


#ifdef USB_CHARGE_DETECT
	b_tpd_suspend = 1;
	cancel_delayed_work_sync(&Judge_tp_delayed_work);
#endif
	 TPD_DMESG("TPD enter sleep\n");
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	mutex_lock(&i2c_access);
	data = 0x3;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	mutex_unlock(&i2c_access);
#endif
	TPD_DMESG("TPD enter sleep done\n");
	//return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = FTS_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("MediaTek FTS touch panel driver init\n");
	i2c_register_board_info(TPD_I2C_NUMBER, &ft5206_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FTS driver failed\n");
	 return 0;
 }
 
 /* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FTS touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}
 
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


