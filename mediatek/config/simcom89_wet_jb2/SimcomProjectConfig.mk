#
# 　请在此注释下增加控制常量
# 　控制常量命名规则：
#   控制常量共分四段，如下所示：
#   LAYER_CUSTOMER_PROJECT_MODULE_FUNCTION
#   LAYER段：　此段分
#      MMI：　仅涉及上层ＡＰＫ的修改使用此标识
#      FMWK: 仅涉及Framework层的修改使用此标识
#      CMN：　common缩写，涉及上层ＡＰＫ与framework等多层的修改使用此标识
#      
#   CUSTOMER_PROJECT段：
#      此段是客户名称与项目名称的全称，如针对联想　A390e项目的功能, 应写为 LENOVO_A390E
#      如果此功能应用于此客户的所有项目的话，可发不写项目名称，如针对联想所有项目的功能，应写为 LENOVO
#      如果此功能是平台性功能，可能会应用于所有客户的话，应写为 PLATFORM
#      
#   MODULE段：
#      此段为模块名称，如电话本模块的修改就写为 CONTACT,
#      
#   FUNCTION段：
#      此段为功能名称
#      
#   示例：
#      电话号１１位匹配功能，需要修改framework代码，应用于平台
#      FMWK_PLATFORM_TELEPHONY_PHONE_NUMBER_11_MATCH
#      
#      电话号码１３位匹配功能，需要修改framewor代码，　应用于联想，Ａ３项目
#      FMWK_LENOVO_A3_TELEPHONY_PHONE_NUMBER_13_MATCH
#      
#      去电彩铃音量抵制功能，需要修改　phone 这个ＡＰＫ中的代码，应用于平台
#      MMI_PLATFORM_PHONE_RINGTONE_VOLUMEN_LIMITED
#             
#    
#
    
#客户项目单拉时需要对此值做修改
CMN_PLATFORM_CUSTOMER_NAME = ACER

#客户项目单拉时需要对此值做修改
CMN_PLATFORM_CUSTOMER_AND_PROJECT_NAME = ACER_C11
    

#add by baoqin.zhang @ 2012-05-29
MOBILEDATA_ATTECHON_POWERSAVE_WIDGET = no

#进入通话记录界面时，先隐藏上面四个过滤器按钮，向下滑动屏幕后再显示出来。
#2012.6.18 added by  huiyang.tang
MMI_ENTER_CALLLOG_HIDE_FILTER_BUTTON = no


#launcher2 加入icon背景图片
#add by heyunlong for doov yes/no
MMI_DOOV_ICON_BITMAP_BACKGROUND = no
#中文（简繁）时，日历中增加农历显示，包括标题农历提示
#add  by 王娜 20120427
MMI_CALENDAR_LUNAR_DISPLAY = yes

#add by wentao for doov 2012-06-19 yes/no
MMI_LAUNCHER_SPECICAL_FOLDER = no
MMI_DISPLAY_ONE_IMEI = no

#add by hongjing.dong for acer launcher allapp list background
#yes is B0000000,no is FF000000
MMI_LAUNCHER_ALLAPP_BACKGROUND = no

#add by wpf for acer get voicemail number from network profile( PLMN ) when user don't save voicemail
FMWK_PLATFORM_CALL_GET_VOICEMAILNUMBER_FROM_NETWORK_PROFILE = yes

#add by wentao 去除workspace 搜索框  yes/no
MMI_ACER_C8_AK330_LAUNCHER_REMOVE_SEARCHBAR = no
#add by wentao 去除搜索框后，workspace 行数加1 yes/no
MMI_ACER_C8_AK330_LAUNCHER_REMOVE_SEARCHBAR_ADD_CELLCOUNTY = no

#20120824 zhangjin
MMI_CAMERA_REVIEW_TOUCH_ADJUST_TO_ZOOM = no

#拨号接听震动提醒功能
#add by pengfei.wang 2012-01-11
MMI_PLATFORM_PHONE_DIALING_CONNECTED_REMINDER = no   

#拨号挂断震动提醒功能
#add by huiyang.tang 2012-09-03
MMI_CALL_DISCONNECTED_REMINDER = no

#add by wentao,2012.9.11 状态栏透明 相应的拉长下拉菜单。
MMI_FMWK_STATUSBAR_TRANSMISSION = no

#add by wentao,2012.9.11 虚拟导航键盘透明 ，拉长壁纸   ---开启虚拟导航键的项目才能够打开
MMI_FMWK_NAVIGATIONBAR_TRANSMISSION = no

#add by chen.yang,2012.9.20 搜索中默认的引擎是baidu
MMI_PLATFORM_DEFAULT_SEARCH_EANGINE_AS_BAIDU = no

#20121101，NavyGuo
MMI_PICTURE_REMOVE_FLASH_DETAIL = no

#yangyang 20120927 for sdcard update mennu:setting--->About->sdcard update
SDCARD_UPDATE_MENU = no
#yangyang 20121015 in filemanager apk--.selsect zip file-->sdcard update
SDCARD_UPDATE_SERVICE = no

# 使用TP代替距离传感器 MMI需要需改部分
# 2012.09.13 曹然格
MMI_PLATFORM_TPD_PS_SUPPORT = no

#add by wangying for cpu model info
MMI_DISPLAY_CPU_MODEL = no

# 来电闪
# 2012.09.13 曹然格
MMI_PLATFORM_CALLING_LIGHT_SUPPORT = no
#来电翻转静音 曹然格
MMI_PHONE_FLIP_SILENT = no

#20130106,qiaoxiujun
MMI_CAMERA_CAPTURE_TO_FOCUS = no

#20130228,yuanhl Acer World wide SKU
MMI_ACER_WW = yes
#20130307 zhujing
#支持camera 手动对焦时，长按对焦点可以拍照
MMI_CAMERA_LONG_PRESS_CAPTURE_FOCUS= yes

#xuxiaohui 2013-02-27 add for C8 LED
FMWK_ACER_SHOW_LED = yes

#状态栏信号格逐步更新（不是阶跃跳变）
#add guozuo.zhang 2013-03-15
MMI_FMWK_GEMINI_SIGNAL_SMOOTH_UPDATE = yes
#彩铃抑制功能
#add huiyang.tang 2013-03-21
MMI_CALL_VOICE_TELEPHONE_VOLUME_LIMITED = yes

#插入耳机振动
#add by NavyGuo 20130325
MMI_VIB_HEADSET_IN = yes
