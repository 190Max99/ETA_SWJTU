/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_config
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#ifndef _zf_device_config_h_
#define _zf_device_config_h_

extern const unsigned char image_frame_header[4];
extern const unsigned char imu660ra_config_file[8192];
extern const unsigned char dl1b_config_file[135];

unsigned char   ov7725_sccb_check_id            (void *soft_iic_obj);
unsigned char   ov7725_sccb_set_config          (const unsigned short buff[6][2]);

unsigned char   mt9v03x_sccb_check_id           (void *soft_iic_obj);
unsigned char   mt9v03x_sccb_set_config         (const short int buff[10][2]);
unsigned char   mt9v03x_sccb_set_exposure_time  (unsigned short int light);
unsigned char   mt9v03x_sccb_set_reg            (unsigned char addr, unsigned short int data);

unsigned char   scc8660_sccb_check_id           (void *soft_iic_obj);
unsigned char   scc8660_sccb_set_config         (const short int buff[11][2]);
unsigned char   scc8660_sccb_set_brightness     (unsigned short int brightness);
unsigned char   scc8660_sccb_set_manual_wb      (unsigned short int manual_wb);
unsigned char   scc8660_sccb_set_reg            (unsigned char reg, unsigned short int data);

#endif