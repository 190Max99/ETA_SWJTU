/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm0plus
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� 
// �������ǿ�Դ��չ��� 
// �������ǿ�Դ��չ��� 

int main(void)
{
    __enable_irq();
    SystemInit();                                                       // ϵͳ��ʼ�� ��ϵͳʱ�ӡ�ϵͳ��Դ��������
    debug_init();			                                // ��ʼ��Ĭ�ϵ��Դ���
    Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);         // ����M7����0
    Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR);         // ����M7����1
    // �˴���д�û���ʼ������ 


    
    
    
    
    // �˴���д�û���ʼ������ 
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

      



        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

// **************************** �������� ****************************

