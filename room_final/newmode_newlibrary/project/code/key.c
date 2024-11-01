#include "key.h"
uint8 key_value = 0, key_old = 0, key_down = 0; // °´¼üÖµ
extern int16 speed_high;
extern int16 speed_low;
uint8 light_sec[8] = {10,10,10,4,5,10,10,10};
uint8 page = 0;
float kp_back=2;
float kp_forward=2;
float kd_back=0;
float kd_forward=0;

uint8 Key_detec()
{
    uint8 temp = 0;
    gpio_set_level(KEY1, 0);
    gpio_set_level(KEY2, 1);
    gpio_set_level(KEY3, 1);
    gpio_set_level(KEY4, 1);
    if (!gpio_get_level(KEY5))
    {
        temp = 1;
    }
    if (!gpio_get_level(KEY6))
    {
        temp = 2;
    }
    if (!gpio_get_level(KEY7))
    {
        temp = 3;
    }
    if (!gpio_get_level(KEY8))
    {
        temp = 4;
    }
    gpio_set_level(KEY1, 1);
    gpio_set_level(KEY2, 0);
    gpio_set_level(KEY3, 1);
    gpio_set_level(KEY4, 1);
    if (!gpio_get_level(KEY5))
    {
        temp = 5;
    }
    if (!gpio_get_level(KEY6))
    {
        temp = 6;
    }
    if (!gpio_get_level(KEY7))
    {
        temp = 7;
    }
    if (!gpio_get_level(KEY8))
    {
        temp = 8;
    }
    gpio_set_level(KEY1, 1);
    gpio_set_level(KEY2, 1);
    gpio_set_level(KEY3, 0);
    gpio_set_level(KEY4, 1);
    if (!gpio_get_level(KEY5))
    {
        temp = 9;
    }
    if (!gpio_get_level(KEY6))
    {
        temp = 10;
    }
    if (!gpio_get_level(KEY7))
    {
        temp = 11;
    }
    if (!gpio_get_level(KEY8))
    {
        temp = 12;
    }
    gpio_set_level(KEY1, 1);
    gpio_set_level(KEY2, 1);
    gpio_set_level(KEY3, 1);
    gpio_set_level(KEY4, 0);
    if (!gpio_get_level(KEY5))
    {
        temp = 13;
    }
    if (!gpio_get_level(KEY6))
    {
        temp = 14;
    }
    if (!gpio_get_level(KEY7))
    {
        temp = 15;
    }
    if (!gpio_get_level(KEY8))
    {
        temp = 16;
    }
    return temp;
}

void key_proc()
{
    if (key_delay)
        return;
    key_delay = 1;
    if (page == 0)
    {
        key_value = Key_detec();
    key_down = key_value & (key_value ^ key_old);
    key_old = key_value;
        switch (key_down)
        {
        case 1:
          
            break;
        case 2:
             if (page == 0)
            {
                page = 1;
                  ips200_clear();
            }
            // whell_pid.Kp += 0.1;
            // flash_union_buffer[0].float_type  = whell_pid.Kp;
            break;
        case 3:
            
            break;
        case 4:
           
            break;
        case 5:
             kp_back+=0.1;
            break;
        case 6:
            kp_back-=0.1;
            break;
        case 7:
           kd_back+=0.1;
            break;
        case 8:
             kd_back-=0.1;
            break;

        case 9:
            speed_high += 50;
            break;

        case 10:
            speed_high -= 50;
            break;

        case 11:
            speed_low += 20;
            break;

        case 12:
            speed_low -= 20;
            break;

        case 13:
            safe++;
            break;

        case 14:
             safe--;
           
            break;

        case 15:
           selct=1;
            break;
        case 16:
            selct=0;
            break;
        }
    }

    if (page == 1)
    {
       key_value = Key_detec();
    key_down = key_value & (key_value ^ key_old);
    key_old = key_value;
        switch (key_down)
        {
        case 9:
            if (light_sec[0] == 10)
            {
                light_sec[0] = 1;
            }
            else if (light_sec[0] == 1)

            {
                light_sec[0] = 10;
            }
            break;

        case 10:
            if (light_sec[1] ==10)
            {
                light_sec[1] = 2;
            }
            else if (light_sec[1] == 2)

            {
                light_sec[1] = 10;
            }

            break;

        case 11:
            if (light_sec[2] == 10)
            {
                light_sec[2] = 3;
            }
            else if (light_sec[2] == 3)

            {
                light_sec[2] = 10;
            }
            break;
        case 12:
            if (light_sec[3] == 10)
            {
                light_sec[3] = 4;
            }
            else if (light_sec[3] == 4)

            {
                light_sec[3] = 10;
            }
            break;
        case 13:
            if (light_sec[4] == 10)
            {
                light_sec[4] = 5;
            }
            else if (light_sec[4] == 5)

            {
                light_sec[4] = 10;
            }
            break;
        case 14:
            if (light_sec[5] == 10)
            {
                light_sec[5] = 6;
            }
            else if (light_sec[5] == 6)

            {
                light_sec[5] = 10;
            }
            break;
        case 15:
            if (light_sec[6] == 10)
            {
                light_sec[6] = 7;
            }
            else if (light_sec[6] == 7)

            {
                light_sec[6] = 10;
            }
            break;
        case 16:
            if (light_sec[7] == 10)
            {
                light_sec[7] = 8;
            }
            else if (light_sec[7] == 8)
            {
                light_sec[7] = 10;
            }
            break;

        case 2:
            if (page == 1)
            {
                page = 0;
                ips200_clear();
            }
            break;

        case 4:
            servo_pid.Kp += 0.1;
            break;
        case 5:
            servo_pid.Kp -= 0.1;
            break;
        case 6:
            servo_pid.Kd += 0.1;
            break;
        case 7:
            servo_pid.Kd -= 0.1;
            break;
            break;
        }
    }
}