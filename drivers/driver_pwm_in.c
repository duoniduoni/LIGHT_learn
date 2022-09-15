/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_pwm_in.c
 + 描述    ：PWM输入捕获驱动
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "rc.h"

/* 8通道输入PWM数据的数组1000-2000微秒的高电平信号 */
u16 Rc_Pwm_In[8];

/*----------------------------------------------------------
 + 实现功能：PWM输入信号捕获
 + 保留函数名，由中断调用
----------------------------------------------------------*/

/*
 * 2022.9.15
 * 前置信息补充：
 * 1. 一般情况下，PWM信号（遥控器中）为50Hz，即周期为20ms。
 * 2. 多旋翼机并没有使用PWM信号的占空比。
 * 3. 多旋翼机使用的是高电平持续时间（每个周期内），一般为1～2ms（1000～2000us）；这个就对应上了上面的注释。
 *      其中1100us对应0油门，1900us对应满油门。
 *
 * 总结记录：
 * 1. Rc_Pwm_In数组记录的是8个通道高电平持续时间
 * 2. 8个通道的高电平持续时间是通过检测GPIO_6/7/0/1/12/13/14/15这8个GPIO接口是否有值（高电平）来确定
 * 3. 持续时间通过2个计时器 TIM3/4 触发中断后，检测GPIO得到。
 * 4. TIM3 监测GPIO_6/7/0/1
 * 5. TIM4 监测GPIO_12/13/14/15
 * 6. 监测逻辑：
 *      每当计时器中断触发后，检查目标GPIO口是否处于高电平（有值）；如果有，则变量temp_ctnx记录时间戳；
 *      否则变量temp_ctnx_2记录时间戳。
 *
 *      出现低电平后，即计算PWM（即高电平）数据持续时间。
 *
 *      可能出现的情况：
 *          a. 先出现高电平，然后出现低电平：最理想情况。
 *          b. 一直处于低电平情况：每次都会计算PWM持续时间。
 *
 *      问题：
 *          a. TIM触发的周期，这个决定了计时精度。
 *          b. 接收器发送给飞控的信号形式是什么？没有遥控动作时PWM宽度时啥？猜测
 *              很有可能是没有动作的时候PWM宽度为1500；拉杆时PWM宽度为1500～2000,
 *              推杆时PWM宽度为1000～1500。
 *
 *              这个需要验证。
 *
 */

void TIM3_IRQHandler(void)
{
    /* PWM输入保存数据 */
    static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;

    /* 记录RadioControl类型为PWM输入方式 */
    Call_RadioControl_Sign(1);

    if(TIM3->SR & TIM_IT_CC1)
    {
        TIM3->SR = ~TIM_IT_CC1;
        TIM3->SR = ~TIM_FLAG_CC1OF;
        if(GPIOC->IDR & GPIO_Pin_6)
        {
            temp_cnt1 = TIM_GetCapture1(TIM3);
        }
        else
        {
            temp_cnt1_2 = TIM_GetCapture1(TIM3);
            if(temp_cnt1_2>=temp_cnt1)
                Rc_Pwm_In[0] = temp_cnt1_2-temp_cnt1;
            else
                Rc_Pwm_In[0] = 0xffff-temp_cnt1+temp_cnt1_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC2)
    {
        TIM3->SR = ~TIM_IT_CC2;
        TIM3->SR = ~TIM_FLAG_CC2OF;
        if(GPIOC->IDR & GPIO_Pin_7)
        {
            temp_cnt2 = TIM_GetCapture2(TIM3);
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2(TIM3);
            if(temp_cnt2_2>=temp_cnt2)
                Rc_Pwm_In[1] = temp_cnt2_2-temp_cnt2;
            else
                Rc_Pwm_In[1] = 0xffff-temp_cnt2+temp_cnt2_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC3)
    {
        TIM3->SR = ~TIM_IT_CC3;
        TIM3->SR = ~TIM_FLAG_CC3OF;
        if(GPIOB->IDR & GPIO_Pin_0)
        {
            temp_cnt3 = TIM_GetCapture3(TIM3);
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3(TIM3);
            if(temp_cnt3_2>=temp_cnt3)
                Rc_Pwm_In[2] = temp_cnt3_2-temp_cnt3;
            else
                Rc_Pwm_In[2] = 0xffff-temp_cnt3+temp_cnt3_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC4)
    {
        TIM3->SR = ~TIM_IT_CC4;
        TIM3->SR = ~TIM_FLAG_CC4OF;
        if(GPIOB->IDR & GPIO_Pin_1)
        {
            temp_cnt4 = TIM_GetCapture4(TIM3);
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4(TIM3);
            if(temp_cnt4_2>=temp_cnt4)
                Rc_Pwm_In[3] = temp_cnt4_2-temp_cnt4;
            else
                Rc_Pwm_In[3] = 0xffff-temp_cnt4+temp_cnt4_2+1;
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：PWM输入信号捕获
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void TIM4_IRQHandler(void)
{
    /* PWM输入保存数据 */
    static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;

    /* 记录RadioControl类型为PWM输入方式 */
    Call_RadioControl_Sign(1);

    if(TIM4->SR & TIM_IT_CC1)
    {
        TIM4->SR = ~TIM_IT_CC1;
        TIM4->SR = ~TIM_FLAG_CC1OF;
        if(GPIOD->IDR & GPIO_Pin_12)
        {
            temp_cnt1 = TIM_GetCapture1(TIM4);
        }
        else
        {
            temp_cnt1_2 = TIM_GetCapture1(TIM4);
            if(temp_cnt1_2>=temp_cnt1)
                Rc_Pwm_In[4] = temp_cnt1_2-temp_cnt1;
            else
                Rc_Pwm_In[4] = 0xffff-temp_cnt1+temp_cnt1_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC2)
    {
        TIM4->SR = ~TIM_IT_CC2;
        TIM4->SR = ~TIM_FLAG_CC2OF;
        if(GPIOD->IDR & GPIO_Pin_13)
        {
            temp_cnt2 = TIM_GetCapture2(TIM4);
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2(TIM4);
            if(temp_cnt2_2>=temp_cnt2)
                Rc_Pwm_In[5] = temp_cnt2_2-temp_cnt2;
            else
                Rc_Pwm_In[5] = 0xffff-temp_cnt2+temp_cnt2_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC3)
    {
        TIM4->SR = ~TIM_IT_CC3;
        TIM4->SR = ~TIM_FLAG_CC3OF;
        if(GPIOD->IDR & GPIO_Pin_14)
        {
            temp_cnt3 = TIM_GetCapture3(TIM4);
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3(TIM4);
            if(temp_cnt3_2>=temp_cnt3)
                Rc_Pwm_In[6] = temp_cnt3_2-temp_cnt3;
            else
                Rc_Pwm_In[6] = 0xffff-temp_cnt3+temp_cnt3_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC4)
    {
        TIM4->SR = ~TIM_IT_CC4;
        TIM4->SR = ~TIM_FLAG_CC4OF;
        if(GPIOD->IDR & GPIO_Pin_15)
        {
            temp_cnt4 = TIM_GetCapture4(TIM4);
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4(TIM4);
            if(temp_cnt4_2>=temp_cnt4)
                Rc_Pwm_In[7] = temp_cnt4_2-temp_cnt4;
            else
                Rc_Pwm_In[7] = 0xffff-temp_cnt4+temp_cnt4_2+1;
        }
    }
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
