#pragma once
#include <Arduino.h>

// STM32F401RC only. PA8=TIM1_CH1=R, PA9=TIM1_CH2=L.
// Functions below require a critical section when called after setup.
// TIM2 sampling, encoder pins and PB6/PB7 are untouched.
namespace motor_hardware {
static bool initialized=false;
constexpr uint32_t EN_MASK=(1u<<8)|(1u<<9);
constexpr uint32_t EN_MODE_MASK=(3u<<16)|(3u<<18);
constexpr uint32_t DIR_MASK=0xf000u;

inline void off() {
  if(initialized) {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1=0; TIM1->CCR2=0;
  }
  // Latch LOW before switching from PWM AF to push-pull output.
  GPIOA->BSRR=EN_MASK<<16;
  GPIOA->MODER=(GPIOA->MODER & ~EN_MODE_MASK)|(1u<<16)|(1u<<18);
  GPIOB->BSRR=DIR_MASK<<16;
}

inline void begin() {
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM1_FORCE_RESET(); __HAL_RCC_TIM1_RELEASE_RESET();
  GPIOA->OTYPER &= ~EN_MASK;
  GPIOA->PUPDR=(GPIOA->PUPDR & ~EN_MODE_MASK)|(2u<<16)|(2u<<18);
  GPIOA->AFR[1]=(GPIOA->AFR[1]&~0xffu)|0x11u; // AF1, PA8 and PA9 only
  uint32_t timerClock=HAL_RCC_GetPCLK2Freq();
  if((RCC->CFGR&RCC_CFGR_PPRE2)!=0) timerClock*=2;
  TIM1->PSC=timerClock/1000000u-1u;
  TIM1->ARR=999; // 1 kHz, same nominal frequency as original Arduino PWM
  TIM1->CCR1=0; TIM1->CCR2=0;
  TIM1->CCMR1=(6u<<4)|TIM_CCMR1_OC1PE|(6u<<12)|TIM_CCMR1_OC2PE;
  TIM1->CCER=TIM_CCER_CC1E|TIM_CCER_CC2E;
  TIM1->CR2=0; // inactive LOW on advanced timer outputs
  TIM1->BDTR=TIM_BDTR_OSSI|TIM_BDTR_OSSR;
  TIM1->EGR=TIM_EGR_UG;
  TIM1->SR=0;
  TIM1->CR1=TIM_CR1_ARPE|TIM_CR1_CEN;
  initialized=true;
  off();
}

inline void pulse(int left,int right) {
  off();
  // Original convention: forward=(u>0)!=inv, invL=false, invR=true.
  uint32_t high=0;
  if(left) high |= left>0 ? (1u<<14) : (1u<<15);
  if(right) high |= right<0 ? (1u<<12) : (1u<<13);
  GPIOB->BSRR=((DIR_MASK & ~high)<<16)|high;
  TIM1->CCR1=(uint32_t)(right<0?-right:right)*10u;
  TIM1->CCR2=(uint32_t)(left<0?-left:left)*10u;
  TIM1->EGR=TIM_EGR_UG;
  TIM1->SR=0;
  GPIOA->MODER=(GPIOA->MODER&~EN_MODE_MASK)|(2u<<16)|(2u<<18);
  TIM1->BDTR |= TIM_BDTR_MOE;
}
}
