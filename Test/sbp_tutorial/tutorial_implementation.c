/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * tutorial_implementation contains functions and definitions that are implementation
 * specific to this tutorial, to keep main.c as simple as possible.
 */

#include <stm32f4xx_gpio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>

#include <tutorial_implementation.h>

/*
 * FIFO to hold received UART bytes before libsbp parses them.
 */
#define FIFO_LEN 512
char sbp_msg_fifo[FIFO_LEN];
u16 head = 0;
u16 tail = 0;

/* Return 1 if true, 0 otherwise. */
u8 fifo_empty(void){
  if (head == tail)
    return 1;
  return 0;
}

/*
 * Append a character to our SBP message fifo.
 * Returns 1 if char successfully appended to fifo.
 * Returns 0 if fifo is full.
 */
u8 fifo_write(char c){
  if (fifo_full())
    return 0;

  sbp_msg_fifo[tail] = c;
  tail = (tail+1) % FIFO_LEN;
  return 1;
}

/*
 * Read 1 char from fifo.
 * Returns 0 if fifo is empty, otherwise 1.
 */
u8 fifo_read_char(char *c) {
  if (fifo_empty())
    return 0;

  *c = sbp_msg_fifo[head];
  head = (head+1) % FIFO_LEN;
  return 1;
}

/*
 * Read arbitrary number of chars from FIFO. Must conform to
 * function definition that is passed to the function
 * sbp_process().
 * Returns the number of characters successfully read.
 */
u32 fifo_read(u8 *buff, u32 n, void *context) {
  int i;
  for (i=0; i<n; i++)
    if (!fifo_read_char((char *)(buff + i)))
      break;
  return i;
}

/* Return 1 if true, 0 otherwise. */
u8 fifo_full(void){
  if (((tail+1)%FIFO_LEN) == head) {
    return 1;
  }
  return 0;
}

void USART1_IRQHandler(void)
{
  fifo_write(USART1->DR);
  DO_EVERY(250,
    leds_toggle();
  );
  USART1->SR &= ~(USART_FLAG_RXNE);
}


void usarts_setup(void){

  /* USART1 to Piksi. */
  GPIO_InitTypeDef GPIOA_InitStructure;
  USART_InitTypeDef USART1_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable peripheral clock for USART1. */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART1 TX on PA9 */
  /* GPIOA Configuration:  USART1 RX on PA10 */
  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIOA_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIOA_InitStructure);

  USART1_InitStructure.USART_BaudRate = 115200;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART1_InitStructure.USART_Mode = USART_Mode_Rx;
  USART_Init(USART1, &USART1_InitStructure);

  /* Enable the USART RX Interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USART1, ENABLE);
}

void leds_set(void){
  GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

void leds_unset(void){
  GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

void leds_toggle(void){
  if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_12))
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
  else
    GPIO_SetBits(GPIOD, GPIO_Pin_12);

  if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_13))
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
  else
    GPIO_SetBits(GPIOD, GPIO_Pin_13);

  if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_14))
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
  else
    GPIO_SetBits(GPIOD, GPIO_Pin_14);

  if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_15))
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);
  else
    GPIO_SetBits(GPIOD, GPIO_Pin_15);
}

void leds_setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
