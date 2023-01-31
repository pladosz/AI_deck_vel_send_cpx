/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck GAP8
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * WiFi image streamer example
 */
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#include "cpx.h"
#include "wifi.h"

static pi_task_t task1;

static struct pi_device camera;
static pi_buffer_t buffer;

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

// Performance menasuring variables
static uint32_t start = 0;
static uint32_t transferTime = 0;



static void capture_done_cb(void *arg)
{
  xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

typedef struct
{
  uint16_t Type;
  float vx;
  float vy;
  float vz;
  float yawrate;
} __attribute__((packed)) vel_message_t;


static CPXPacket_t txp;

void createVelMsgPacket(CPXPacket_t * packet) {
  vel_message_t *velMsg = (vel_message_t *) packet->data;
  velMsg->Type = 1;
  velMsg->vx = 0.1;
  velMsg->vy = 0.1;
  velMsg->vz = 0.1;
  velMsg->yawrate = 0;
  packet->dataLength = sizeof(vel_message_t);
}

void sendBufferViaCPX(CPXPacket_t * packet, uint8_t * buffer, uint32_t bufferSize) {
  uint32_t offset = 0;
  uint32_t size = 0;
  do {
    size = sizeof(packet->data);
    if (offset + size > bufferSize)
    {
      size = bufferSize - offset;
    }
    memcpy(packet->data, &buffer[offset], sizeof(packet->data));
    packet->dataLength = size;
    cpxSendPacketBlocking(packet);
    offset += size;
  } while (size == sizeof(packet->data));
}




void velocity_task(void *parameters) {
  //initalize CPX route
  cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_CRTP, &txp.route);
  //wait for carzyflie to initalize itself
  pi_time_wait_us(5000000);
  while (1) {
    // send velocity image
    cpxPrintToConsole(LOG_TO_CRTP, "attempting to send velocity !\n");
    createVelMsgPacket(&txp);
    cpxSendPacketBlocking(&txp);
    pi_time_wait_us(100000);
  }

}

void start_example(void)
{
    struct pi_uart_conf conf;
  struct pi_device device;
  pi_uart_conf_init(&conf);
  conf.baudrate_bps = 115200;

  pi_open_from_conf(&device, &conf);
  if (pi_uart_open(&device))
  {
    printf("[UART] open failed !\n");
    pmsis_exit(-1);
  }

  cpxInit();
  cpxEnableFunction(CPX_F_CRTP);

  evGroup = xEventGroupCreate();

  BaseType_t xTask;

  xTask = xTaskCreate(velocity_task, "vel_sending_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "velocity sending task didn't start !\n");
    pmsis_exit(-1);
  }
  while (1)
  {
    pi_yield();
  }
}


int main(void)
{
  pi_bsp_init();

  // Increase the FC freq to 250 MHz
  pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
  pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

  return pmsis_kickoff((void *)start_example);
}
