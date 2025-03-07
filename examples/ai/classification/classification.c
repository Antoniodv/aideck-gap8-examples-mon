/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck examples
 *
 * Copyright (C) 2021 Bitcraze AB
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
 * @file classification.c
 *
 *  
 */

#include "classification.h"
#include "bsp/camera/himax.h"
#include "bsp/transport/nina_w10.h"
#include "classificationKernels.h"
#include "gaplib/ImgIO.h"
#include "pmsis.h"
#include "stdio.h"
#include "bsp/bsp.h"
#include "cpx.h"
// #include "perf.h"
// #include "timer.h"

#define CAM_WIDTH 324
#define CAM_HEIGHT 244

#define CHANNELS 1
#define IO RGB888_IO
#define CAT_LEN sizeof(uint32_t)

#define __XSTR(__s) __STR(__s)
#define __STR(__s) #__s

static pi_task_t task1;
static pi_task_t task2;
static unsigned char *cameraBuffer;
// static unsigned char *imageDemosaiced;
// static signed char *imageCropped;
static signed short *Output_1;
static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;

AT_HYPERFLASH_FS_EXT_ADDR_TYPE __PREFIX(_L3_Flash) = 0;

#define IMG_ORIENTATION 0x0101

/* Define global vars for perf counters */
volatile uint64_t cycles =        0;     
volatile uint64_t imiss  =        0;
volatile uint64_t ld_ext =        0;
volatile uint64_t st_ext =        0;
volatile uint64_t tcdm_cont =     0;
volatile uint64_t perf_instr=     0;
volatile uint64_t active_cycles = 0;
volatile uint64_t ld_stall =      0;
volatile uint64_t jr_stall =      0;
volatile uint64_t perf_branch =   0;

static void RunNetwork()
{
  // __PREFIX(CNN)
  // (cameraBuffer, Output_1);
}

static void cam_handler(void *arg)
{
  // pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

  // /* Run inference */
  // pi_cluster_send_task_to_cl(&cluster_dev, task);

  // if (Output_1[0] > Output_1[1])
  // {
  //   cpxPrintToConsole(LOG_TO_CRTP, "Packet,     confidence: %hd\n", Output_1[0] - Output_1[1]);
  // }
  // else
  // {
  //   cpxPrintToConsole(LOG_TO_CRTP, "Background, confidence: %hd\n", Output_1[1] - Output_1[0]);
  // }

  // pi_camera_capture_async(&camera, cameraBuffer, CAM_WIDTH * CAM_HEIGHT, pi_task_callback(&task1, cam_handler, NULL));
  // pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
}

static int open_camera(struct pi_device *device)
{

  // struct pi_himax_conf cam_conf;

  // pi_himax_conf_init(&cam_conf);

  // cam_conf.format = PI_CAMERA_QVGA;

  // pi_open_from_conf(device, &cam_conf);
  // if (pi_camera_open(device))
  //   return -1;

  // pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
  // uint8_t set_value = 3;
  // uint8_t reg_value;
  // pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
  // pi_time_wait_us(1000000);
  // pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);

  // if (set_value != reg_value)
  // {
  //   cpxPrintToConsole(LOG_TO_CRTP,"Failed to rotate camera image\n");
  //   return -1;
  // }
              
  // pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

  // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
  // return 0;
}

// Functions and init for LED toggle
#define LED_PIN 2
static pi_device_t led_gpio_dev;

void hb_task(void *parameters)
{
  // (void)parameters;
  // char *taskname = pcTaskGetName(NULL);

  // // Initialize the LED pin
  // pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);

  // const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

  // while (1)
  // {
  //   pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
  //   vTaskDelay(xDelay);
  //   pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
  //   vTaskDelay(xDelay);
  // }
}

int classification()
{
  cpxPrintToConsole(LOG_TO_CRTP,"classification");
/*	
  pi_freq_set(PI_FREQ_DOMAIN_FC, FREQ_FC*1000*1000);
  //pi_freq_set(PI_FREQ_DOMAIN_CL, FREQ_CL*1000*1000);
  __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

  // For debugging
  struct pi_uart_conf uart_conf;
  struct pi_device device;
  pi_uart_conf_init(&uart_conf);
  uart_conf.baudrate_bps = 115200;

    // Start LED toggle
  BaseType_t xTask;
  xTask = xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);

  pi_open_from_conf(&device, &uart_conf);
  if (pi_uart_open(&device))
  {
    printf("[UART] open failed !\n");
    pmsis_exit(-1);
  }

  cpxInit();
  cpxEnableFunction(CPX_F_WIFI_CTRL);

  cpxPrintToConsole(LOG_TO_CRTP, "*** Classification ***\n");

  cpxPrintToConsole(LOG_TO_CRTP, "Starting to open camera\n");

  if (open_camera(&camera))
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
    return -1;
  }
  cpxPrintToConsole(LOG_TO_CRTP,"Opened Camera\n");

  cameraBuffer = (unsigned char *)pmsis_l2_malloc((CAM_WIDTH * CAM_HEIGHT) * sizeof(unsigned char));
  if (cameraBuffer == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed Allocated memory for camera buffer\n");
    return 1;
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for camera buffer\n");

  Output_1 = (signed short *)pmsis_l2_malloc(2 * sizeof(signed short));
  if (Output_1 == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for output\n");
    pmsis_exit(-1);
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for output\n");

  // Configure CNN task 
  pi_cluster_conf_init(&cluster_conf);
  pi_open_from_conf(&cluster_dev, (void *)&cluster_conf);
  pi_cluster_open(&cluster_dev);
  task = pmsis_l2_malloc(sizeof(struct pi_cluster_task));
  if (!task)
  {  
    cpxPrintToConsole(LOG_TO_CRTP, "failed to allocate memory for task\n");
  }
  cpxPrintToConsole(LOG_TO_CRTP,"Allocated memory for task\n");

  memset(task, 0, sizeof(struct pi_cluster_task));
  task->entry = &RunNetwork;
  task->stack_size = STACK_SIZE;             // defined in makefile
  task->slave_stack_size = SLAVE_STACK_SIZE; // "
  task->arg = NULL;

  // Construct CNN 
  int ret = __PREFIX(CNN_Construct)();
  if (ret)
  {
    cpxPrintToConsole(LOG_TO_CRTP,"Failed to construct CNN with %d\n", ret);
    pmsis_exit(-5);
  }
  cpxPrintToConsole(LOG_TO_CRTP,"Constructed CNN\n");

  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
  pi_camera_capture_async(&camera, cameraBuffer, CAM_WIDTH * CAM_HEIGHT, pi_task_callback(&task1, cam_handler, NULL));
  pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

  while (1)
  {
    pi_yield();
  }

  // Destruct CNN 
  __PREFIX(CNN_Destruct)
  ();

  return 0;
  */
}

void vTimerCallbackMonitor(TimerHandle_t xTimer){
    cpxPrintToConsole(LOG_TO_CRTP, "p.c. read\n");	
    // pi_perf_stop();
    // cycles =        pi_perf_read(PI_PERF_CYCLES); 
    // imiss  =        pi_perf_read(PI_PERF_IMISS); 
    // ld_ext =        pi_perf_read(PI_PERF_LD_EXT); 
    // st_ext =        pi_perf_read(PI_PERF_ST_EXT); 
    // tcdm_cont =     pi_perf_read(PI_PERF_TCDM_CONT); 
    // perf_instr =    pi_perf_read(PI_PERF_INSTR); 
    // active_cycles = pi_perf_read(PI_PERF_ACTIVE_CYCLES); 
    // ld_stall =      pi_perf_read(PI_PERF_LD_STALL); 
    // jr_stall =      pi_perf_read(PI_PERF_JR_STALL); 
    // perf_branch =   pi_perf_read(PI_PERF_BRANCH);   

    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_CYCLES: %d\n", cycles);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_IMISS: %d\n" , imiss);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_LD_EXT: %d\n", ld_ext);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_ST_EXT: %d\n", st_ext);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_TCDM_CONT: %d\n", tcdm_cont);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_INSTR: %d\n", perf_instr);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_ACTIVE_CYCLES: %d\n", active_cycles);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_LD_STALL: %d\n", ld_stall);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_JR_STALL: %d\n", jr_stall);
    // cpxPrintToConsole(LOG_TO_CRTP,"PI_PERF_BRANCH: %d\n", perf_branch);

    // pi_perf_start();
}

void init_monitor(void){
    cpxPrintToConsole(LOG_TO_CRTP, "perf counters init\n");
    // pi_perf_stop(); 
    // pi_perf_conf(1<<PI_PERF_CYCLES);     
    // pi_perf_conf(1<<PI_PERF_IMISS);      
    // pi_perf_conf(1<<PI_PERF_LD_EXT);   
    // pi_perf_conf(1<<PI_PERF_ST_EXT);    
    // pi_perf_conf(1<<PI_PERF_TCDM_CONT);   
    // pi_perf_conf(1<<PI_PERF_INSTR);     
    // pi_perf_conf(1<<PI_PERF_ACTIVE_CYCLES);    
    // pi_perf_conf(1<<PI_PERF_LD_STALL);    
    // pi_perf_conf(1<<PI_PERF_JR_STALL);   
    // pi_perf_conf(1<<PI_PERF_BRANCH);                        
    // pi_perf_reset(); 
    // pi_perf_start(); 
}

int main(void){
    /* Init HW */
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);
    cpxPrintToConsole(LOG_TO_CRTP, "*** Classification with Performance Monitoring ***\n");
    
    while(1){
      cpxPrintToConsole(LOG_TO_CRTP, "main\n");
    }
    // /* Create FreeRTOS SW timer */
    // TimerHandle_t xMonitorTimer = xTimerCreate(
    //     "Timer",
    //     pdMS_TO_TICKS(1000),
    //     pdTRUE,
    //     ( void * )0,
    //     vTimerCallbackMonitor
    // );

    // /* Start timer */
    // if(xMonitorTimer != 0){
    //     xTimerStart( xMonitorTimer, 0 );
    // }

    // init_monitor();
    // return pmsis_kickoff((void *)classification);
}
