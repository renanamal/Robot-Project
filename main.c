/***************************************************************************//**
 * @file main.c
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/


//TODO - need to change the project debug setting and optimization level when
// switching to operational code


#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "stdbool.h"
#include <stddef.h>


#include "generalDefines.h"
#include "generalPurposeFunctions.h"
#include "motorControlStateMachine.h"
#include "callBacks.h"
#include "motorDriverMain.h"


//#include "tests/pwm/test_pwm.h"
//#include "tests/gpio_int/test_gpio_int.h"
//#include "tests/read_hulls/read_hulls.h"
//#include "tests/no_hulls/test_no_hulls.h"
//#include "tests/change_dir/testChangeDir.h"
//#include "tests/movingAverage/testMovingAverage.h"
//#include "tests/SPI/test_spi.h"


extern SMotorsData motors[NUM_OF_MOTORS];

int main(void)
{
  motorDriverPhaseConfigurationInit();

  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
//  app_init();


  // Initialize callbacks
  GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_PF9_PORT, SL_EMLIB_GPIO_INIT_PF9_PIN);
  setTimedCallBacksDB();
  init_callbacks_GPIO();
  counter_default_cfg( );
  delay_ms(300);



// Test functions for Debug
//  test_gpio_init();
//  test_pwm();
//  read_hulls();
//  test_motors_handle();
//  runMotorNoHulls(left);
//  init_test_callbacks_timed();
//  test_moving_average();
//  test_counter_spi();
//  test_arduino_spi();
//  test_counter_spi_once();


#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
      //motors[left].speedControler.refSpeed = 70.0;
//      for (int n =0; n < 1000000; n ++){
//          executeTimedFunctions();
//      }

      float real_speeds[15] = {0};
      float counts_encouder[15] = {0};
      float frequencies[1] = {20.0}; // Hz
      float speeds[1] = {63.0}; // Rad/Sec
      uint64_t window_size = 5000000; // 5 seconds


      for (int j=0; j<1; j++){
          // set frequency here !


          setChangeDir(frequencies[j]);
          for (int i = 0; i< 1; i++){
              motors[left].speedControler.refSpeed = speeds[i];

              uint64_t start_time = getuSec();
              uint64_t curr_time = getuSec();

              while (curr_time < start_time + window_size){
                   executeTimedFunctions();
                   curr_time = getuSec();
              }

          real_speeds[3*j+i]=motors[left].speedControler.speedFromEncoder;
          counts_encouder[3*j+i]=motors[left].speedControler.lastEncoderCnt;
          }
      }

      int x = 2;


    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
//    sl_system_process_action();

    // Application process.
//    app_process_action();

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}
