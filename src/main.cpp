// temporary bring-up main (replaced later)
#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include "hardware/I2CBus.h"
#include "hardware/IMUDriver.h"
#include "hardware/TouchDriver.h"
#include "hardware/DisplayDriver.h"
#include "hardware/BatteryManager.h"
#include "measurement/MeasurementService.h"
#include "system/Log.h"
#include "BoardConfig.h"
static I2CBus bus; static QMI8658Driver imu(bus); static TouchDriver touch(bus); static DisplayDriver disp; static MeasurementService ms;
void setup(){ sda_log::begin(sda_log::Level::Info); bus.begin(0,board::kI2cSda,board::kI2cScl,board::kI2cFreqHz); imu.begin(); touch.begin(); disp.begin(0); disp.initLvgl(&touch);
 ms.begin(&imu,&bus,sda::PipelineConfig(),sda::SensorCalibration(),sda::MountModel()); }
void loop(){ lv_timer_handler(); delay(5);}
