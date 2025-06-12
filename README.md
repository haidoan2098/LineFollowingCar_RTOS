# Line Following PID and Stair-Climbing Robot Car using FreeRTOS.
## RTOS Task description.    
```cpp
void taskMainLine(void *pvParameters);      // Thực hiện nhiệm vụ dò line 
void taskClimbStairs(void *pvParameters);   // Sử dụng HCSR04 để đo khoảng cách, gần bậc thang thì sẽ chuyển sang MODE_CLIMB_STAIR.

/*-Vận chuyển bóng và thả bóng*/
void taskBallDetection(void *pvParameters);
void taskPassingBall(void *pvParameters);
```
[Link Line Following Car](https://www.tiktok.com/@haidoan2098/video/7513945389721259271?is_from_webapp=1&sender_device=pc)

## Hardware       
- ESP32 Devkit V1.    
- Module TCRT5000.
- HCSR-04 Ultrasonic Sensor.
- L298N.
- Motor.
- Infrared Sensor.
- Servo.

## Software    
- VSCode + PlatformIO.
