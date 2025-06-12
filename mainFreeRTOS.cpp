#include <Arduino.h>
#include <ESP32Servo.h>

// ================= PID ====================
#define ENA 27
#define ENB 14

#define MOTOR_L_1 32 // IN1 (+)
#define MOTOR_L_2 33 // IN2 (-)
#define MOTOR_R_1 25 // IN3 (-)
#define MOTOR_R_2 26 // IN4 (+)

// LINE 1 -> 5
const uint8_t SENSORS_PIN[] = {34, 18, 19, 35, 23};

float Kp = 7.0;     // 7.8222222
float Ki = 0.0007;  // 0.085555
float Kd = 3.5;     // 3.45555

float P;
float I;
float D;

int PIDValue = 0;
int error = 0;
int lastError = 0;
bool isRunning = true;
bool stoped = false;

const uint16_t maxSpeed = 235;

uint16_t outLineSpeed = 182;
uint16_t lineSpeed = 187;   // 185;
uint16_t climbingSpeed = 210;

bool flagTaskClimbStair = false;

int checkStopCar = 0;
uint8_t countStair = 0;     // Số lượng bậc

bool checkModeBallTransport = false;

bool checkOutLine = false;

const int freq = 30000;
const uint8_t ENAChanel = 2; // 0
const uint8_t ENBChanel = 3; // 1
const uint16_t resolution = 8;

void caculate_pid();
void motor_control(uint16_t currentSpeed);
void read_sensors();
// ================= END PID ====================

// -- Ultrasonic sensor
#define TRIG 2
#define ECHO 4
#define N 4
float distanceArray[N] = {0};
char idx = 0;
float sum = 0;
float getDistance();
float movingAverageFilter(float newValue);
// -----------------

// -- IR sensor & Servo
#define IR_DETECTION_BALL_PIN 15
#define IR_OPEN_DOOR_PIN 17
#define SERVO_DOOR_CONTROL_PIN 21

Servo sg90;
// -----------------

// -- State Car
enum RobotMode
{
    MODE_LINE_FOLLOWING, // Chế độ dò line
    MODE_CLIMB_STAIR,    // Chế độ leo bậc
    MODE_BALL_TRANSPORT  // Chế độ vận chuyển bóng
};

RobotMode currentMode = MODE_LINE_FOLLOWING;
// -----------------------------------------------

// ================= RTOS ====================
TaskHandle_t xTaskMainLine = NULL;
TaskHandle_t xTaskClimbStairs = NULL;
TaskHandle_t xTaskBallDetection = NULL;
TaskHandle_t xTaskPassingBall = NULL;

SemaphoreHandle_t xMutex = NULL;

/* Task Line */
void taskMainLine(void *pvParameters)
{
    mode_t mode;
    static unsigned long ballTransportStartTime = 0;
    static bool inBallTransport = false;

    while (1)
    {
        while (isRunning)
        {
            unsigned long startTime = micros();

            read_sensors();
            caculate_pid();

            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                mode = currentMode;
                xSemaphoreGive(xMutex);
            }

            // Serial.println(mode);

            switch (mode)
            {
            case MODE_LINE_FOLLOWING:
                motor_control(lineSpeed);
                break;

            case MODE_CLIMB_STAIR:
                // Đâm thẳng
                digitalWrite(MOTOR_R_1, HIGH);
                digitalWrite(MOTOR_R_2, LOW);
                digitalWrite(MOTOR_L_1, HIGH);
                digitalWrite(MOTOR_L_2, LOW);
                ledcWrite(ENAChanel, climbingSpeed);
                ledcWrite(ENBChanel, climbingSpeed);
                break;

            case MODE_BALL_TRANSPORT:
                if (!inBallTransport)
                {
                    ballTransportStartTime = millis();
                    inBallTransport = true;
                }

                // Đi thẳng đoạn ngắn để ra khỏi line dừng
                digitalWrite(MOTOR_R_1, HIGH);
                digitalWrite(MOTOR_R_2, LOW);
                digitalWrite(MOTOR_L_1, HIGH);
                digitalWrite(MOTOR_L_2, LOW);
                ledcWrite(ENAChanel, outLineSpeed);
                ledcWrite(ENBChanel, outLineSpeed);

                if (millis() - ballTransportStartTime >= 300)
                {
                    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                    {
                        currentMode = MODE_LINE_FOLLOWING; // Cập nhật biến toàn cục
                        xSemaphoreGive(xMutex);
                    }
                    inBallTransport = false;
                    checkOutLine = true;
                }
                break;
            }

            vTaskDelay(15 / portTICK_PERIOD_MS);
        }

        // Lùi lại tránh đi quá line
        if (stoped == false)
        {
            digitalWrite(MOTOR_L_1, LOW);
            digitalWrite(MOTOR_L_2, HIGH);
            digitalWrite(MOTOR_R_1, LOW);
            digitalWrite(MOTOR_R_2, HIGH);
            ledcWrite(ENAChanel, 180);
            ledcWrite(ENBChanel, 180);
            vTaskDelay(130 / portTICK_PERIOD_MS);
            stoped = true;
        }

        digitalWrite(MOTOR_L_1, LOW);
        digitalWrite(MOTOR_L_2, LOW);
        digitalWrite(MOTOR_R_1, LOW);
        digitalWrite(MOTOR_R_2, LOW);
        ledcWrite(ENAChanel, 0);
        ledcWrite(ENBChanel, 0);

    }
}

/* Task Climb Stairs using Ultrasonic Sensor */
void taskClimbStairs(void *pvParameters)
{
    bool stair_detected = false;
    uint8_t extendTime = 0;

    // Khởi tạo mảng với giá trị đo đầu tiên
    float firstDistance = getDistance();
    if (firstDistance == -1)
    {
        firstDistance = 0; // Hoặc giá trị mặc định hợp lý
    }
    for (int i = 0; i < N; i++)
    {
        distanceArray[i] = firstDistance;
        sum += firstDistance;
    }

    while (1)
    {
        float rawDistance = getDistance(); // Đọc khoảng cách thực tế

        if (rawDistance == -1)
        {
            rawDistance = 25;
        }

        float filteredDistance = movingAverageFilter(rawDistance); // Lọc dữ liệu
        Serial.println(filteredDistance);

        // Đã phát hiện bậc thang
        if ((filteredDistance >= 2 && filteredDistance <= 15) && (currentMode == MODE_LINE_FOLLOWING))
        {
            stair_detected = true; // đánh dấu đã phát hiện bậc thang
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                currentMode = MODE_CLIMB_STAIR; // Chuyển sang mode Leo bậc
                xSemaphoreGive(xMutex);
            }
        }

        // Bánh đã lên bậc
        if (stair_detected && (filteredDistance >= 20 && filteredDistance <= 100))
        {
            extendTime++;
            if (extendTime >= 5) // Kéo dài thời gian leo bậc
            {
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                {
                    currentMode = MODE_LINE_FOLLOWING; // Chuyển về mode Dò line
                    xSemaphoreGive(xMutex);
                }

                flagTaskClimbStair = true; // Bật cờ đã leo bâc
                stair_detected = false;    // Kết thúc đánh dấu, coi như lên bậc xong
                countStair++;              // Tăng số lượng bậc đã leo lên
            }

            // Hoàn thành xong task leo bậc
            if (countStair >= 5)
            {
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                {
                    currentMode = MODE_LINE_FOLLOWING; // Chuyển về mode Dò line
                    xSemaphoreGive(xMutex);
                }

                flagTaskClimbStair = false;
                // Xóa Task leo bậc
                vTaskDelete(xTaskClimbStairs);
            }
        }

        // Chưa leo đủ 5 bậc thì chưa được dừng
        if ((countStair < 5) && !isRunning)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                isRunning = true; // Cho phép chạy lại
                currentMode = MODE_LINE_FOLLOWING;
                xSemaphoreGive(xMutex);
            }
        }

        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

/* Task Ball Detection using IR Sensor */
void taskBallDetection(void *pvParameters)
{
    bool checkCompleteDropBall = false; 
    bool lastIRState = HIGH;            // Trạng thái trước đó của cảm biến (giả sử ban đầu không có vật)
    bool isBlocked = false;
    unsigned long blockStartTime = 0;
    bool flagTaskResume = true;

    while (1)
    {
        bool currentIRState = digitalRead(IR_DETECTION_BALL_PIN);

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            if (isRunning == false) // Nếu xe đang dừng mới xử lý
            {
                // Bắt đầu hạ cần bỏ bóng, không có vật (HIGH) → có vật (LOW)
                if (lastIRState == HIGH && currentIRState == LOW)
                {
                    blockStartTime = millis();
                    isBlocked = true;
                }

                // Kết thúc bị che
                if (lastIRState == LOW && currentIRState == HIGH)
                {
                    if (isBlocked)
                    {
                        unsigned long duration = millis() - blockStartTime;
                        if (duration >= 1500)
                        {
                            vTaskDelay(1000 / portTICK_PERIOD_MS); // Chờ 2s rồi xe chạy
                            checkCompleteDropBall = true;          // Đánh dấu thả bóng xong
                            isRunning = true;                      // Vô dò line
                            currentMode = MODE_BALL_TRANSPORT;     // Cho xe chạy đoạn
                        }
                        isBlocked = false; // Reset trạng thái
                    }
                }
            }
            xSemaphoreGive(xMutex);
        }

        lastIRState = currentIRState; // Cập nhật trạng thái cũ

        // Xử lý trường hợp mở Servo để thả bóng xuống
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            if (checkCompleteDropBall && checkOutLine && (isRunning == false))
            {
                if (flagTaskResume)
                {
                    vTaskResume(xTaskPassingBall); // Bắt đầu chạy task
                    flagTaskResume = false;
                    checkCompleteDropBall = false;
                }
            }
            xSemaphoreGive(xMutex);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/* Task Passing the Ball */
void taskPassingBall(void *pvParameters)
{
    bool doorOpened = false;
    int lengthenTimeDetection = 0;

    sg90.attach(SERVO_DOOR_CONTROL_PIN, 500, 2400); 

    while (1)
    {
        bool detected = digitalRead(IR_OPEN_DOOR_PIN) == LOW; // LOW = có vật cản

        if (!doorOpened)
        {
            if (detected)
            {
                lengthenTimeDetection++;
                if (lengthenTimeDetection >= 30) // khoảng 3s nếu delay là 100ms
                {
                    for (int pos = 0; pos <= 160; pos += 4)
                    {
                        sg90.write(pos);
                        delay(10);
                    }
                    doorOpened = true;
                    lengthenTimeDetection = 0;
                }
            }
            else
            {
                if (lengthenTimeDetection > 0)
                {
                    lengthenTimeDetection = 0;
                }
            }
        }
        else
        {
            if (!detected)
            {
                for (int pos = 160; pos >= 0; pos -= 4)
                {
                    sg90.write(pos);
                    delay(10);
                }
                doorOpened = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// ================= END RTOS ====================

void setup()
{
    Serial.begin(9600);

    // -- PID
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);

    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);

    ledcSetup(ENAChanel, freq, resolution);
    ledcSetup(ENBChanel, freq, resolution);
    ledcAttachPin(ENA, ENAChanel);
    ledcAttachPin(ENB, ENBChanel);

    for (auto sensorPin : SENSORS_PIN)
    {
        pinMode(sensorPin, INPUT);
    }
    // ---------------------------------

    // -- Ultrasonic & IR sensor
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    pinMode(IR_DETECTION_BALL_PIN, INPUT);
    pinMode(IR_OPEN_DOOR_PIN, INPUT);

    sg90.setPeriodHertz(50);                        // Tần số PWM cho SG90
    sg90.attach(SERVO_DOOR_CONTROL_PIN, 500, 2400); // Điều chỉnh góc từ 0° đến 180°
    sg90.write(0);                                  // Khởi đầu ở vị trí đóng
    sg90.detach();                                  // Ngắt servo để tiết kiệm tài nguyên
    // --------------------------

    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL)
    {
        Serial.println("Failed to create mutex");
        return;
    }

    // Tạo task FreeRTOS
    xTaskCreatePinnedToCore(taskMainLine, "Task Main Line", 4098, NULL, 3, &xTaskMainLine, 1);
    xTaskCreatePinnedToCore(taskClimbStairs, "Task CLimb Stairs", 1024, NULL, 3, &xTaskClimbStairs, 0);
    xTaskCreatePinnedToCore(taskBallDetection, "Task Ball Detection", 1024, NULL, 2, &xTaskBallDetection, 0);
    xTaskCreatePinnedToCore(taskPassingBall, "Task Passing the Ball", 1024, NULL, 1, &xTaskPassingBall, 0);

    vTaskSuspend(xTaskPassingBall); // chưa dùng ngay
    vTaskStartScheduler();
}

void loop()
{
}

void read_sensors()
{
    String sensorArray = "";
    for (auto sensorPin : SENSORS_PIN)
    {
        sensorArray += (char)(digitalRead(sensorPin) + 48);
    }

    // Tính toán lỗi dựa trên 5 cảm biến
    if ((sensorArray == "10000"))       // Line lệch phải nhiều
        error = 6; 
    else if ((sensorArray == "11000"))  // Line lệch phải nhiều
        error = 5; 
    else if (sensorArray == "11110")    // Line lệch phải nhiều
        error = 4; 
    else if (sensorArray == "11100")    // Line lệch phải
        error = 3;                   
    else if (sensorArray == "11101")    // Line hơi lệch phải nhiều
        error = 2;                   
    else if (sensorArray == "11001")    // Line hơi lệch phải
        error = 1;                   
    else if (sensorArray == "11011")    // Line ở giữa
        error = 0;                   
    else if (sensorArray == "10011")    // Line hơi lệch trái
        error = -1;                  
    else if (sensorArray == "10111")    // Line hơi lệch trái nhiều
        error = -2;                  
    else if (sensorArray == "00111")    // Line lệch trái
        error = -3; 
    else if (sensorArray == "01111")    // Line lệch trái nhiều
        error = -4; 
    else if ((sensorArray == "00011"))  // Line lệch trái nhiều
        error = -5; 
    else if ((sensorArray == "00001"))  // Line lệch trái nhiều
        error = -6; 
    else if (sensorArray == "11111")    // Mất Line
    {
        if (error > 0)
            error = 7; // Lệch trái hoàn toàn, ko còn line nào bắt đc
        else if (error < 0)
            error = -7; // Lệch phải hoàn toàn, ko còn line nào bắt đc
    }
    else if (sensorArray == "01110" || sensorArray == "01010")
    {
        isRunning = false; // Điều kiện dừng
    }
    else if (sensorArray == "00000")
    { // Trường hợp bắt đc điểm kết thúc line hoặc mất line
        if (currentMode == MODE_CLIMB_STAIR)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                isRunning = true;
                xSemaphoreGive(xMutex);
            }
        }
        else if (currentMode == MODE_LINE_FOLLOWING)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                isRunning = false;
                xSemaphoreGive(xMutex);
            }
        }
    }
}

void caculate_pid()
{
    P = error;
    float dynamicKp = Kp;
    float dynamicKi = Ki;

    // Giới hạn tích lũy lỗi để tránh vượt mức ổn định
    if (abs(error) < 5)
    {
        I += Ki * error;
        I = constrain(I, -15, 15); // Giới hạn giá trị I để tránh dao động quá mức
    }
    else
    {
        I = 0; // Reset I nếu lỗi quá lớn
    }

    D = error - lastError;

    // Tính toán PID
    PIDValue = dynamicKp * P + I + Kd * D;

    // Giữ PID trong khoảng -30 đến 30
    PIDValue = constrain(PIDValue, -40, 40); // Giảm giới hạn PIDValue

    // Serial.println(PIDValue);

    lastError = error;
}

void set_motor(uint16_t speedA, uint16_t speedB)
{
    // Xoay xe sang phải (bánh trái tiến, bánh phải lùi) khi lỗi lớn
    if (error >= 5)
    {
        digitalWrite(MOTOR_R_1, LOW);
        digitalWrite(MOTOR_R_2, HIGH); // Bánh phải lùi
        digitalWrite(MOTOR_L_1, HIGH);
        digitalWrite(MOTOR_L_2, LOW);
        speedA = maxSpeed + 5; // Tốc độ bánh trái
        speedB = maxSpeed; // Tốc độ bánh phải (lùi)
    }
    else if (error <= -5)
    { // Xoay xe sang trái (bánh phải tiến, bánh trái lùi) khi lỗi lớn
        digitalWrite(MOTOR_L_1, LOW);
        digitalWrite(MOTOR_L_2, HIGH); // Bánh trái lùi
        digitalWrite(MOTOR_R_1, HIGH);
        digitalWrite(MOTOR_R_2, LOW); // Bánh phải tiến
        speedA = maxSpeed;            // Tốc độ bánh trái (lùi)
        speedB = maxSpeed + 5;            // Tốc độ bánh phải
    }
    else
    {
        digitalWrite(MOTOR_R_1, HIGH);
        digitalWrite(MOTOR_R_2, LOW);
        digitalWrite(MOTOR_L_1, HIGH);
        digitalWrite(MOTOR_L_2, LOW);
    }

    speedA = constrain(speedA, 0, maxSpeed);
    speedB = constrain(speedB, 0, maxSpeed);

    ledcWrite(ENAChanel, speedA);
    ledcWrite(ENBChanel, speedB);
}

void motor_control(uint16_t currentSpeed)
{
    if (error == 0)
    {
        set_motor(currentSpeed, currentSpeed); // đi thẳng
    }
    else if (error > 0)
    {                                                                          // line bên phải
        set_motor(currentSpeed + abs(PIDValue), currentSpeed - abs(PIDValue)); // bánh trái mạnh, bánh phải yếu
    }
    else if (error < 0)
    {                                                                          // line bên trái
        set_motor(currentSpeed - abs(PIDValue), currentSpeed + abs(PIDValue)); // bánh trái yếu, bánh phải mạnh
    }
}

float getDistance()
{
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 15000); // Đo thời gian phản hồi từ cảm biến

    float distance = 0;

    if (duration == 0)
    {
        distance = -1; 
    }
    else
    {
        distance = duration * 0.034 / 2; // Chuyển đổi sang cm
    }

    // Kiểm tra khoảng cách ko hợp lệ
    if (distance <= 2 && distance != -1)
    {
        distance = -1;
    }

    if (distance >= 100)
    {
        distance = 100;
    }

    return distance;
}

float movingAverageFilter(float newValue)
{
    if (newValue == -1)
    {
        return sum / N; // Bỏ qua giá trị lỗi
    }

    sum = sum - distanceArray[idx] + newValue; // Cập nhật tổng
    distanceArray[idx] = newValue;             // Cập nhật giá trị trong mảng
    idx = (idx + 1) % N;                       // Di chuyển vị trí theo vòng tròn

    return sum / N;
}