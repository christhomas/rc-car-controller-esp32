#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define COMMAND_QUEUE_LENGTH 32
#define COMMAND_INTERVAL 1000
#define SNES_CONTROLLER_INTERVAL 100
#define SNES_TASK_CORE 0
#define SNES_TASK_PRIORITY 1
#define CUSTOM_CONTROLLER_MEMORY 2048
#define USER_INTERFACE_MEMORY 8192

typedef enum {
    CONTROLLER = 0,
    BATTERY,
} CommandMessageType_t;

typedef struct snesButtons {
    bool up;
    bool down;
    bool left;
    bool right;
    bool a;
    bool b;
    bool x;
    bool y; 
    bool l;
    bool r;
    bool select;
    bool start;
} SnesButtons_t;

typedef struct {
    float voltage;
    int percent;
} Battery_t;

typedef struct {
    CommandMessageType_t type;

    union {
        SnesButtons_t controller;
        Battery_t battery;
    };
} CommandMessage_t;

extern void createSnesControllerTask(QueueHandle_t commandQueue);
extern void createCustomControllerTask(QueueHandle_t commandQueue);
extern void createBatteryMonitorTask(QueueHandle_t commandQueue);
extern QueueHandle_t createUserInterfaceTaskU8G2();
extern QueueHandle_t createUserInterfaceTaskLVGL();