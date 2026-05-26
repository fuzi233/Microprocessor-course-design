#include "button.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"  /* B1_Pin, B1_GPIO_Port 定义在此 */

/* 消抖参数 */
#define DEBOUNCE_SAMPLES     3U    /* 连续3次相同读数才确认 */
#define LONG_PRESS_TICKS     pdMS_TO_TICKS(1000)  /* 1秒 */
#define DOUBLE_CLICK_TICKS   pdMS_TO_TICKS(400)   /* 双击窗口 */
#define BUTTON_POLL_MS       20U   /* 轮询周期 */

/* 按键状态机 */
typedef enum {
    BTN_STATE_IDLE = 0,
    BTN_STATE_DEBOUNCE_PRESS,
    BTN_STATE_PRESSED,
    BTN_STATE_DEBOUNCE_RELEASE,
} ButtonState;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    ButtonState state;
    uint8_t debounce_cnt;
    TickType_t press_start_tick;
    TickType_t last_release_tick;
    bool pending_double;
} ButtonContext;

static ButtonContext s_btns[BUTTON_COUNT];
static ButtonCallback s_callback = NULL;

void Button_Init(void)
{
    s_btns[BUTTON_B1].port = B1_GPIO_Port;
    s_btns[BUTTON_B1].pin  = B1_Pin;
    s_btns[BUTTON_B1].state = BTN_STATE_IDLE;
    s_btns[BUTTON_B1].debounce_cnt = 0;
    s_btns[BUTTON_B1].press_start_tick = 0;
    s_btns[BUTTON_B1].last_release_tick = 0;
    s_btns[BUTTON_B1].pending_double = false;
}

void Button_RegisterCallback(ButtonCallback cb)
{
    s_callback = cb;
}

static bool Button_IsPressed(const ButtonContext *btn)
{
    /* PC13 按下为低电平 */
    return (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET);
}

static void Button_HandleEvent(ButtonId id, ButtonEvent event)
{
    if (s_callback != NULL)
    {
        s_callback(id, event);
    }
}

void Button_Task(void *argument)
{
    TickType_t now;
    uint8_t i;

    (void)argument;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        now = xTaskGetTickCount();

        for (i = 0; i < BUTTON_COUNT; i++)
        {
            ButtonContext *btn = &s_btns[i];
            bool pressed = Button_IsPressed(btn);

            switch (btn->state)
            {
            case BTN_STATE_IDLE:
                if (btn->pending_double
                    && (now - btn->last_release_tick) > DOUBLE_CLICK_TICKS)
                {
                    btn->pending_double = false;
                    Button_HandleEvent((ButtonId)i, BUTTON_EVENT_SHORT_PRESS);
                }

                if (pressed)
                {
                    btn->debounce_cnt = 1;
                    btn->state = BTN_STATE_DEBOUNCE_PRESS;
                }
                break;

            case BTN_STATE_DEBOUNCE_PRESS:
                if (pressed)
                {
                    btn->debounce_cnt++;
                    if (btn->debounce_cnt >= DEBOUNCE_SAMPLES)
                    {
                        /* 确认按下 */
                        btn->state = BTN_STATE_PRESSED;
                        btn->press_start_tick = now;
                        btn->debounce_cnt = 0;
                    }
                }
                else
                {
                    /* 误触发，回退 */
                    btn->state = BTN_STATE_IDLE;
                    btn->debounce_cnt = 0;
                }
                break;

            case BTN_STATE_PRESSED:
                if (!pressed)
                {
                    btn->debounce_cnt = 1;
                    btn->state = BTN_STATE_DEBOUNCE_RELEASE;
                }
                break;

            case BTN_STATE_DEBOUNCE_RELEASE:
                if (!pressed)
                {
                    btn->debounce_cnt++;
                    if (btn->debounce_cnt >= DEBOUNCE_SAMPLES)
                    {
                        /* 确认释放，判断事件类型 */
                        TickType_t duration = now - btn->press_start_tick;

                        if (duration >= LONG_PRESS_TICKS)
                        {
                            btn->pending_double = false;
                            Button_HandleEvent((ButtonId)i, BUTTON_EVENT_LONG_PRESS);
                        }
                        else
                        {
                            if (btn->pending_double
                                && (now - btn->last_release_tick) <= DOUBLE_CLICK_TICKS)
                            {
                                btn->pending_double = false;
                                Button_HandleEvent((ButtonId)i, BUTTON_EVENT_DOUBLE_CLICK);
                            }
                            else
                            {
                                btn->pending_double = true;
                                btn->last_release_tick = now;
                            }
                        }

                        btn->state = BTN_STATE_IDLE;
                        btn->debounce_cnt = 0;
                    }
                }
                else
                {
                    /* 释放抖动，回到按下状态 */
                    btn->state = BTN_STATE_PRESSED;
                    btn->debounce_cnt = 0;
                }
                break;
            }
        }
    }
}
