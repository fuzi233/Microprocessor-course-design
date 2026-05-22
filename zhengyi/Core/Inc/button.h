#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 按键事件类型 */
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,    /* 短按 (< 1s) */
    BUTTON_EVENT_LONG_PRESS,     /* 长按 (>= 1s) */
    BUTTON_EVENT_DOUBLE_CLICK,   /* 双击 */
} ButtonEvent;

/* 按键ID */
typedef enum {
    BUTTON_B1 = 0,               /* PC13 蓝色用户按键 */
    BUTTON_COUNT
} ButtonId;

/* 按键回调函数类型 */
typedef void (*ButtonCallback)(ButtonId id, ButtonEvent event);

void Button_Init(void);
void Button_RegisterCallback(ButtonCallback cb);
void Button_Task(void *argument);  /* FreeRTOS 任务入口 */

/* 供 ISR 调用：记录按键中断 */
void Button_ISRNotify(ButtonId id);

#ifdef __cplusplus
}
#endif

#endif
