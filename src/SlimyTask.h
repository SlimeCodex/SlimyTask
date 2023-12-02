/*
 * This file is part of SlimyTask Library.
 * Copyright (C) 2023 Alejandro Nicolini
 *
 * SlimyTask is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SlimyTask is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SlimyTask. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ___SLIMY_TASK_H___
#define ___SLIMY_TASK_H___

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class SlimyTask {
private:
    TaskHandle_t xHandle = NULL;
    StaticTask_t xTaskBuffer;
    StackType_t* xStack;
    size_t stackSize;
    void (*taskFunction)(void* pvParameter);
    const char* taskName;
    UBaseType_t priority;
    BaseType_t core;

    // Monitoring variables
    uint32_t lastExecutionTimestamp = 0;
    uint32_t executionTime = 0;
    uint32_t loopCounter = 0;
    uint32_t m_executionEndTime = 0;

    uint32_t lastCheckTimestamp = 0;
    uint32_t lastLoopCounterAtCheck = 0;
    uint32_t freezeTimeout = 400;

public:
    SlimyTask(size_t _stackSize, void (*_taskFunction)(void*), const char* _taskName, UBaseType_t _priority, BaseType_t _core) : stackSize(_stackSize), taskFunction(_taskFunction), taskName(_taskName), priority(_priority), core(_core) {
        xStack = new StackType_t[stackSize];
    }

    ~SlimyTask() {
        stop();
        delete[] xStack;
    }

    void start() {
        if (xHandle == NULL) {
            xHandle = xTaskCreateStaticPinnedToCore(taskFunction, taskName, stackSize, NULL, priority, xStack, &xTaskBuffer, core);
        }
    }

    void stop() {
        if (xHandle != NULL) {
            vTaskDelete(xHandle);
            xHandle = NULL;
        }
    }

    void pause() {
        if (xHandle != NULL) {
            vTaskSuspend(xHandle);
        }
    }

    void resume() {
        if (xHandle != NULL) {
            vTaskResume(xHandle);
        }
    }

    bool isRunning() {
        return (xHandle != NULL && eTaskGetState(xHandle) != eSuspended);
    }

    void restart() {
        stop();
        start();
    }

    bool isBusy() {
        uint32_t currentTimestamp = millis();

        if (loopCounter == lastLoopCounterAtCheck && (currentTimestamp - lastCheckTimestamp) > freezeTimeout) {
            return true;
        }

        lastCheckTimestamp = currentTimestamp;
        lastLoopCounterAtCheck = loopCounter;

        return false;
    }

    TaskHandle_t getTaskHandle() {
        return xHandle;
    }

    UBaseType_t getStackHighWaterMark() {
        if (xHandle != NULL) {
            return uxTaskGetStackHighWaterMark(xHandle);
        }
        return 0;
    }

    void sendNotification(uint32_t value, eNotifyAction action = eSetValueWithOverwrite) {
        if (xHandle != NULL) {
            xTaskNotify(xHandle, value, action);
        }
    }

    uint32_t receiveNotification(TickType_t waitTicks = portMAX_DELAY) {
        uint32_t notificationValue = 0;
        if (xHandle != NULL) {
            xTaskNotifyWait(0, 0, &notificationValue, waitTicks);
        }
        return notificationValue;
    }

    void startExecution() {
        lastExecutionTimestamp = millis();
    }

    void endExecution() {
        uint32_t currentTimestamp = millis();
        executionTime = currentTimestamp - lastExecutionTimestamp;
        loopCounter++;

        m_executionEndTime = xTaskGetTickCount();
    }

    uint32_t getExecutionTime() const {
        return executionTime;
    }

    const char* getTaskName() const {
        return taskName;
    }

    int getAvailableStackPercentage() const {
        const UBaseType_t stackDepth = uxTaskGetStackHighWaterMark(xHandle);
        return 100 - floor((static_cast<float>(stackDepth) / stackSize) * 100.0);
    }

    float getLoopsPerSecond() {
        return executionTime > 0 ? 1000.0f / executionTime : 0.0f;
    }
};

#endif // ___SLIMY_TASK_H___