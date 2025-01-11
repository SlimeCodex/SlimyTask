/*
 * This file is part of SlimyTask Library.
 * Copyright (C) 2023 SlimeCodex
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

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"  // Include the Watchdog Timer library
#include <functional>
#include <cmath>

// Enum to represent the type of task lifecycle event
enum class TaskEvent {
    START,
    PAUSE,
    RESUME,
    STOP
};

// Enum to indicate before or after a lifecycle event
enum class EventStage {
    BEFORE,
    AFTER
};

// Unified callback signature
typedef void (*LifecycleCallback)(TaskEvent event, EventStage stage);

class SlimyTask {
private:
    // Task management members
    TaskHandle_t xHandle = NULL;
    StaticTask_t xTaskBuffer;
    StackType_t* xStack = nullptr;
    size_t stackSize;
    const char* taskName;
    UBaseType_t priority;
    BaseType_t core;

    // Default stack size
    static const size_t DEFAULT_STACK_SIZE = 4 * 1024;

    // Monitoring variables
    uint32_t lastExecutionTimestamp = 0;
    uint32_t executionTime = 0;
    uint32_t loopCounter = 0;
    uint32_t m_executionEndTime = 0;
    uint32_t lastCheckTimestamp = 0;
    uint32_t lastLoopCounterAtCheck = 0;
    uint32_t freezeTimeout = 400;

    // Optional unified callback function
    LifecycleCallback lifecycleCallback = nullptr;

    // User-defined setup and loop callbacks
    std::function<void()> setupFunction = nullptr;
    std::function<void()> loopFunction = nullptr;

    // Optional pre- and post-setup and loop callbacks
    std::function<void()> preSetupCallback = nullptr;
    std::function<void()> postSetupCallback = nullptr;
    std::function<void()> preLoopCallback = nullptr;
    std::function<void()> postLoopCallback = nullptr;

    // Programmable delay for the task loop
    TickType_t loopDelayTicks;

public:
    // Constructor with setup, loop, optional delay, and optional pre/post callbacks
    SlimyTask(size_t _stackSize, const char* _taskName, UBaseType_t _priority, BaseType_t _core,
              std::function<void()> _setupFunction = nullptr, std::function<void()> _loopFunction = nullptr,
              uint32_t delayMs = 10)  // Optional delay parameter with a default value
        : stackSize(_stackSize), taskName(_taskName), priority(_priority), core(_core),
          setupFunction(_setupFunction), loopFunction(_loopFunction) {
        
        loopDelayTicks = pdMS_TO_TICKS(delayMs);  // Set initial delay in ticks

        // Memory and stack management
        size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        if (stackSize > freeHeap) {
            stackSize = freeHeap - 512;
        }
        if (stackSize > freeHeap || stackSize <= 0) {
            stackSize = DEFAULT_STACK_SIZE;
        }
        xStack = static_cast<StackType_t*>(heap_caps_malloc(stackSize * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (xStack == nullptr) {
            static StackType_t fallbackStaticStack[DEFAULT_STACK_SIZE];
            xStack = fallbackStaticStack;
            stackSize = DEFAULT_STACK_SIZE;
        }
    }

    // Destructor
    ~SlimyTask() {
        stop();
        if (xStack != nullptr) {
            heap_caps_free(xStack);
        }
    }

    // Set the loop delay dynamically
    void setLoopDelay(uint32_t delayMs) {
        loopDelayTicks = pdMS_TO_TICKS(delayMs);
    }

    // Configure the Watchdog Timer
    void configWatchdogTimer(uint32_t timeoutSeconds, bool panicOnTrigger = false) {
        esp_task_wdt_init(timeoutSeconds, panicOnTrigger);
    }

    // Set individual callbacks
    void setPreSetupCallback(std::function<void()> callback) {
        preSetupCallback = callback;
    }

    void setPostSetupCallback(std::function<void()> callback) {
        postSetupCallback = callback;
    }

    void setPreLoopCallback(std::function<void()> callback) {
        preLoopCallback = callback;
    }

    void setPostLoopCallback(std::function<void()> callback) {
        postLoopCallback = callback;
    }

    // Start the task
    void start() {
        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::START, EventStage::BEFORE);
        }

        if (xHandle == NULL && xStack != nullptr) {
            xHandle = xTaskCreateStaticPinnedToCore(taskFunctionWrapper, taskName, stackSize / sizeof(StackType_t), this, priority, xStack, &xTaskBuffer, core);
            if (xHandle == NULL) {
                // Handle task creation failure if needed
            }
        }

        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::START, EventStage::AFTER);
        }
    }

    // Stop the task
    void stop() {
        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::STOP, EventStage::BEFORE);
        }

        if (xHandle != NULL) {
            vTaskDelete(xHandle);
            xHandle = NULL;
        }

        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::STOP, EventStage::AFTER);
        }
    }

    // Pause the task
    void pause() {
        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::PAUSE, EventStage::BEFORE);
        }

        if (xHandle != NULL) {
            vTaskSuspend(xHandle);
        }

        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::PAUSE, EventStage::AFTER);
        }
    }

    // Resume the task
    void resume() {
        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::RESUME, EventStage::BEFORE);
        }

        if (xHandle != NULL) {
            vTaskResume(xHandle);
        }

        if (lifecycleCallback) {
            lifecycleCallback(TaskEvent::RESUME, EventStage::AFTER);
        }
    }

    // Restart the task
    void restart() {
        stop();
        start();
    }

    // Static task function wrapper
    static void taskFunctionWrapper(void* pvParameter) {
        SlimyTask* task = static_cast<SlimyTask*>(pvParameter);

        // Run pre-setup callback if defined
        if (task->preSetupCallback) {
            task->preSetupCallback();
        }

        // Run the setup function once if provided
        if (task->setupFunction != nullptr) {
            task->setupFunction();
        }

        // Run post-setup callback if defined
        if (task->postSetupCallback) {
            task->postSetupCallback();
        }

        // Enter the loop
        while (1) {
            // Run pre-loop callback if defined
            if (task->preLoopCallback) {
                task->preLoopCallback();
            }

            // Start tracking execution time
            task->startExecution();

            // Run the user-provided loop function
            if (task->loopFunction != nullptr) {
                task->loopFunction();
            }

            // End tracking execution time
            task->endExecution();

            // Run post-loop callback if defined
            if (task->postLoopCallback) {
                task->postLoopCallback();
            }

            // Delay to yield CPU to other tasks, allowing for configurable delay
            vTaskDelay(task->loopDelayTicks);
        }
    }

    // Start tracking execution time
    void startExecution() {
        lastExecutionTimestamp = millis();
    }

    // End tracking execution time
    void endExecution() {
        uint32_t currentTimestamp = millis();
        executionTime = currentTimestamp - lastExecutionTimestamp;
        loopCounter++;
        m_executionEndTime = xTaskGetTickCount();
    }

    // Set unified callback function for task lifecycle events
    void setLifecycleCallback(LifecycleCallback callback) {
        lifecycleCallback = callback;
    }

    // Getter for Execution Time
    uint32_t getExecutionTime() const {
        return executionTime;
    }

    // Getter for Loops per Second as an integer (rounded)
    int getLoopsPerSecond() const {
        return executionTime > 0 ? static_cast<int>(1000 / executionTime) : 0;
    }

    // Getter for Available Stack Percentage
    int getAvailableStackPercentage() const {
        if (xHandle != NULL) {
            UBaseType_t stackDepth = uxTaskGetStackHighWaterMark(xHandle);
            return static_cast<int>(100 - std::floor((static_cast<float>(stackDepth) / stackSize) * 100.0));
        }
        return 0;
    }

    // Getter for Stack High Water Mark
    UBaseType_t getStackHighWaterMark() const {
        return (xHandle != NULL) ? uxTaskGetStackHighWaterMark(xHandle) : 0;
    }
    
    // Check if the task is currently running
    bool isRunning() const {
        return (xHandle != NULL && eTaskGetState(xHandle) == eRunning);
    }

    // Check if the task is currently ready
    bool isReady() const {
        return (xHandle != NULL && eTaskGetState(xHandle) == eReady);
    }

    // Check if the task is currently blocked
    bool isBlocked() const {
        return (xHandle != NULL && eTaskGetState(xHandle) == eBlocked);
    }

    // Check if the task is currently suspended
    bool isSuspended() const {
        return (xHandle != NULL && eTaskGetState(xHandle) == eSuspended);
    }

    // Check if the task has been deleted
    bool isDeleted() const {
        return (xHandle != NULL && eTaskGetState(xHandle) == eDeleted);
    }

    // Check if the task state is invalid (e.g., task handle is NULL)
    bool isInvalid() const {
        return (xHandle == NULL || eTaskGetState(xHandle) == eInvalid);
    }

    // Getter for Task Name
    const char* getTaskName() const {
        return taskName;
    }

    // Getter for Current Stack Usage
    size_t getCurrentStackUsage() const {
        if (xHandle != NULL) {
            UBaseType_t stackDepth = uxTaskGetStackHighWaterMark(xHandle);
            return stackSize - stackDepth * sizeof(StackType_t);
        }
        return 0;
    }

    // Getter for Total Heap Size
    size_t getTotalHeap() const {
        return heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    }

    // Getter for Available Heap Size
    size_t getFreeHeap() const {
        return heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    }

    // Getter for Used Heap Size
    size_t getUsedHeap() const {
        return getTotalHeap() - getFreeHeap();
    }

    // Estimate CPU Usage by Calculating Time Spent in Loop as an integer percentage (0 to 100)
    int getTaskCpuUsageEstimate() const {
        if (executionTime > 0 && loopDelayTicks > 0) {
            // Estimation: (execution time) / (execution time + delay)
            uint32_t totalLoopTimeMs = executionTime + loopDelayTicks * portTICK_PERIOD_MS;
            return static_cast<int>((executionTime * 100) / totalLoopTimeMs);
        }
        return 0;
    }
};
