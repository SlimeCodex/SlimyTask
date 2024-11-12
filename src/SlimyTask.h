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

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

    // Use static allocation for stack
    static const size_t DEFAULT_STACK_SIZE = 4 * 1024;  // A default stack size to use

    // Monitoring variables
    uint32_t lastExecutionTimestamp = 0;
    uint32_t executionTime = 0;
    uint32_t loopCounter = 0;
    uint32_t m_executionEndTime = 0;
    uint32_t lastCheckTimestamp = 0;
    uint32_t lastLoopCounterAtCheck = 0;
    uint32_t freezeTimeout = 400;

    // Optional unified callback function for task lifecycle events
    LifecycleCallback lifecycleCallback = nullptr;

    // User-defined setup and loop callbacks
    std::function<void()> setupFunction = nullptr;
    std::function<void()> loopFunction = nullptr;

    // Programmable delay for the task loop
    TickType_t loopDelayTicks = pdMS_TO_TICKS(10); // Default to 10 ms

public:
    // Constructor with setup and loop callbacks
    SlimyTask(size_t _stackSize, const char* _taskName, UBaseType_t _priority, BaseType_t _core,
              std::function<void()> _setupFunction = nullptr, std::function<void()> _loopFunction = nullptr)
        : stackSize(_stackSize), taskName(_taskName), priority(_priority), core(_core),
          setupFunction(_setupFunction), loopFunction(_loopFunction) {
        
        // Check available heap space to decide whether the requested stack size is feasible
        size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

        // If stack size is more than available heap, set a warning and adjust it
        if (stackSize > freeHeap) {
            //Serial.printf("Warning: Requested stack size (%zu bytes) exceeds available internal heap (%zu bytes). Using available heap size instead.\n", stackSize, freeHeap);
            stackSize = freeHeap - 512;  // Leave some space for other uses to avoid system instability
        }

        // If stack size is still too large, print error and use a default safe stack size
        if (stackSize > freeHeap || stackSize <= 0) {
            //Serial.printf("Error: Stack size adjustment failed. Using default stack size: %zu bytes.\n", DEFAULT_STACK_SIZE);
            stackSize = DEFAULT_STACK_SIZE;
        }

        // Allocate the stack in internal memory explicitly to ensure proper alignment and type
        xStack = static_cast<StackType_t*>(heap_caps_malloc(stackSize * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        
        if (xStack == nullptr) {
            //Serial.println("Failed to allocate stack memory - using static stack buffer instead.");
            static StackType_t fallbackStaticStack[DEFAULT_STACK_SIZE];
            xStack = fallbackStaticStack;
            stackSize = DEFAULT_STACK_SIZE;  // Fall back to the default size
        }
    }

    // Destructor
    ~SlimyTask() {
        stop();
        if (xStack != nullptr) {
            heap_caps_free(xStack);
        }
    }

    // Set the loop delay
    void setLoopDelay(uint32_t delayMs) {
        loopDelayTicks = pdMS_TO_TICKS(delayMs);
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
                //Serial.println("Failed to create task");
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

        // Run the setup function once if provided
        if (task->setupFunction != nullptr) {
            task->setupFunction();
        }

        // Enter the loop
        while (1) {
            // Start tracking execution time
            task->startExecution();

            // Run the user-provided loop function
            if (task->loopFunction != nullptr) {
                task->loopFunction();
            }

            // End tracking execution time
            task->endExecution();

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

    // Getter for Loops per Second
    float getLoopsPerSecond() const {
        return executionTime > 0 ? 1000.0f / static_cast<float>(executionTime) : 0.0f;
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

    // Estimate CPU Usage by Calculating Time Spent in Loop
    float getTaskCpuUsageEstimate() const {
        if (executionTime > 0 && loopDelayTicks > 0) {
            // Estimation: (execution time) / (execution time + delay)
            float totalLoopTimeMs = executionTime + loopDelayTicks * portTICK_PERIOD_MS;
            return (executionTime / totalLoopTimeMs) * 100.0f;
        }
        return 0.0f;
    }
};