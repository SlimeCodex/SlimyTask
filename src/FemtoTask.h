/*
 * This file is part of FemtoTask Library.
 * Copyright (C) 2024 SlimeCodex
 *
 * FemtoTask is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FemtoTask is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FemtoTask. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <functional>

// Enums for lifecycle events
enum class TaskEvent { START, PAUSE, RESUME, STOP };
enum class EventStage { BEFORE, AFTER };

// Unified callback signature
using LifecycleCallback = void (*)(TaskEvent, EventStage);

class FemtoTask {
private:
	TaskHandle_t handle = nullptr;
	StaticTask_t taskBuffer;
	StackType_t* stack = nullptr;
	size_t stackSize;
	const char* name;
	UBaseType_t priority;
	BaseType_t core;

	static constexpr size_t DEFAULT_STACK_SIZE = 4 * 1024;

	// Monitoring variables
	uint32_t lastExecutionTimestamp = 0;
	uint32_t executionTime = 0;
	uint32_t loopCounter = 0;
	uint32_t m_executionEndTime = 0;

	LifecycleCallback lifecycleCallback = nullptr;

	std::function<void()> setupCallback = nullptr;
	std::function<void()> loopCallback = nullptr;
	std::function<void()> preSetupCallback = nullptr;
	std::function<void()> postSetupCallback = nullptr;
	std::function<void()> preLoopCallback = nullptr;
	std::function<void()> postLoopCallback = nullptr;

	TickType_t loopDelayTicks;

	void allocateStack();
	void cleanupStack();
	void startExecution();
	void endExecution();
	static void taskFunctionWrapper(void* pvParameter);

public:
	FemtoTask(size_t _stackSize, const char* _name, UBaseType_t _priority, BaseType_t _core,
			  std::function<void()> _setupCallback = nullptr, std::function<void()> _loopCallback = nullptr,
			  uint32_t delayMs = 10);
	~FemtoTask();

	void start();
	void stop();
	void pause();
	void resume();
	void restart();

	void setLoopDelay(uint32_t delayMs);
	void configWatchdogTimer(uint32_t timeoutSeconds, bool panicOnTrigger = false);

	void setLifecycleCallback(LifecycleCallback callback);
	void setPreSetupCallback(std::function<void()> callback);
	void setPostSetupCallback(std::function<void()> callback);
	void setPreLoopCallback(std::function<void()> callback);
	void setPostLoopCallback(std::function<void()> callback);

	const char* getTaskName() const;

	// Task state helpers
	bool isRunning() const;
	bool isReady() const;
	bool isBlocked() const;
	bool isSuspended() const;
	bool isDeleted() const;
	bool isInvalid() const;

	// Monitoring getters
	uint32_t getExecutionTime() const;
	int getLoopsPerSecond() const;
	int getAvailableStackPercentage() const;
	UBaseType_t getStackHighWaterMark() const;

	// Memory usage stats
	size_t getCurrentStackUsage() const;
	size_t getTotalHeap() const;
	size_t getFreeHeap() const;
	size_t getUsedHeap() const;

	// CPU usage estimation
	int getTaskCpuUsageEstimate() const;
};
