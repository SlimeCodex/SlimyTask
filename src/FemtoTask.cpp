/*
 * FemtoTask.cpp
 * Part of FemtoTask Library.
 * Copyright (C) 2023 Alejandro Nicolini
 *
 * Licensed under GNU GPL v3 or later. See <https://www.gnu.org/licenses/>.
 */

#include "FemtoTask.h"
#include <cmath>
#include <esp_heap_caps.h>

// Private methods
void FemtoTask::allocateStack() {
	size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	if (stackSize > freeHeap) {
		stackSize = freeHeap - 512;
	}
	stackSize = stackSize > 0 ? stackSize : DEFAULT_STACK_SIZE;

	stack = static_cast<StackType_t*>(heap_caps_malloc(stackSize * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
	if (!stack) {
		static StackType_t fallbackStack[DEFAULT_STACK_SIZE];
		stack = fallbackStack;
		stackSize = DEFAULT_STACK_SIZE;
	}
}

void FemtoTask::cleanupStack() {
	if (stack) {
		heap_caps_free(stack);
	}
}

void FemtoTask::startExecution() {
	lastExecutionTimestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void FemtoTask::endExecution() {
	uint32_t currentTimestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
	executionTime = currentTimestamp - lastExecutionTimestamp;
	loopCounter++;
	m_executionEndTime = currentTimestamp;
}

// Constructor and Destructor
FemtoTask::FemtoTask(size_t _stackSize, const char* _name, UBaseType_t _priority, BaseType_t _core,
					 std::function<void()> _setupCallback, std::function<void()> _loopCallback,
					 uint32_t delayMs)
	: stackSize(_stackSize), name(_name), priority(_priority), core(_core),
	  setupCallback(_setupCallback), loopCallback(_loopCallback) {
	loopDelayTicks = pdMS_TO_TICKS(delayMs);
	allocateStack();
}

FemtoTask::~FemtoTask() {
	stop();
	cleanupStack();
}

// Task Function Wrapper
void FemtoTask::taskFunctionWrapper(void* pvParameter) {
	FemtoTask* task = static_cast<FemtoTask*>(pvParameter);

	if (task->preSetupCallback) task->preSetupCallback();
	if (task->setupCallback) task->setupCallback();
	if (task->postSetupCallback) task->postSetupCallback();

	while (true) {
		if (task->preLoopCallback) task->preLoopCallback();

		task->startExecution();
		if (task->loopCallback) task->loopCallback();
		task->endExecution();

		if (task->postLoopCallback) task->postLoopCallback();
		vTaskDelay(task->loopDelayTicks);
	}
}

// Public Methods
void FemtoTask::start() {
	if (!handle && stack) {
		if (lifecycleCallback) lifecycleCallback(TaskEvent::START, EventStage::BEFORE);
		handle = xTaskCreateStaticPinnedToCore(taskFunctionWrapper, name, stackSize / sizeof(StackType_t),
											   this, priority, stack, &taskBuffer, core);
		if (lifecycleCallback) lifecycleCallback(TaskEvent::START, EventStage::AFTER);
	}
}

void FemtoTask::stop() {
	if (handle) {
		if (lifecycleCallback) lifecycleCallback(TaskEvent::STOP, EventStage::BEFORE);
		vTaskDelete(handle);
		handle = nullptr;
		if (lifecycleCallback) lifecycleCallback(TaskEvent::STOP, EventStage::AFTER);
	}
}

// Lifecycle Callbacks
void FemtoTask::setLifecycleCallback(LifecycleCallback callback) {
	lifecycleCallback = callback;
}

// Task state helpers
bool FemtoTask::isRunning() const {
	return handle && eTaskGetState(handle) == eRunning;
}

// Check if the task is ready
bool FemtoTask::isReady() const {
	return handle && eTaskGetState(handle) == eReady;
}

// Check if the task is blocked
bool FemtoTask::isBlocked() const {
	return handle && eTaskGetState(handle) == eBlocked;
}

// Check if the task is suspended
bool FemtoTask::isSuspended() const {
	return handle && eTaskGetState(handle) == eSuspended;
}

// Check if the task is deleted
bool FemtoTask::isDeleted() const {
	return handle && eTaskGetState(handle) == eDeleted;
}

// Check if the task state is invalid
bool FemtoTask::isInvalid() const {
	return !handle || eTaskGetState(handle) == eInvalid;
}

// Get execution time for the last loop
uint32_t FemtoTask::getExecutionTime() const {
	return executionTime;
}

// Get loops per second (rounded)
int FemtoTask::getLoopsPerSecond() const {
	return executionTime > 0 ? static_cast<int>(1000 / executionTime) : 0;
}

// Get available stack percentage
int FemtoTask::getAvailableStackPercentage() const {
	if (handle) {
		UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(handle);
		return static_cast<int>(100 - std::floor((highWaterMark * 100.0) / stackSize));
	}
	return 0;
}

// Get stack high water mark (minimum free stack)
UBaseType_t FemtoTask::getStackHighWaterMark() const {
	return handle ? uxTaskGetStackHighWaterMark(handle) : 0;
}

// Get current stack usage
size_t FemtoTask::getCurrentStackUsage() const {
	if (handle) {
		UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(handle);
		return stackSize - (highWaterMark * sizeof(StackType_t));
	}
	return 0;
}

// Get total heap size
size_t FemtoTask::getTotalHeap() const {
	return heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
}

// Get free heap size
size_t FemtoTask::getFreeHeap() const {
	return heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
}

// Get used heap size
size_t FemtoTask::getUsedHeap() const {
	return getTotalHeap() - getFreeHeap();
}

// Estimate CPU usage for the task
int FemtoTask::getTaskCpuUsageEstimate() const {
	if (executionTime > 0 && loopDelayTicks > 0) {
		uint32_t totalLoopTimeMs = executionTime + (loopDelayTicks * portTICK_PERIOD_MS);
		return static_cast<int>((executionTime * 100) / totalLoopTimeMs);
	}
	return 0;
}

// Set the loop delay dynamically
void FemtoTask::setLoopDelay(uint32_t delayMs) {
	loopDelayTicks = pdMS_TO_TICKS(delayMs);
}

// Configure the Watchdog Timer
void FemtoTask::configWatchdogTimer(uint32_t timeoutSeconds, bool panicOnTrigger) {
	esp_task_wdt_init(timeoutSeconds, panicOnTrigger);
}

// Set individual callbacks
void FemtoTask::setPreSetupCallback(std::function<void()> callback) {
	preSetupCallback = callback;
}

void FemtoTask::setPostSetupCallback(std::function<void()> callback) {
	postSetupCallback = callback;
}

void FemtoTask::setPreLoopCallback(std::function<void()> callback) {
	preLoopCallback = callback;
}

void FemtoTask::setPostLoopCallback(std::function<void()> callback) {
	postLoopCallback = callback;
}

// Pause the task
void FemtoTask::pause() {
    if (handle) {
        vTaskSuspend(handle);
    }
}

// Resume the task
void FemtoTask::resume() {
    if (handle) {
        vTaskResume(handle);
    }
}

// Restart the task
void FemtoTask::restart() {
	stop();
	start();
}