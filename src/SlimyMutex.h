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

#ifndef ___SLIMY_MUTEX_H___
#define ___SLIMY_MUTEX_H___

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SlimyMutex {
public:
	// Constructor: creates a new mutex
	SlimyMutex() {
		mutex = xSemaphoreCreateMutex();
	}

	// Destructor: deletes the mutex
	~SlimyMutex() {
		vSemaphoreDelete(mutex);
	}

	// Attempt to take the mutex, waiting for up to `ticksToWait` ticks
	bool take(TickType_t ticksToWait = portMAX_DELAY) {
		return xSemaphoreTake(mutex, ticksToWait) == pdTRUE;
	}

	// Release the mutex
	bool give() {
		return xSemaphoreGive(mutex) == pdTRUE;
	}

private:
	SemaphoreHandle_t mutex;
};

// Use std::lock_guard for scoped locking
using ScopedLock = std::lock_guard<SlimyMutex>;

#endif // ___SLIMY_MUTEX_H___