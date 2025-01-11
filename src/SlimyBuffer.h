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

#ifndef ___SLIMY_BUFFER_H___
#define ___SLIMY_BUFFER_H___

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <functional>

template <typename T>
class SlimyBuffer {
public:
	// Default constructor
	SlimyBuffer() {
		_sb_mutex = xSemaphoreCreateMutex();
	}

	// Constructor with initial value
	SlimyBuffer(const T& val) : data(val) {
		_sb_mutex = xSemaphoreCreateMutex();
	}

	// Copy constructor
	SlimyBuffer(const SlimyBuffer& other) : data(other.data) {
		_sb_mutex = xSemaphoreCreateMutex();
	}

	// Copy assignment operator
	SlimyBuffer& operator=(const SlimyBuffer& other) {
		if (this != &other) {
			data = other.data;
		}
		return *this;
	}

	// Destructor
	~SlimyBuffer() {
		vSemaphoreDelete(_sb_mutex);
	}

	// Set the value
	bool set(const T& value, TickType_t ticksToWait = portMAX_DELAY) {
		if (xSemaphoreTake(_sb_mutex, ticksToWait) == pdTRUE) {
			data = value;
			xSemaphoreGive(_sb_mutex);
			if (callback) callback(data);
			return true;
		}
		return false;
	}

	// Get the value
	T get(TickType_t ticksToWait = portMAX_DELAY) {
		T value;
		if (xSemaphoreTake(_sb_mutex, ticksToWait) == pdTRUE) {
			value = data;
			xSemaphoreGive(_sb_mutex);
			return value;
		}
		return T(); // Return default-constructed value for type T
	}

	// Execute a function with the data
	void execute(std::function<void(T&)> func) {
		xSemaphoreTake(_sb_mutex, portMAX_DELAY);
		func(data);
		xSemaphoreGive(_sb_mutex);
	}

	// Set the callback function
	void setCallback(std::function<void(const T&)> cb) {
		callback = cb;
	}

private:
	T data;
	SemaphoreHandle_t _sb_mutex;
	std::function<void(const T&)> callback = nullptr;
};

#endif // ___SLIMY_BUFFER_H___