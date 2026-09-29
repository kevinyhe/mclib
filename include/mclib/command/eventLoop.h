// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

/**
 * @brief Event loops store user-defined bindings to be run every frame. This is used mostly to control the bindings
 * necessary for \refitem Trigger
 */
class EventLoop
{
private:
	std::vector<std::function<void()>> bindings;
	// bind() and clear() may be called from inside a binding. While poll()
	// runs, bind() appends here and clear() only sets the flag, so the vector
	// poll() walks never reallocates and the running binding is never
	// destroyed under itself. poll() applies both once every binding is done.
	std::vector<std::function<void()>> pending;
	bool polling = false;
	bool clearRequested = false;

public:
	/**
	 * Initialize a new empty EventLoop with no bindings
	 */
	EventLoop() = default;

	/**
	 * @brief Initialize the EventLoop with a vector of bindings
	 *
	 * @param bindings Vector storing the bindings to initialize the EventLoop
	 */
	explicit EventLoop(std::vector<std::function<void()>> bindings) : bindings(std::move(bindings)) {}

	/**
	 * @brief Initialize the EventLoop with a initializer list of bindings
	 *
	 * @param bindings Initializer list for new bindings
	 */
	EventLoop(const std::initializer_list<std::function<void()>> bindings) : bindings(bindings) {}

	/**
	 * @brief Poll is run every frame and runs each of the bindings. This is generally run by the CommandScheduler
	 *
	 * @details Bindings run in the order they were bound. A binding added from inside a binding first runs on the
	 * next poll. A clear() from inside a binding skips the rest of this poll.
	 */
	void poll()
	{
		polling = true;
		for (std::size_t i = 0; i < bindings.size() && !clearRequested; ++i)
		{
			bindings[i]();
		}
		polling = false;
		if (clearRequested)
		{
			clearRequested = false;
			bindings.clear();
		}
		for (auto &binding : pending)
		{
			bindings.emplace_back(std::move(binding));
		}
		pending.clear();
	}

	/**
	 * @brief Bind a new command to the EventLoop
	 * @param binding The void function binding to run every frame
	 */
	void bind(const std::function<void()> &binding)
	{
		if (polling)
		{
			pending.emplace_back(binding);
			return;
		}
		bindings.emplace_back(binding);
	}

	/**
	 * @brief Clear all bindings on this event loop
	 *
	 * @details Called from inside a binding, the bindings are removed once the running binding returns, and
	 * the rest of that poll is skipped.
	 */
	void clear()
	{
		pending.clear();
		if (polling)
		{
			clearRequested = true;
			return;
		}
		bindings.clear();
	}

	~EventLoop() = default;
};