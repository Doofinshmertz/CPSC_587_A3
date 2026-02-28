#pragma once
#include "givio.h"
#include <functional>

namespace panel {
	extern std::function<void(void)> update_lambda_function;
	void updateMenu();
}