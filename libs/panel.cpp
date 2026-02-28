#include "panel.h"

namespace panel {
	std::function<void(void)> update_lambda_function;
	void updateMenu() {
		giv::io::ImGuiBeginFrame();
		update_lambda_function();
		giv::io::ImGuiEndFrame();
	}
}