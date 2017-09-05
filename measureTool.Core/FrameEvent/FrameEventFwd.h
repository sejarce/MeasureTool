#pragma once

#include <functional>

class	FrameEvent;

using	FrameEventResponser = std::function<void(const FrameEvent&)>;