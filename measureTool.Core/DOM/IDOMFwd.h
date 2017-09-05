#pragma once

#include <memory>
#include <vector>

class	IDOM;
using	IDOMSPtr = std::shared_ptr<IDOM>;
using	IDOMWPtr = std::weak_ptr<IDOM>;
using	IDOMList = std::vector<IDOMSPtr>;