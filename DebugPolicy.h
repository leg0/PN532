#pragma once

#if PN532_DEBUG
#include "SerialDebugPolicy.h"
#else
#include "NopDebugPolicy.h"
#endif
