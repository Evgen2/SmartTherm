/* Pages.hpp - AutoConnect web pages API (aggregator) */

#pragma once

#include <AutoConnect.h>
#include "Shared.hpp"  // URIs and shared controls

// Register all pages (create controls, attach handlers, join to portal)
void RegisterWebPages(AutoConnect& portal);
