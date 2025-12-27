/* Pages.hpp - AutoConnect web pages API */

#pragma once

#include <AutoConnect.h>

// Export the Info URI so Web.cpp can redirect to it from root
extern const char INFO_URI[];

// Register all pages (create controls, pages, attach handlers, join to portal)
void RegisterWebPages(AutoConnect& portal);

