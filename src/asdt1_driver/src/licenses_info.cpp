/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sstream>

std::string get_licenses_info(const char* licenses[][2])
{
    std::ostringstream oss;
    oss << "This SW contains the following open source components:\n";

    for (int i = 0; licenses[i][0]; i++)
    {
        oss << "================================================================================\n";
        oss << licenses[i][0] << "\n";
        oss << "================================================================================\n";
        oss << licenses[i][1] << "\n";
    }

    return oss.str();
}
