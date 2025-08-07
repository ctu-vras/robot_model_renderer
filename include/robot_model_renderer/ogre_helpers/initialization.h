// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <string>
#include <vector>

namespace robot_model_renderer
{

typedef std::vector<std::string> V_string;

void cleanupOgre();

void initializeResources(const V_string& resource_paths);

}
