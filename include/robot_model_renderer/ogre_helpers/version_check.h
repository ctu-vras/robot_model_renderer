// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2019 Bielefeld University

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

// For use like #if (OGRE_VERSION >= OGRE_VERSION_CHECK(1, 9, 0))
#define OGRE_VERSION_CHECK(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
