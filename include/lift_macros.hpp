/*
 * Copyright (c) 2026, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at 
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

namespace LiftControl {
    // Sets initial hardware configurations (like brake modes)
    void initialize();
    
    // The main control logic that runs continuously in opcontrol
    void update();
}