#pragma once

/**
 * @file modules.h
 * @brief Central header for all autonomous modules in the CubeSat system
 * 
 * This file provides a unified include point for all module types used in the
 * CubeSat's autonomous control system. Modules implement specific behaviors
 * (MPPT, debris avoidance, thermal control, etc.) and are executed by the Manager.
 * 
 * Architecture:
 * - Each module inherits from Module base class
 * - Modules are polled periodically by Manager
 * - condition() determines if module should execute
 * - poll() performs the module's action
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#ifndef MODULES_H
#define MODULES_H

#include "module.h"
#include "mppt.h"

// Future modules can be added here:
// #include "debris_avoidance.h"
// #include "thermal_control.h"
// #include "power_management.h"

/**
 * Module execution priority levels
 * Used by Manager to sort modules before execution
 */
enum class ModulePriority {
    CRITICAL = 0,    ///< Safety-critical modules (debris avoidance, battery protection)
    HIGH = 1,        ///< Important modules (ADCS, energy management)
    NORMAL = 2,      ///< Regular modules (MPPT, thermal control)
    LOW = 3          ///< Non-essential modules (science payload, camera)
};

#endif // MODULES_H
