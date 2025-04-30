#pragma once

#include <etherlab/ecrt.h>
#include <cstdint>

// ---- 状态机相关 ----
// CiA 402 的基本控制字位定义，可以后面封装为函数
namespace CiA402 {
    constexpr uint16_t CONTROLWORD_SHUTDOWN             = 0x0006;
    constexpr uint16_t CONTROLWORD_SWITCH_ON            = 0x0007;
    constexpr uint16_t CONTROLWORD_ENABLE_OPERATION     = 0x000F;
    constexpr uint16_t CONTROLWORD_DISABLE_VOLTAGE      = 0x0000;
    constexpr uint16_t CONTROLWORD_QUICK_STOP           = 0x0002;
    constexpr uint16_t CONTROLWORD_DISABLE_OPERATION    = 0x0007;
    constexpr uint16_t CONTROLWORD_FAULT_RESET          = 0x0080;
}

enum cia402_state_t {
  no_ready_to_switch_on = 0,
  /* Low level power(e.g. +/- 15V, 5V) has been applied to the drive.
   *  The drive is being initialized or is running self test.
   *  A brake, if present, has to be applied in this state.
   *  The drive function is disable.
   * */

  switch_on_disable,
  /* Drive initialization is complete.
   * The drive parameters have been set up.
   * Drive parameters may be changed.
   * High voltage may not be applied to the dirve.
   * The drive function is disabled.
   * */

  ready_to_switch_on,
  /* High voltage may be applied to the drive.
   * The drive parameters may be changed.
   * The drive function is disabled.
   * */

  switched_on,
  /* High voltage has been applied to the drive.
   * The power amplifier is ready.
   * The dirve parameters may be change.
   * The drive functuion is disable.
   * */
  operation_enable,
  /* No faults have been detected.
   * The dirve  function is enabled and power is apllied to the motor.
   * The dirve function is enable.
   * */

  quick_stop_active,
  /* The drive paramters may be changed.
   * The quick stop function is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault_reaction_active,
  /* The parameters may be changed.
   * A fault has occurred in the drive.
   * The quick stop functon is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault,
  /* The drive parameters may be changed.
   * A fault has occurred in the drive.
   * High voltage switch-on/off depends on the application.
   * The drive function is disabled.
   * */
  none
};

// 状态字典位（后续可以细分，比如Ready to Switch On，Fault等检测）
namespace StatusWordMasks {
    constexpr uint16_t MASK_READY_TO_SWITCH_ON  = 0x0001;
    constexpr uint16_t MASK_SWITCHED_ON         = 0x0002;
    constexpr uint16_t MASK_OPERATION_ENABLED   = 0x0004;
    constexpr uint16_t MASK_FAULT               = 0x0008;
    constexpr uint16_t MASK_VOLTAGE_ENABLED     = 0x0010;
    constexpr uint16_t MASK_QUICK_STOP          = 0x0020;
    constexpr uint16_t MASK_SWITCH_ON_DISABLED  = 0x0040;
    constexpr uint16_t MASK_WARNING             = 0x0080;
    constexpr uint16_t MASK_REMOTE              = 0x0200;
    constexpr uint16_t MASK_TARGET_REACHED      = 0x0400;
    constexpr uint16_t MASK_INTERNAL_LIMIT_ACTIVE = 0x0800;
}

