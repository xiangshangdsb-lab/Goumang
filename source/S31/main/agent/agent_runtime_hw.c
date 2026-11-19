/*
 * agent_runtime_hw.c - platform hook for the on-device agent.
 *
 * The agent runtime stays host-testable; only this file knows about the
 * chassis snapshot and the safety flags.
 */
#include "agent_runtime.h"

#include "app/app_config.h"
#include "app/app_state.h"
#include "protocol/robot_protocol.h"

bool agent_runtime_hw_safety_ok(void)
{
    td_status_t status;
    app_state_snapshot(&status);
    const uint8_t faults = RP_FAULT_ESTOP | RP_FAULT_LOW_BATTERY |
                           RP_FAULT_IMU_OFFLINE | RP_FAULT_UART_OVERFLOW;
    return status.stm32_online && status.mode != TD_MODE_ESTOP &&
           status.stm32_safety_rx_us > 0 &&
           app_now_us() - status.stm32_safety_rx_us <= (int64_t)TD_STM32_ONLINE_MS * 1000 &&
           status.safety.robot_state != RP_ROBOT_ESTOP &&
           status.safety.robot_state != RP_ROBOT_FAULT &&
           (status.safety.fault_flags & faults) == 0u;
}