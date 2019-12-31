#ifndef __INTERFACE_UAVCAN_HPP
#define __INTERFACE_UAVCAN_HPP

#include "canard_stm32.h"

void start_uavcan_server(void);

extern CanardSTM32Stats can_stats_;

#endif // __INTERFACE_UAVCAN_HPP
