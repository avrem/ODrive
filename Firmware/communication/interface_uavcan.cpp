#include "odrive_main.h"

#include "interface_uavcan.hpp"

#include "canard.h"
#include "canard_stm32.h"

#include <stdlib.h>

#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0
#define APP_NODE_NAME                                               "com.aeroxo.ss"

#define UNIQUE_ID_LENGTH_BYTES                                      12

#define CANARD_SPIN_PERIOD                                          1000
#define PUBLISHER_PERIOD                                            100

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UAVCAN_RESTART_NODE_DATA_TYPE_ID                            5
#define UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE                     0x569e05394a3017f0
#define UAVCAN_RESTART_NODE_REQUEST_MAX_SIZE                        ((40+7)/8)
#define UAVCAN_RESTART_NODE_RESPONSE_MAX_SIZE                       ((1+7)/8)

#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_TYPE_ID                    1011
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_TYPE_SIGNATURE             0x5e9bba44faf1ea04
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIZE                       (64/8)

#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID                   1010
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE            0xd8a7486238ec3af3

static CanardInstance canard;
static uint8_t canard_memory_pool[1024];
static uint8_t transfer_id = 0;
CanardSTM32Stats can_stats_;

static void readUniqueID(uint8_t* out_uid)
{
    memcpy(out_uid, reinterpret_cast<const void*>(0x1FFFF7E8), UNIQUE_ID_LENGTH_BYTES);
}

void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    uint32_t uptime_sec = (HAL_GetTick() / 1000);
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;

    readUniqueID(&buffer[24]);
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len;
}

static const uint8_t restartNodeMagicNumber[UAVCAN_RESTART_NODE_REQUEST_MAX_SIZE] = {0x1E, 0x1B, 0x55, 0xCE, 0xAC};

static void onRestartNode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uint8_t response = 0;
    if(memcmp(transfer->payload_head, restartNodeMagicNumber, UAVCAN_RESTART_NODE_REQUEST_MAX_SIZE) == 0)
		NVIC_SystemReset();
    canardRequestOrRespond(ins,
			transfer->source_node_id,
			UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE,
			UAVCAN_RESTART_NODE_DATA_TYPE_ID,
			&transfer->transfer_id,
			transfer->priority,
			CanardResponse,
			&response,
			1);
}

static void onGetNodeInfo(CanardInstance *ins, CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    uint16_t len = makeNodeInfoMessage(buffer);
    canardRequestOrRespond(ins,
			transfer->source_node_id,
			UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
			UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
			&transfer->transfer_id,
			transfer->priority,
			CanardResponse,
			&buffer[0],
			(uint16_t)len);
}

void process_actuator_command(uint8_t actuator, uint8_t cmd_type, float setpoint)
{
    if (cmd_type != 0 || actuator < 1 || actuator > 20)
        return;

    for (size_t i = 0; i < AXIS_COUNT; ++i)
        if (axes[i]->config_.use_uavcan_setpoint && axes[i]->config_.uavcan_actuator_id == actuator) {
            int gpio_num = 3 + i;
            float fraction = 0.5f * (setpoint + 1.0f);
            float value = board_config.pwm_mappings[gpio_num - 1].min + fraction * (board_config.pwm_mappings[gpio_num - 1].max - board_config.pwm_mappings[gpio_num - 1].min);
            axes[i]->controller_.pos_setpoint_ = value;
        }
}

static void onArrayCommand(CanardInstance *ins, CanardRxTransfer* transfer)
{
    int offset = 0;
    for (int i = 0; i <= 15; i++) { // max 15 commands in frame
        uint8_t actuator;
        if (canardDecodeScalar(transfer, offset, 8, false, &actuator) < 8)
            return;
        offset += 8;

        uint8_t type;
        if (canardDecodeScalar(transfer, offset, 8, false, &type) < 8)
            return;
        offset += 8;

        uint16_t _setpoint;
        if (canardDecodeScalar(transfer, offset, 16, false, &_setpoint) < 16)
            return;
        offset += 16;
        float setpoint = canardConvertFloat16ToNativeFloat(_setpoint);

        process_actuator_command(actuator, type, setpoint);
    }
}

typedef void (*OnTransferReceivedCB)(CanardInstance* ins, CanardRxTransfer* transfer);

typedef struct {
	uint8_t transfer_type;
	uint16_t data_type_id;
	uint64_t data_type_signature;
	OnTransferReceivedCB cb;
} ReceiveTransfer;


ReceiveTransfer receiveTransfers[] = {
{CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE, onArrayCommand},
{CanardTransferTypeRequest, UAVCAN_GET_NODE_INFO_DATA_TYPE_ID, UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE, onGetNodeInfo},
{CanardTransferTypeRequest, UAVCAN_RESTART_NODE_DATA_TYPE_ID, UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE, onRestartNode},
};

static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
	for (const auto& rt: receiveTransfers)
		if (transfer->transfer_type == rt.transfer_type && transfer->data_type_id == rt.data_type_id) {
			rt.cb(ins, transfer);
                        break;
                }
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
	if (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
		*out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
		return true;
	}

	if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID) {
		*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
		return true;
	}

	if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_RESTART_NODE_DATA_TYPE_ID) {
		*out_data_type_signature = UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE;
		return true;
	}

	return false;
}

void publishActuatorStatus(const Axis *axis)
{
    if (axis->config_.uavcan_actuator_id == 0)
        return;

    int8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIZE];
    memset(buffer, 0, UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIZE);

    canardEncodeScalar(buffer, 0, 8, &axis->config_.uavcan_actuator_id);

    float position = 2 * M_PI * axis->encoder_.pos_estimate_ / (axis->encoder_.config_.cpr * axis->config_.reduction_ratio);
    uint16_t _position = canardConvertNativeFloatToFloat16(position);
    canardEncodeScalar(buffer, 8, 16, &_position);

    // force 16
    // speed 16

    float Id = axis->motor_.current_control_.Id_measured;
    float Iq = axis->motor_.current_control_.Iq_measured;
    float I = sqrtf(Id * Id + Iq * Iq);
    uint8_t _power;
    if (isnan(I) || isinf(I) || I > 100.0f || I < 0.0f)
        _power = 127;
    else
        _power = (int)(I + 0.5f);
    canardEncodeScalar(buffer, 57, 7, &_power);

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_TYPE_SIGNATURE,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_TYPE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    sizeof(buffer));
}

void init_can()
{
    CanardSTM32CANTimings timings;
    canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);
    canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);

    const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    const uint32_t FlagERR = 1U << 29;                  ///< Error frame

    uint32_t service_filter_id = 0x80 | (static_cast<uint32_t>(board_config.uavcan_node_id) << 8) | FlagEFF;
    uint32_t service_filter_mask = 0x7F80 | FlagEFF | FlagRTR | FlagERR;

    uint32_t ac_filter_id = (static_cast<uint32_t>(UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) << 8) | FlagEFF;
    uint32_t ac_filter_mask = 0xFFFF80 | FlagEFF | FlagRTR | FlagERR;

    CanardSTM32AcceptanceFilterConfiguration filters[] = {{service_filter_id, service_filter_mask}, {ac_filter_id, ac_filter_mask}};
    canardSTM32ConfigureAcceptanceFilters(filters, sizeof(filters) / sizeof(filters[0]));
}

void init_node()
{
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);
    canardSetLocalNodeID(&canard, board_config.uavcan_node_id);
}

void pollCanard()
{
    const int max_frames_per_spin = 16;

    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = canardSTM32Transmit(txf);
        if (tx_res == 0) // tx timeout
            break;
        canardPopTxQueue(&canard);
    }

    for (int i = 0; i < max_frames_per_spin; i++) {
        CanardCANFrame rx_frame;
        const int16_t rx_res = canardSTM32Receive(&rx_frame);
        if (rx_res < 1)
            break;
        canardHandleRxFrame(&canard, &rx_frame, HAL_GetTick() * 1000);
    }

    can_stats_ = canardSTM32GetStats();
}

osThreadId uavcan_thread;

static void uavcan_server_thread(void * ctx) 
{
    (void) ctx;
  
    for (;;) {
        pollCanard();

        static uint32_t spin_time = 0;
        if(HAL_GetTick() >= spin_time + CANARD_SPIN_PERIOD) {
            spin_time = HAL_GetTick();

            uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
            makeNodeStatusMessage(buffer);
            canardBroadcast(&canard,
                    UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                    UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    UAVCAN_NODE_STATUS_MESSAGE_SIZE);
        }

        static uint32_t publish_time = 0;
        if (HAL_GetTick() >= publish_time + PUBLISHER_PERIOD) {
            publish_time = HAL_GetTick();
            for (size_t i = 0; i < AXIS_COUNT; ++i)
                publishActuatorStatus(axes[i]);
        }

        osDelay(1);
    }
}

void start_uavcan_server()
{
    if (!board_config.enable_uavcan || board_config.uavcan_node_id == 0)
        return;

    init_can();
    init_node();

    // Start UAVCAN communication thread
    osThreadDef(uavcan_server_thread_def, uavcan_server_thread, osPriorityNormal, 0, 1024);
    uavcan_thread = osThreadCreate(osThread(uavcan_server_thread_def), NULL);
}
