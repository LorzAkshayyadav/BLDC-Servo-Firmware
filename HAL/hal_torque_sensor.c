#include "hal.h"
#include "usart.h"                 /* CubeMX: husart3 */

#define TORQUE_SENSOR_ID        0xAAu
#define TORQUE_FRAME_LEN        8u
#define TORQUE_TRIGGER_CMD_LEN  6u

static uint8_t s_torque_rx_buf[TORQUE_FRAME_LEN];
static volatile bool s_torque_rx_ready;
static bool s_torque_enabled;

/**
 * @brief Drive the RS-485 DE pin for TX or RX mode.
 * @param enable true to assert the transmitter, false to release the bus.
 */
static void torque_tx_enable(bool enable)
{
    HAL_GPIO_WritePin(GPIOD, TORQ_SNS_DE_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Transmit one command frame to the torque sensor.
 * @param cmd Pointer to the command bytes.
 * @param len Number of command bytes to send.
 */
static void torque_send_cmd(const uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL || len == 0u) {
        return;
    }

    torque_tx_enable(true);
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, len, 10u);
    torque_tx_enable(false);
}

/**
 * @brief Decode one valid 8-byte torque reply.
 *
 * Frame format: 0x49 | sensor_id | torque_float[4] | 0x0D | 0x0A.
 * The payload is IEEE-754 float bytes in the sensor's wire order and is
 * converted back to the project-local physical torque value.
 *
 * @param frame Pointer to the 8-byte DMA RX buffer.
 * @param out Destination sample populated on success.
 * @return true when the frame was valid and decoded, false otherwise.
 */
static bool torque_parse_frame(const uint8_t *frame, hal_torque_sample_t *out)
{
    if (frame == NULL || out == NULL) {
        return false;
    }

    if (frame[0] != 0x49u || frame[6] != 0x0Du || frame[7] != 0x0Au || frame[1] != TORQUE_SENSOR_ID) {
        return false;
    }

    /* The sensor payload occupies bytes [2..5] of the 8-byte frame:
     * [0]=0x49, [1]=sensor ID, [2..5]=float payload, [6..7]=0x0D 0x0A.
     * Example wire bytes B0 75 1C C1 must be interpreted as 0xC11C75B0.
     */
    uint32_t raw = ((uint32_t)frame[5] << 24u)
                 | ((uint32_t)frame[4] << 16u)
                 | ((uint32_t)frame[3] << 8u)
                 | ((uint32_t)frame[2] << 0u);
    union {
        uint32_t u;
        float    f;
    } payload = { .u = raw };

    out->valid = true;
    out->torque_mnm = (int32_t)(payload.f * 1000.0f);
    out->stamp = HAL_GetTick();
    return true;
}

/**
 * @brief HAL UART receive-complete callback for the RS-485 torque sensor.
 *
 * DMA delivers a complete 8-byte frame and marks the buffer as ready for the
 * higher-level polling API to consume without blocking.
 *
 * @param huart UART handle for the torque sensor channel.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        s_torque_rx_ready = true;
    }
}

/**
 * @brief Read the latest valid torque sample if one has arrived.
 *
 * This is a non-blocking poll interface for the motion layer. If no new frame
 * is available, the function leaves @p out marked invalid and returns without
 * disturbing the drive state.
 *
 * @param out Filled with the latest valid sample or invalid result.
 */
void hal_torque_read(hal_torque_sample_t *out)
{
    if (out == NULL) {
        return;
    }

    out->valid = false;
    out->torque_mnm = 0;
    out->stamp = 0u;

    if (!s_torque_enabled) {
        return;
    }

    if (!s_torque_rx_ready) {
        return;
    }

    s_torque_rx_ready = false;

    if (torque_parse_frame(s_torque_rx_buf, out)) {
        HAL_UART_Receive_DMA(&huart3, s_torque_rx_buf, TORQUE_FRAME_LEN);
        return;
    }

    /* Start a fresh DMA receive on the next cycle even if the frame was invalid.
     * This avoids latching stale data and keeps the driver tolerant of misses. */
    HAL_UART_Receive_DMA(&huart3, s_torque_rx_buf, TORQUE_FRAME_LEN);
}

/**
 * @brief Start torque conversion reporting from the sensor.
 *
 * Sends the sensor's start/trigger command and arms the DMA receive buffer so
 * the next reply is captured without blocking.
 */
void hal_torque_sensor_start(void)
{
    static const uint8_t start_cmd[TORQUE_TRIGGER_CMD_LEN] = {
        0x49u, TORQUE_SENSOR_ID, 0x85u, 0x7Fu, 0x0Du, 0x0Au
    };

    s_torque_enabled = true;
    s_torque_rx_ready = false;

    torque_send_cmd(start_cmd, sizeof(start_cmd));
    HAL_UART_Receive_DMA(&huart3, s_torque_rx_buf, TORQUE_FRAME_LEN);
}

/**
 * @brief Stop the torque sensor transmission stream.
 *
 * Sends the stop command and disables the DMA receive path so the motion layer
 * no longer consumes stale frames while the sensor is idle.
 */
void hal_torque_sensor_stop(void)
{
    static const uint8_t stop_cmd[TORQUE_TRIGGER_CMD_LEN] = {
        0x43u, TORQUE_SENSOR_ID, 0x85u, 0x7Fu, 0x0Du, 0x0Au
    };

    s_torque_enabled = false;
    s_torque_rx_ready = false;
    torque_send_cmd(stop_cmd, sizeof(stop_cmd));
    HAL_UART_DMAStop(&huart3);
}

/**
 * @brief Initialise the torque sensor interface.
 *
 * Sets the driver to a known idle state and arms one DMA receive buffer so the
 * first sensor reply can be captured immediately after start-up.
 */
void hal_torque_init(void)
{
    s_torque_rx_ready = false;
    s_torque_enabled = false;
    HAL_UART_Receive_DMA(&huart3, s_torque_rx_buf, TORQUE_FRAME_LEN);
}