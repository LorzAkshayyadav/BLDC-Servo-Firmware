#ifndef SERVO_MAIN_H
#define SERVO_MAIN_H

/**
 * @brief L4 entry point. Called once from platform/Core/Src/main.c, inside
 *        the preserved USER CODE BEGIN 2 block, after CubeMX has run
 *        SystemClock_Config() and every MX_*_Init().
 *
 * Brings up memory protection, the safety tier, the ADCs, the encoders and
 * the fieldbus, in that order, then starts the carrier last (hal_init_sequencer())
 * so every trigger consumer already exists when TIM1's counter starts
 * pulsing GETSENS lines and requesting DMA transfers. Never returns: control
 * passes into the main loop's fieldbus/parameter-service/scope-readout
 * polling for the remaining lifetime of the program.
 *
 * Never call this from anywhere else, and never call it twice -- see
 * app/servo_main.c's file banner for why platform/ owns clock and peripheral
 * configuration and nothing of ours duplicates it.
 */
void servo_main(void);

#endif /* SERVO_MAIN_H */
