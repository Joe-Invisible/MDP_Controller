#ifndef INC_MOTIONTASK_H_
#define INC_MOTIONTASK_H_

/*
 * Executor and 100 Hz control loop. Sole owner of the motor, servo
 * and controller instances, plus the one current/latest command batch.
 * Polls complete UART lines between control updates.
 */

/**
 * Thread entry, osThreadNew signature. Performs the full robot
 * bring-up before entering the control loop.
 */
void MotionTask(void *argument);

#endif /* INC_MOTIONTASK_H_ */
