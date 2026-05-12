#ifndef PMON_GPIO_H
#define PMON_GPIO_H

/*
 * MAX78002EVKIT PMON trigger routing:
 *   TRIG1 -> P1.6 via JP18
 *   TRIG2 -> P1.7 via JP19
 *
 * The power monitor guide uses:
 *   SYS_START / SYS_COMPLETE -> TRIG1
 *   CNN_START / CNN_COMPLETE -> TRIG2
 */

void pmon_trig_init(void);
void pmon_sys_start(void);
void pmon_sys_complete(void);
void pmon_cnn_start(void);
void pmon_cnn_complete(void);
void pmon_cnn_hold_begin(void);
void pmon_cnn_hold_end(void);

#endif /* PMON_GPIO_H */
