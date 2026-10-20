#ifndef BUZZER_H
#define BUZZER_H
void Buzzer_Init(void);
void Buzzer_Beep(void);
void Buzzer_Tick(void); /* Call once per 5ms control cycle, before new events. */
void Buzzer_Stop(void);
#endif
