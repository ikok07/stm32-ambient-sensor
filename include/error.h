//
// Created by Kok on 7/12/25.
//

#ifndef ERROR_H
#define ERROR_H

#define ERR_LED_PORT            GPIOA
#define ERR_LED_PIN             8

void InitErrorLED();
void TriggerError(char *msgFormat, ...);
void ClearActiveError();

#endif //ERROR_H
