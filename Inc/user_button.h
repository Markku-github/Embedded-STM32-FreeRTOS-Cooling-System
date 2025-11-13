/**
 ******************************************************************************
 * @file    user_button.h
 * @brief   USER button task header
 ******************************************************************************
 */

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

/**
 * @brief  Button task (handles USER button on PC13)
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void UserButtonTask(void *pvParameters);
void UserButton_Init(void);

#endif /* USER_BUTTON_H */
