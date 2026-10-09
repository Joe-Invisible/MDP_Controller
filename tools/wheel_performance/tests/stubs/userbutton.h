typedef enum { SW1_Idle=1, SW1_Enabled=0 } SW1_State_t;
SW1_State_t SW1_ReadState(void);
void SW1_WaitForPressAndRelease(void);
