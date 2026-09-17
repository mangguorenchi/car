#ifndef MODE_H
#define MODE_H

typedef enum {
  mode_manual = 0,
  mode_auto=1
}mode_t;

void Carmode_Init(void);
void carmode_SetMode(mode_t mode);
mode_t carmode_Getmode(void);


#endif // MODE_H