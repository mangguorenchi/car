#include "mode.h"

static mode_t current_mode;

void Carmode_Init(void){
    current_mode=mode_manual;
  
}


void carmode_SetMode(mode_t mode) {
    current_mode = mode;
  
}


mode_t carmode_Getmode(void) {
    return current_mode;
  
}


