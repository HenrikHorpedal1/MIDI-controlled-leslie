#ifndef FOOTSWITCH_H
#define FOOTSWITCH_H

#ifdef __cplusplus
extern "C" {            // prevents C++ name mangling when included from .ino/.cpp
#endif

extern volatile bool switchA;
extern volatile bool switchB;

void footSwitchTask(void *pvParameters);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FOOTSWITCH_H
