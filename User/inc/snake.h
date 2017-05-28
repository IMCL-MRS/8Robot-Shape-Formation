#ifndef __SNAKE_H
#define __SNAKE_H

extern void snakeForm(float x, float y);
extern void snakeRun(float i, float j,int stage);
extern float readAaccelZ();
extern int isPickedUp();
extern void drawCircle();
extern void snakeStart(float x, float y);
extern void goFastStraight(float x, float y);
extern void sychronize();

#endif

