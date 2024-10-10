#include <string>
#include <mujoco/mujoco.h>


int handCtrl(mjModel *m,mjData *d,float V[]);

int handCtrlTest(mjModel *m,mjData *d,double t);