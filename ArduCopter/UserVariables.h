// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
/*
#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA
*/

uint16_t radio[7];
int Radio_roll, Radio_pitch, Radio_yaw, Radio_th, Aux_1, Aux_2, Aux_3;
Vector3f pos;
Vector3f velxyz;
float  a_roll, a_pitch, a_yaw, giro_x,giro_y,giro_z;
#endif  // USERHOOK_VARIABLES


