#define MIDDLE_OF_LINE 50  
#define OFFSET 0 
#define MAX_TUNE_SPEED 0.1 //Maximum tuning speed in the correction 

int magnetic_reading = 50;
float offset_calc(int readings, int stardard);
float calculate_rot_speed(int readings, int stardard); 
int check_stopping_marker(); 