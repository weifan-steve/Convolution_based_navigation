#include "magnetic_navigation.h" 
#include "math.h" 



float offset_calc(int readings, int stardard) {
	float normalized_offset = (readings - stardard) / MIDDLE_OF_LINE;
	return normalized_offset; 
}

float calculate_rot_speed(int readings, int stardard) {
	return MAX_TUNE_SPEED * offset_calc(readings, stardard); 
}

int check_stopping_marker() {
   return 0;
}
