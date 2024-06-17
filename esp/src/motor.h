extern volatile float wanted_pos_x;
extern volatile float wanted_pos_y;
extern volatile float wanted_velocity;
extern volatile float current_pos_x;
extern volatile float current_pos_y;

extern volatile int32_t left_trim;
extern volatile int32_t right_trim;

void motor_thread();