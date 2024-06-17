#include <stdint.h>

uint8_t id_to_len(uint8_t id);
uint8_t handle_message(uint8_t id, uint8_t *in_buf, uint8_t *out_buf);