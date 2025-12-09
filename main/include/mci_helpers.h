#pragma once

#include <inttypes.h>
#include <stddef.h>

const char *mci_state_to_name(uint16_t state);
const char *mci_error_code_to_name(uint16_t error_code);
void format_mci_warning_flags(uint16_t warning_flags, const char *separator, char *buffer, size_t buffer_size);