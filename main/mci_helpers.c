#include "mci_helpers.h"

#include <stdbool.h>
#include <stdio.h>

const char *mci_state_to_name(uint16_t state) {
    uint8_t main_state = state & 0x00FF;

    switch (main_state) {
    case 0:
        return "Not Ready to Switch On";
    case 1:
        return "Switch On Disabled";
    case 2:
        return "Ready to Switch On";
    case 3:
        return "Setup Error";
    case 4:
        return "Error";
    case 5:
        return "Hardware Tests";
    case 6:
        return "Ready to Switch On";
    case 8:
        return "Operation Enabled";
    case 9:
        return "Homing";
    case 10:
        return "Clearance Check";
    case 11:
        return "Going to Initial Position";
    case 12:
        return "Aborting";
    case 13:
        return "Freezing";
    case 14:
        return "Quick Stop";
    case 15:
        return "Going to Position";
    case 16:
        return "Jogging Positive";
    case 17:
        return "Jogging Negative";
    case 18:
        return "Linearizing";
    case 19:
        return "Phase Search";
    case 20:
        return "Special Mode";
    case 21:
        return "Brake Delay";
    default:
        return "Unknown State";
    }
}

const char *mci_error_code_to_name(uint16_t error_code) {
    switch (error_code) {
    case 0x0000:
        return "No Error";
    case 0x0001:
        return "Logic Supply Too Low";
    case 0x0002:
        return "Logic Supply Too High";
    case 0x0003:
        return "Motor Supply Too Low";
    case 0x0004:
        return "Motor Supply Too High";
    case 0x0007:
        return "Min Position Undershot";
    case 0x0008:
        return "Max Position Overshot";
    case 0x000B:
        return "Position Lag Always Too Big";
    case 0x0020:
        return "Motor Hot Sensor";
    case 0x0022:
        return "Motor Slider Missing";
    case 0x0023:
        return "Motor Short Time Overload";
    case 0x0045:
        return "Motor Communication Lost";
    case 0x0080:
        return "Not Homed";
    case 0x0081:
        return "Unknown Motion Command";
    case 0x0082:
        return "PVT Buffer Overflow";
    case 0x0083:
        return "PVT Buffer Underflow";
    case 0x0084:
        return "PVT Controller Too Fast";
    case 0x0085:
        return "PVT Controller Too Slow";
    case 0x0086:
        return "Motion Command In Wrong State";
    default:
        return "Unknown Error";
    }
}

void format_mci_warning_flags(uint16_t warning_flags, const char *separator, char *buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    buffer[0] = '\0';
    size_t offset = 0;

    const struct {
        uint16_t flag;
        const char *name;
    } flag_names[] = {
        {1 << 0, "Motor Hot"},
        {1 << 1, "Motor Overload"},
        {1 << 2, "Motor Supply Low"},
        {1 << 3, "Motor Supply High"},
        {1 << 4, "Position Lag"},
        {1 << 6, "Drive Hot"},
        {1 << 7, "Not Homed"},
        {1 << 8, "PTC Sensor 1 Hot"},
        {1 << 9, "PTC Sensor 2 Hot"},
        {1 << 10, "Regen Temp Overload"},
        {1 << 11, "Speed Lag"},
        {1 << 12, "Position Sensor"},
        {1 << 14, "Interface Warning"},
        {1 << 15, "App Warning"},
    };

    bool first = true;
    for (size_t i = 0; i < sizeof(flag_names) / sizeof(flag_names[0]); ++i) {
        if ((warning_flags & flag_names[i].flag) == 0) {
            continue;
        }

        if (!first && offset < buffer_size) {
            offset += snprintf(buffer + offset, buffer_size - offset, "%s", separator);
        }

        if (offset < buffer_size) {
            offset += snprintf(buffer + offset, buffer_size - offset, "%s", flag_names[i].name);
        }

        first = false;
    }
}
