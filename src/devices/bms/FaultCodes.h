#ifndef FAULT_CODES_H
#define FAULT_CODES_H

#include <unordered_map>
#include <string>

const std::unordered_map<uint32_t, std::pair<std::string, std::string>> faultCodeMap = {
    {0x80000000, {"P0AA1", "Precharge Circuit Malfunction"}},
    {0x40000000, {"P0A0C", "Highest Cell Voltage Too High Fault"}},
    {0x20000000, {"P0A0E", "Lowest Cell Voltage Too Low Fault"}},
    {0x10000000, {"P0A10", "Pack Too Hot Fault"}},
    {0x08000000, {"P0A0F", "Cell ASIC Fault"}},
    {0x04000000, {"P0ACA", "Charge Interlock Fault"}},
    {0x02000000, {"P0A0D", "Cell Voltage Over 5 Volts"}},
    {0x01000000, {"P0A04", "Open Wiring Fault"}},
    {0x00800000, {"P0A03", "Pack Voltage Mismatch Error"}},
    {0x00400000, {"P0A80", "Weak Cell Fault"}},
    {0x00200000, {"P0A0B", "Internal Software Fault"}},
    {0x00100000, {"P0A0A", "Internal Heatsink Thermistor Fault"}},
    {0x00080000, {"P0A09", "Internal Hardware Fault"}},
    {0x00040000, {"P0A12", "Cell Balancing Stuck Off"}},
    {0x00020000, {"P0AC0", "Current Sensor Fault"}},
    {0x00010000, {"P0A1F", "Internal Cell Communication Fault"}},
    {0x00008000, {"P0AFA", "Low Cell Voltage Fault"}},
    {0x00004000, {"P0AA6", "High Voltage Isolation Fault"}},
    {0x00002000, {"P0A01", "Pack Voltage Sensor Fault"}},
    {0x00001000, {"P0A02", "Weak Pack Fault"}},
    {0x00000800, {"P0A81", "Fan Monitor Fault"}},
    {0x00000400, {"U0100", "External Communication Fault"}},
    {0x00000200, {"P0560", "Redundant Power Supply Fault"}},
    {0x00000100, {"P0A05", "Input Power Supply Fault"}},
    {0x00000080, {"P0A06", "Charge Limit Enforcement Fault"}},
    {0x00000040, {"P0A07", "Discharge Limit Enforcement Fault"}},
    {0x00000020, {"P0A08", "Charger Safety Relay Fault"}},
    {0x00000010, {"P0A9C", "Battery Thermistor Fault"}}
};

#endif // FAULT_CODES_H
