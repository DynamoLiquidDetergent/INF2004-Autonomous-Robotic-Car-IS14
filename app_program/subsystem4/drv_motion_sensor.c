/* Exact chip names and register numbers are confined to this driver.
 * LSM303DLHC DS7202 Rev 2: accel 0x19, magnetic 0x1e, X/Z/Y order. */
#include "drv_motion_sensor.h"
static int read_bytes(const s4_bus_t *bus, uint8_t addr, uint8_t reg,
                      uint8_t *data, s4_size_t size)
{
    return bus->transfer(bus->context, addr, reg, data, size, true);
}
static int write_byte(const s4_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t value)
{
    return bus->transfer(bus->context, addr, reg, &value, 1U, false);
}
static int16_t signed_word(uint16_t word)
{
    int32_t value = (word >= 32768U) ? (int32_t)word - 65536 : word;
    return (int16_t)value;
}
int s4_sensor_init(const s4_bus_t *bus)
{
    uint8_t id[3], check = 0U;
    if ((bus == NULL) || (bus->transfer == NULL)) { return -1; }
    if (read_bytes(bus, 0x1eU, 0x0aU, id, 3U) != 0) { return -2; }
    if ((id[0] != 'H') || (id[1] != '4') || (id[2] != '3')) { return -3; }
    /* Accelerometer: 50 Hz XYZ; BDU, +/-4 g high-resolution, 2 mg/LSB.
     * Magnetometer: 75 Hz (no 50 Hz ODR), +/-1.3 gauss, continuous. */
    if ((write_byte(bus, 0x19U, 0x20U, 0x47U) != 0) ||
        (write_byte(bus, 0x19U, 0x23U, 0x98U) != 0) ||
        (write_byte(bus, 0x1eU, 0x00U, 0x18U) != 0) ||
        (write_byte(bus, 0x1eU, 0x01U, 0x20U) != 0) ||
        (write_byte(bus, 0x1eU, 0x02U, 0x00U) != 0)) { return -4; }
    if ((read_bytes(bus, 0x19U, 0x23U, &check, 1U) != 0) ||
        (check != 0x98U)) { return -5; }
    return 0;
}
int s4_sensor_read(const s4_bus_t *bus, s4_sample_t *sample)
{
    uint8_t status, bytes[6];
    unsigned int i;
    bool magnetic_overflow = false;
    sample->accel_fresh = false;
    sample->magnetic_fresh = false;
    sample->saturated = false;
    if (read_bytes(bus, 0x19U, 0x27U, &status, 1U) != 0) { return -1; }
    if ((status & 0x08U) != 0U)
    {
        if (read_bytes(bus, 0x19U, 0xa8U, bytes, 6U) != 0) { return -1; }
        for (i = 0U; i < 3U; ++i)
        {
            int16_t raw = signed_word((uint16_t)bytes[2U*i] |
                                     ((uint16_t)bytes[2U*i+1U] << 8U));
            int value = raw / 16;
            sample->accel_g[i] = (float)value * 0.002F;
            if ((value <= -2048) || (value >= 2047)) { sample->saturated = true; }
        }
        sample->accel_fresh = true;
    }
    if (read_bytes(bus, 0x1eU, 0x09U, &status, 1U) != 0) { return -1; }
    if ((status & 1U) != 0U)
    {
        static const unsigned int order[3] = {0U, 2U, 1U};
        if (read_bytes(bus, 0x1eU, 0x03U, bytes, 6U) != 0) { return -1; }
        for (i = 0U; i < 3U; ++i)
        {
            unsigned int j = order[i];
            int16_t raw = signed_word(((uint16_t)bytes[2U*j] << 8U) |
                                       bytes[2U*j+1U]);
            sample->magnetic_gauss[i] = (float)raw / ((i == 2U) ? 980.0F : 1100.0F);
            if ((raw == -4096) || (raw <= -2048) || (raw >= 2047))
            { magnetic_overflow = true; }
        }
        sample->magnetic_fresh = !magnetic_overflow;
    }
    return 0;
}
