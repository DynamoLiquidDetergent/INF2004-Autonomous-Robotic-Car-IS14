/* Private, bounded I2C1 backend for this RTOS port.
 * The repository does not link the Pico SDK I2C runtime. This uses the
 * same RP2040 peripheral as its disabled kernel driver, without enabling
 * that driver or claiming I2C0. Only the IMU task calls this file.
 * RP2040 datasheet: I2C register map, GPIO function 3, timer raw low. */
#include "subsystem4.h"
#include "drv_motion_sensor.h"
#include <stdint.h>

#define S4_I2C_BASE  0x40048000UL
#define S4_REG(a)    (*(volatile uint32_t *)(uintptr_t)(a))
#define S4_I2C(o)    S4_REG(S4_I2C_BASE + (o))
uint32_t s4_board_time_us(void) { return S4_REG(0x40054028UL); }
static bool expired(uint32_t start)
{
    return (s4_board_time_us() - start) >= S4_SENSOR_TIMEOUT_US;
}
static int wait_bits(unsigned int offset, uint32_t mask, uint32_t start)
{
    while ((S4_I2C(offset) & mask) == 0U)
    {
        if (((S4_I2C(0x34U) & (1U << 6U)) != 0U) || expired(start))
        { return -1; }
    }
    return 0;
}
static int transfer(void *context, uint8_t address, uint8_t reg,
                    uint8_t *bytes, s4_size_t count, bool read)
{
    uint32_t start = s4_board_time_us();
    s4_size_t i;
    (void)context;
    if ((count == 0U) || (count > 6U)) { return -1; }
    S4_I2C(0x6cU) = 0U;
    while ((S4_I2C(0x9cU) & 1U) != 0U)
    { if (expired(start)) { return -1; } }
    S4_I2C(0x04U) = address;
    (void)S4_I2C(0x40U);
    S4_I2C(0x6cU) = 1U;
    if (wait_bits(0x70U, 2U, start) != 0) { goto fail; }
    S4_I2C(0x10U) = reg;
    for (i = 0U; i < count; ++i)
    {
        uint32_t command;
        if (wait_bits(0x70U, 2U, start) != 0) { goto fail; }
        command = read ? (1U << 8U) : bytes[i];
        if (read && (i == 0U)) { command |= 1U << 10U; }
        if ((i + 1U) == count) { command |= 1U << 9U; }
        S4_I2C(0x10U) = command;
        if (read)
        {
            if (wait_bits(0x70U, 8U, start) != 0) { goto fail; }
            bytes[i] = (uint8_t)S4_I2C(0x10U);
        }
    }
    if (wait_bits(0x34U, 1U << 9U, start) != 0) { goto fail; }
    if ((S4_I2C(0x34U) & (1U << 6U)) != 0U) { goto fail; }
    (void)S4_I2C(0x60U);
    S4_I2C(0x6cU) = 0U;
    return 0;
fail:
    S4_I2C(0x6cU) = 0U;  /* Stop issuing commands; task reports fault. */
    return -1;
}
int s4_board_bus_init(s4_bus_t *bus)
{
    uint32_t start = s4_board_time_us();
    uint32_t period = S4_PERIPHERAL_CLOCK_HZ / 100000U;
    unsigned int pin;
    /* Atomic CLEAR alias: release only I2C1 from reset. Never reset GPIO. */
    S4_REG(0x4000f000UL) = 1U << 4U;
    while ((S4_REG(0x4000c008UL) & (1U << 4U)) == 0U)
    { if (expired(start)) { return -1; } }
    for (pin = 2U; pin <= 3U; ++pin)
    {
        S4_REG(0x40014004UL + pin * 8U) = 3U; /* I2C function */
        S4_REG(0x4001c004UL + pin * 4U) = 0x4aU; /* input, pull-up, Schmitt */
    }
    S4_I2C(0x6cU) = 0U;
    while ((S4_I2C(0x9cU) & 1U) != 0U)
    { if (expired(start)) { return -1; } }
    S4_I2C(0x00U) = 0x63U; /* master, standard speed, restart, slave disabled */
    S4_I2C(0x30U) = 0U;    /* no IRQ handler needed for this bounded backend */
    S4_I2C(0x88U) = 0U;
    S4_I2C(0x14U) = period * 4U / 10U;
    S4_I2C(0x18U) = period * 6U / 10U;
    S4_I2C(0x7cU) = S4_PERIPHERAL_CLOCK_HZ / 10000000U * 3U;
    bus->transfer = transfer;
    bus->context = NULL;
    return 0;
}
