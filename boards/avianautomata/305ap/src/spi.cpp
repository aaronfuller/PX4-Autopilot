#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_BARO_DEVTYPE_BMP581, SPI::CS{GPIO::PortD, GPIO::Pin11}, SPI::DRDY{GPIO::PortB, GPIO::Pin15}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM6DSV, SPI::CS{GPIO::PortE, GPIO::Pin4}, SPI::DRDY{GPIO::PortE, GPIO::Pin0}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM45686,   SPI::CS{GPIO::PortE, GPIO::Pin4}, SPI::DRDY{GPIO::PortE, GPIO::Pin0}),
	}),
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM6DSV, SPI::CS{GPIO::PortA, GPIO::Pin14}, SPI::DRDY{GPIO::PortC, GPIO::Pin15}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM45686,   SPI::CS{GPIO::PortA, GPIO::Pin14}, SPI::DRDY{GPIO::PortC, GPIO::Pin15}),
	}),
	initSPIBusExternal(SPI::Bus::SPI4, {
		initSPIConfigExternal(SPI::CS{GPIO::PortC, GPIO::Pin13}),
		initSPIConfigExternal(SPI::CS{GPIO::PortC, GPIO::Pin14}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
