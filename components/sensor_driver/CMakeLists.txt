set(component_srcs "src/sensor_driver.c"
                   "src/sensor_driver_bme280.c"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
