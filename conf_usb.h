#define USB_DEVICE_VENDOR_ID             0x2323
#define USB_DEVICE_PRODUCT_ID            0x4001
#define USB_DEVICE_MAJOR_VERSION         1
#define USB_DEVICE_MINOR_VERSION         0
#define USB_DEVICE_POWER                 20 // Consumption on Vbus line (mA)
#define USB_DEVICE_ATTR                  (USB_CONFIG_ATTR_BUS_POWERED)

#define USB_DEVICE_MANUFACTURE_NAME      "Bits and Electrons"
#define USB_DEVICE_PRODUCT_NAME          "Piezo Sensor"
// #define USB_DEVICE_SERIAL_NAME           "12...EF"

#define UDC_VBUS_EVENT(b_vbus_high)
/* #define UDC_SOF_EVENT()                  qi_sof_action() */
/* #define UDC_SUSPEND_EVENT()              qi_suspend_action() */
/* #define UDC_RESUME_EVENT()               qi_resume_action() */

#define UDI_CDC_PORT_NB 1

#define UDI_CDC_ENABLE_EXT(port)         piezo_cdc_enable()
#define UDI_CDC_DISABLE_EXT(port)        piezo_cdc_disable()
#define UDI_CDC_RX_NOTIFY(port)
#define UDI_CDC_TX_EMPTY_NOTIFY(port)
#define UDI_CDC_SET_CODING_EXT(port,cfg)
#define UDI_CDC_SET_DTR_EXT(port,set)    piezo_cdc_dtr(set)
#define UDI_CDC_SET_RTS_EXT(port,set)

#define UDI_CDC_LOW_RATE

#define UDI_CDC_DEFAULT_RATE             115200
#define UDI_CDC_DEFAULT_STOPBITS         CDC_STOP_BITS_1
#define UDI_CDC_DEFAULT_PARITY           CDC_PAR_NONE
#define UDI_CDC_DEFAULT_DATABITS         8

#include "udi_cdc_conf.h"
#include "piezo-sensor.h"
