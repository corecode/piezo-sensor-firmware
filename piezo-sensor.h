#ifndef PIEZO_SENSOR_H
#define PIEZO_SENSOR_H

bool piezo_cdc_enable(void);
void piezo_cdc_disable(void);
void piezo_cdc_dtr(int set);

#endif
