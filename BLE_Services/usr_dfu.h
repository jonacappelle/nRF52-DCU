#ifndef __USR_DFU_H__
#define __USR_DFU_H__


void dfu_async_init();
void gap_params_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(bool erase_bonds);



#endif