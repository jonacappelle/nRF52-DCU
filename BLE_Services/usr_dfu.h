/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: usr_dfu.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: DFU: Firmware upgrades
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef __USR_DFU_H__
#define __USR_DFU_H__

void dfu_async_init();
void gap_params_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(bool erase_bonds);

#endif