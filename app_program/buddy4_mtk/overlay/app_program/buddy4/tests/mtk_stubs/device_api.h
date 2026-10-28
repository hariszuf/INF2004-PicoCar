#ifndef BUDDY4_TEST_DEVICE_API_H
#define BUDDY4_TEST_DEVICE_API_H
#include <stddef.h>
#include <stdint.h>
typedef int ER, ID, INT, RELTIM;
typedef long SZ;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef uint32_t UW;
typedef struct {int32_t hi; uint32_t lo;} SYSTIM;
typedef struct {int mtxatr;} T_CMTX;
typedef struct {int tskatr; void (*task)(INT,void *); int itskpri,stksz;} T_CTSK;
typedef struct {UW sadr; SZ snd_size; UB *snd_data; SZ rcv_size; UB *rcv_data;} T_I2C_EXEC;
#define TDN_I2C_EXEC -100
#define E_OK 0
#define E_PAR -17
#define E_OBJ -41
#define E_IO -57
#define E_TMOUT -50
#define TA_INHERIT 2
#define TA_HLNG 1
#define TA_RNG3 0
#define TMO_FEVR -1
#define TD_UPDATE 3
#define TD_EXCL 256
ID tk_opn_dev(UB *name, UINT mode);
ER tk_cls_dev(ID id, UINT option);
ER tk_swri_dev(ID id, int start, const void *buf, SZ size, SZ *actual);
ER tk_get_otm(SYSTIM *t);
ID tk_cre_mtx(const T_CMTX *m);
ER tk_del_mtx(ID id);
ER tk_loc_mtx(ID id, int timeout);
ER tk_unl_mtx(ID id);
ID tk_cre_tsk(const T_CTSK *t);
ER tk_sta_tsk(ID id, INT code);
ER tk_del_tsk(ID id);
ER tk_dly_tsk(RELTIM delay);
#endif
