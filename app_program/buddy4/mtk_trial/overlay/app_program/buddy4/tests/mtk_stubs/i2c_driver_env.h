#ifndef BUDDY4_I2C_DRIVER_ENV_H
#define BUDDY4_I2C_DRIVER_ENV_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#define CPU_RP2040 1
#define DEV_IIC_ENABLE 1
#define TK_SUPPORT_SMP 0
#define LOCAL static
#define EXPORT
#define Inline static inline
typedef uint32_t UW, UINT;
typedef int32_t W;
typedef int INT, ER, ID, PRI, TMO, BOOL;
typedef long SZ;
typedef uint8_t UB;
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_IO -57
#define E_PAR -17
#define E_TMOUT -50
#define TA_TFIFO 0
#define TA_HLNG 1
#define TWF_ORW 1
#define TWF_BITCLR 2
#define TTW_FLG 8
#define TMCLK_MHz 125
#define SYST_CVR 0xe000e018u
#define SCB_ICSR 0xe000ed04u
UW mock_disint(void);
void mock_restore_interrupts(UW mask);
#define DI(x) ((x)=mock_disint())
#define EI(x) mock_restore_interrupts(x)
#define TIMER_TIMERAWL 0x40054028u
#define NVIC_ISER(n) (0xe000e100u + ((n)/32u)*4u)
#define NVIC_ISPR(n) (0xe000e200u + ((n)/32u)*4u)
typedef struct {int flgatr;} T_CFLG;
typedef struct {int intatr;void (*inthdr)(UINT);} T_DINT;
typedef struct {UW unit;} T_I2C_DCB;
typedef struct {UW sadr;SZ snd_size;UB *snd_data;SZ rcv_size;UB *rcv_data;} T_I2C_EXEC;
enum {LLD_I2C_OPEN,LLD_I2C_CLOSE,LLD_I2C_READ,LLD_I2C_WRITE,LLD_I2C_EXEC};
#include "../../../../device/i2c/sysdepend/rp2040/i2c_sysdep.h"
#include "../../../../device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h"
#define RESETS_RESET 0x4000c000u
#define RESETS_RESET_DONE 0x4000c008u
#define RESETS_RESET_I2C0 (1u<<3)
#define RESETS_RESET_I2C1 (1u<<4)
#define GPIO_CTRL(p) (0x40014004u+8u*(p))
#define GPIO(p) (0x4001c004u+4u*(p))
#define GPIO_CTRL_FUNCSEL_I2C 3
#define GPIO_IE 64
#define GPIO_DRIVE_4MA 16
#define GPIO_PUE 8
#define GPIO_SHEMITT 2
UW in_w(UW address);
void out_w(UW address,UW value);
void set_w(UW address,UW value);
void clr_w(UW address,UW value);
void ClearInt(UINT number);
void EnableInt(UINT number,PRI priority);
ID tk_cre_flg(const T_CFLG *f);
ER tk_del_flg(ID id);
ER tk_def_int(UINT number,const T_DINT *d);
ER tk_set_flg(ID id,UINT flags);
ER tk_clr_flg(ID id,UINT flags);
ER tk_wai_flg(ID id,UINT pattern,UINT mode,UINT *result,TMO timeout);
#endif
