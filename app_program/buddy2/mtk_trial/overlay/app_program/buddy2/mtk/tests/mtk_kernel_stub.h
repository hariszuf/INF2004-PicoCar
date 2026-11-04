/* Host-test stand-ins for the few micro T-Kernel items motion_mtk_hw.c uses. */
#ifndef MTK_KERNEL_STUB_H
#define MTK_KERNEL_STUB_H
#include <stdint.h>
typedef unsigned int UINT;
typedef int INT;
typedef int ER;
typedef void (*FP)(void);
typedef struct { UINT intatr; FP inthdr; } T_DINT;
#define TA_HLNG 1u
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
extern UINT stub_irq_masked;
#define DI(s) ((s) = stub_irq_masked, stub_irq_masked = 1u)
#define EI(s) (stub_irq_masked = (s))
ER tk_def_int(UINT intno, const T_DINT *p);
void ClearInt(UINT intno);
void EnableInt(UINT intno, INT level);
#endif
