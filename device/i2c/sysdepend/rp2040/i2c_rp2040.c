/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2020-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/03.
 *
 *----------------------------------------------------------------------
 */

#ifdef BUDDY4_NATIVE_I2C_HOST_TEST
#include "../../../../app_program/buddy4/tests/mtk_stubs/i2c_driver_env.h"
#else
#include <sys/machine.h>
#endif
#ifdef CPU_RP2040

#ifndef BUDDY4_NATIVE_I2C_HOST_TEST
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include "../../i2c.h"
#include "../../../include/dev_def.h"
#endif
#if DEV_IIC_ENABLE
/*
 *	i2c_rp2040.c
  *	I2C device driver
 *	System-dependent definition for RP2040
 */

/*----------------------------------------------------------------------
 * Device register base address
*/
const LOCAL UW ba[DEV_I2C_UNITNM] = { I2C0_BASE, I2C1_BASE };

/*----------------------------------------------------------------------
 * Device data
 **** This data is not used in beta version
*/
const LOCAL struct {
	UINT	intno;		// Interrupt number
	PRI	intpri;		// Interrupt priority
	TMO	timout;		// I2C timeout time
} ll_devdat[DEV_I2C_UNITNM] = {
	{	/* I2C0 */
		.intno		= INTNO_I2C0,
		.intpri		= DEVCNF_I2C0_INTPRI,
		.timout		= DEVCNF_I2C0_TMO,
	},
	{	/* I2C1 */
		.intno		= INTNO_I2C1,
		.intpri		= DEVCNF_I2C1_INTPRI,
		.timout		= DEVCNF_I2C1_TMO,
	},
};

/*----------------------------------------------------------------------
 * Device control data
*/
typedef struct {
	ID	done_flgid;	// Transaction phase completion flag
	UW	state;		// Operating state
	UW	sadr;		// Slave address
	ER	ioerr;		// Communication error
	UW	err_reason;	// Error reason
	UW	sdat_num;	// Number of send data
	UW	rdat_num;	// Number of receive data
	UB	*sbuf;		// Send buffer
	UB	*rbuf;		// Receive buffer
	/* Diagnostic hardware-timer offsets; never used for deadlines. */
	UW start_us, last_irq_us, irq_count, last_irq_status, last_irq_state, done_us;
	UW wait_probe[16];
} T_I2C_LLDCB;
LOCAL T_I2C_LLDCB	ll_devcb[DEV_I2C_UNITNM];

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK ll_lock[DEV_I2C_UNITNM];
#define I2C_LOCK(unit, sts)   ((void)ISpinLock(&ll_lock[(unit)], &(sts)))
#define I2C_UNLOCK(unit, sts) ((void)ISpinUnlock(&ll_lock[(unit)], (sts)))
#else
#define I2C_LOCK(unit, sts)   DI(sts)
#define I2C_UNLOCK(unit, sts) EI(sts)
#endif

/* Last failed wait, captured before cleanup changes the controller state. */
LOCAL UW timeout_trace[DEV_I2C_UNITNM][32];
EXPORT void buddy4_i2c_timeout_trace(unsigned int unit, unsigned int out[32])
{
    UINT sts;
    if (unit >= DEV_I2C_UNITNM || out == NULL) return;
    I2C_LOCK(unit, sts);
    for (UINT i = 0; i < 32; ++i) out[i] = (unsigned int)timeout_trace[unit][i];
    I2C_UNLOCK(unit, sts);
}

#define I2C_STS_WAIT_STOP 0x0006u

#define I2C_INT_ERROR	(I2C_INT_TX_ABRT|I2C_INT_TX_OVER|I2C_INT_RX_OVER|I2C_INT_RX_UNDER)

/* Called inside the kernel's existing critical section. Only the I2C flag
 * being observed is matched; the probe never changes wait/flag/timer state. */
EXPORT void buddy4_i2c_wait_event(UINT event, ID id, UW kind, UW deadline, UW now)
{
    if (kind != TTW_FLG) return;
    for (UINT unit = 0; unit < DEV_I2C_UNITNM; ++unit) {
        T_I2C_LLDCB *p = &ll_devcb[unit];
        if (id != p->done_flgid || id <= 0) continue;
        UW elapsed = in_w(TIMER_TIMERAWL) - p->start_us;
        if (event == 0) {
            p->wait_probe[0] = elapsed;
            p->wait_probe[1] = now;
            p->wait_probe[2] = deadline;
            /* CVR phase is observational; a pending SysTick can be serviced
             * immediately after this critical section rather than in CVR us. */
            p->wait_probe[3] = (in_w(SYST_CVR) + 1u) / TMCLK_MHz;
            p->wait_probe[4] = in_w(SCB_ICSR);
        } else if (event == 1) {
            p->wait_probe[5] = elapsed;
            p->wait_probe[6] = now;
            p->wait_probe[14] = p->state;
            p->wait_probe[15] = p->rdat_num;
        } else if (event == 2) {
            p->wait_probe[7] = elapsed;
        }
        break;
    }
}

/*-------------------------------------------------------
 * Interrupt handler
 */
LOCAL void i2c_inthdr( UINT intno )
{
	T_I2C_LLDCB	*p_cb;
	UW		i2c_sts, cmd;
	INT		unit;
	BOOL		abort	= FALSE;
	BOOL		wup	= FALSE;
	UINT		intsts;

	for ( unit = 0; unit < DEV_I2C_UNITNM; unit++ ) {
		if ( ll_devdat[unit].intno == intno ) {
			p_cb = &ll_devcb[unit];
			break;
		}
	}
	if(unit >= DEV_I2C_UNITNM) {
		ClearInt(intno);	// Clear interrupt
		return;
	}
	I2C_LOCK(unit, intsts);

	i2c_sts = in_w(I2C_INTR_STAT(unit));
    /* A pending NVIC request can outlive its masked/cleared peripheral cause
     * (notably at timeout cleanup). It is not a new transfer failure. */
    if (i2c_sts == 0) {
        ClearInt(intno);
        I2C_UNLOCK(unit, intsts);
        return;
    }
	UW irq_us = in_w(TIMER_TIMERAWL) - p_cb->start_us;
    UW gap = irq_us - (p_cb->irq_count ? p_cb->last_irq_us : 0);
    if (gap > p_cb->wait_probe[10]) p_cb->wait_probe[10] = gap;
    if (!p_cb->irq_count) p_cb->wait_probe[11] = irq_us;
	p_cb->last_irq_us = irq_us;
	p_cb->irq_count++;
	p_cb->last_irq_status = i2c_sts;
	p_cb->last_irq_state = p_cb->state;
	if(i2c_sts & I2C_INT_ERROR) abort = TRUE;

	if(!abort) {
	switch ( p_cb->state ) {
	case I2C_STS_SEND:
		if(i2c_sts & I2C_INT_TX_EMPTY) {
			cmd = *(p_cb->sbuf)++ & I2C_DATA_CMD_DAT;
			if(--(p_cb->sdat_num)) {
				cmd |= I2C_DATA_CMD_CMD_WRITE;
			} else {	// Last data
				clr_w(I2C_INTR_MASK(unit), I2C_INT_TX_EMPTY);
				if(p_cb->rdat_num) {
                    /* Keep the combined transaction inside the IRQ state
                     * machine. Waking a task here used to leave the transfer
                     * suspended across a scheduler tick before the restart. */
                    out_w(I2C_DATA_CMD(unit), cmd | I2C_DATA_CMD_CMD_WRITE);
                    cmd = I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_RESTART;
                    if (p_cb->rdat_num == 1) cmd |= I2C_DATA_CMD_STOP;
                    p_cb->state = I2C_STS_RECV;
                    set_w(I2C_INTR_MASK(unit), I2C_INT_RX_FULL | I2C_INT_ERROR);
                    out_w(I2C_DATA_CMD(unit), cmd);
                    break; /* Do not wake the task until physical STOP/error. */
				} else {
					cmd |= I2C_DATA_CMD_CMD_WRITE | I2C_DATA_CMD_STOP;
                    p_cb->state = I2C_STS_WAIT_STOP;
                    set_w(I2C_INTR_MASK(unit), I2C_INT_STOP_DET | I2C_INT_ERROR);
                }
                /* Completion is signalled only at STOP or error. */
			}
			out_w(I2C_DATA_CMD(unit), cmd);
		} else {	// Error
			abort = TRUE;	// Error
		}
		break;
	case I2C_STS_RECV:
		if(i2c_sts & I2C_INT_RX_FULL) {
			*(p_cb->rbuf)++ = in_w(I2C_DATA_CMD(unit)) & I2C_DATA_CMD_DAT;
			if(--(p_cb->rdat_num)) {
                cmd = I2C_DATA_CMD_CMD_READ;
                if (p_cb->rdat_num == 1) cmd |= I2C_DATA_CMD_STOP;
                out_w(I2C_DATA_CMD(unit), cmd);
			} else {	// Last data
				clr_w(I2C_INTR_MASK(unit), I2C_INT_RX_FULL);
                p_cb->state = I2C_STS_WAIT_STOP;
                clr_w(I2C_INTR_MASK(unit), I2C_INT_RX_FULL);
                set_w(I2C_INTR_MASK(unit), I2C_INT_STOP_DET | I2C_INT_ERROR);
			}
		} else {
			abort = TRUE;	// Error
		}
		break;
    case I2C_STS_WAIT_STOP:
        break;
    default:
        abort = TRUE;	// Error
	}
	}

    if (!abort && p_cb->state == I2C_STS_WAIT_STOP &&
        (i2c_sts & I2C_INT_STOP_DET)) {
        p_cb->state = I2C_STS_STOP;
        wup = TRUE;
    }
    if(abort) {
		out_w(I2C_INTR_MASK(unit), 0);	// Mask all innterrupts
		p_cb->ioerr = E_IO;
		p_cb->err_reason = i2c_sts;
		p_cb->state = I2C_STS_STOP;
		wup = TRUE;
	}

    /* Clear only latched causes actually observed. Clearing all here can
     * erase a STOP arriving while the last RX byte is being handled. */
    if (i2c_sts & I2C_INT_STOP_DET) (void)in_w(I2C_CLR_STOP_DET(unit));
    if (i2c_sts & I2C_INT_TX_ABRT) (void)in_w(I2C_CLR_TX_ABRT(unit));
    if (i2c_sts & I2C_INT_TX_OVER) (void)in_w(I2C_CLR_TX_OVER(unit));
    if (i2c_sts & I2C_INT_RX_OVER) (void)in_w(I2C_CLR_RX_OVER(unit));
    if (i2c_sts & I2C_INT_RX_UNDER) (void)in_w(I2C_CLR_RX_UNDER(unit));
	ClearInt(intno);		// Clear interrupt

	if(wup) {
		out_w(I2C_INTR_MASK(unit), 0);
		p_cb->done_us = in_w(TIMER_TIMERAWL) - p_cb->start_us;
#if TK_SUPPORT_SMP
		/* Preserve the existing cross-core publication order. The isolated
		 * trial is SMP=0; unlocking before notification needs an SMP audit. */
		(void)tk_set_flg(p_cb->done_flgid, 1);
#endif
	}
	I2C_UNLOCK(unit, intsts);
#if !TK_SUPPORT_SMP
    /* ARMv6-M END_CRITICAL_SECTION requests PendSV only when PRIMASK was
     * clear on entry. Publish STOP/error and mask the controller under DI,
     * then notify with the IRQ's original mask restored. A single-core task
     * cannot run until this IRQ returns; EI alone does not request dispatch. */
    if (wup) {
        p_cb->wait_probe[8] = in_w(TIMER_TIMERAWL) - p_cb->start_us;
        p_cb->wait_probe[12] = (UW)tk_set_flg(p_cb->done_flgid, 1);
    }
#endif
}
/*----------------------------------------------------------------------
 * Execution of communication
 */
LOCAL ER i2c_trans(INT unit, T_I2C_LLDCB *p_cb)
{
	UW	cmd;
	UINT	imask;
	UINT	flgptn;
	ER	err;

	p_cb->ioerr = err = E_OK;
	p_cb->start_us = in_w(TIMER_TIMERAWL);
	p_cb->last_irq_us = p_cb->done_us = ~0u;
	p_cb->irq_count = p_cb->last_irq_status = p_cb->last_irq_state = 0;
    for (UINT i = 0; i < 16; ++i) p_cb->wait_probe[i] = ~0u;
    p_cb->wait_probe[10] = 0;

	out_w(I2C_TAR(unit), p_cb->sadr & I2C_TAR_7BIT_ADR);	// Set slave address
	out_w(I2C_INTR_MASK(unit), 0);				// Mask all innterrupts
	out_w(I2C_ENABLE(unit), I2C_ENABLE_ENABLE);		// Enable I2C
	(void)in_w(I2C_CLR_INTR(unit));				// Clear all Interrupt Registers
	in_w(I2C_CLR_TX_ABRT(unit));				// Clear Transmit abort

	while (p_cb->state != I2C_STS_STOP) {
		I2C_LOCK(unit, imask);
		(void)tk_clr_flg(p_cb->done_flgid, 0);
		if(p_cb->sdat_num > 0 ) {	/* Send */
			set_w(I2C_INTR_MASK(unit), I2C_INT_TX_EMPTY|I2C_INT_ERROR);
			p_cb->state = I2C_STS_SEND;
		} else {			/* Receive */
			if(p_cb->rdat_num > 1) {
				cmd = I2C_DATA_CMD_CMD_READ;
			} else {	// Last data
				cmd = I2C_DATA_CMD_CMD_READ | I2C_DATA_CMD_STOP;
			}
            if (p_cb->sbuf != NULL) cmd |= I2C_DATA_CMD_RESTART;
            set_w(I2C_INTR_MASK(unit), I2C_INT_RX_FULL|I2C_INT_ERROR);
			out_w(I2C_DATA_CMD(unit), cmd);
			p_cb->state = I2C_STS_RECV;
		}
		I2C_UNLOCK(unit, imask);

		if(p_cb->ioerr != E_OK) {
			err = p_cb->ioerr;
			break;
		}

		err = tk_wai_flg(p_cb->done_flgid, 1,
				 TWF_ORW | TWF_BITCLR, &flgptn,
				 ll_devdat[unit].timout);
        p_cb->wait_probe[9] = in_w(TIMER_TIMERAWL) - p_cb->start_us;
        p_cb->wait_probe[13] = (UW)err;
		if ( err < E_OK ) {
			I2C_LOCK(unit, imask);
            timeout_trace[unit][0] = p_cb->state;
            timeout_trace[unit][1] = p_cb->rdat_num;
            timeout_trace[unit][2] = p_cb->sdat_num;
            timeout_trace[unit][3] = in_w(I2C_RAW_INTR_STAT(unit));
            timeout_trace[unit][4] = in_w(I2C_INTR_MASK(unit));
            timeout_trace[unit][5] = in_w(I2C_TX_ABRT_SOURCE(unit));
            timeout_trace[unit][6] = in_w(I2C_STATUS(unit));
            timeout_trace[unit][7] = in_w(I2C_RXFLR(unit));
            timeout_trace[unit][8] = in_w(TIMER_TIMERAWL) - p_cb->start_us;
            timeout_trace[unit][9] = p_cb->last_irq_us;
            timeout_trace[unit][10] = p_cb->irq_count;
            timeout_trace[unit][11] = p_cb->last_irq_status;
            timeout_trace[unit][12] = p_cb->done_us;
            timeout_trace[unit][13] = p_cb->last_irq_state;
            timeout_trace[unit][14] = (in_w(NVIC_ISER(ll_devdat[unit].intno)) >> ll_devdat[unit].intno) & 1u;
            timeout_trace[unit][15] = (in_w(NVIC_ISPR(ll_devdat[unit].intno)) >> ll_devdat[unit].intno) & 1u;
            for (UINT i = 0; i < 16; ++i) timeout_trace[unit][16+i] = p_cb->wait_probe[i];
			out_w(I2C_INTR_MASK(unit), 0);
			p_cb->state = I2C_STS_STOP;
			I2C_UNLOCK(unit, imask);
			break;
		}
		else err = p_cb->ioerr;
	}
	out_w(I2C_ENABLE(unit), 0);				// Disable I2C

	return err;
}

/*----------------------------------------------------------------------
 * Set communication parameters
 */
Inline void set_com_start(UW unit, UW sadr, UW sdat_num, UW rdat_num, UB *sbuf, UB *rbuf)
{
	ll_devcb[unit].state	= I2C_STS_START;
	ll_devcb[unit].sadr	= sadr;		/* Slave address */
	ll_devcb[unit].sdat_num	= sdat_num;	/* Number of send data */
	ll_devcb[unit].rdat_num	= rdat_num;	/* Number of receive data */
	ll_devcb[unit].sbuf	= sbuf;		/* Send buffer */
	ll_devcb[unit].rbuf	= rbuf;		/* Receive buffer */
}


/*----------------------------------------------------------------------
 * Low level device control
 */
EXPORT W dev_i2c_llctl( UW unit, INT cmd, UW p1, UW p2, UW *pp)
{
	T_I2C_EXEC	*p_ex;
	ER		err	= E_OK;

	p_ex = (T_I2C_EXEC*)pp;

	switch(cmd) {
	case LLD_I2C_OPEN:
		/* Initial Configuration */
		out_w(I2C_ENABLE(unit), 0);						// Disable I2C

		out_w(I2C_CON(unit), I2C_CON_SLAVE_DISABLE | I2C_CON_MASTER_MODE	// Master Mode
			| I2C_CON_RESTART_EN | I2C_CON_TX_EMPTY_CTRL
			| I2C_CON_7BIT_ADR | (unit == 0 ? I2C_CON_SPEED_STANDARD : I2C_CON_SPEED_FAST));			// 7bit Address & Fast Speed

		out_w(I2C_RX_TL(unit), 0);
		out_w(I2C_TX_TL(unit), 0);
		out_w(I2C_DMA_CR(unit), I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE);

		/* I2C0 uses PicoCar's nominal 100 kHz; preserve I2C1 fast mode. */
        out_w(I2C_SS_SCL_HCNT(unit), DEVCNF_I2C0_SS_HCNT);
        out_w(I2C_SS_SCL_LCNT(unit), DEVCNF_I2C0_SS_LCNT);
        /* Existing I2C1 fast-mode counts. */
		out_w(I2C_FS_SCL_HCNT(unit), 126);
		out_w(I2C_FS_SCL_LCNT(unit), 187);
		out_w(I2C_FS_SPKLEN(unit), 187/16);
		out_w(I2C_SDA_HOLD(unit), 38);

		/* I2C interrupt enable */
		break;

	case LLD_I2C_CLOSE:
		/* I2C interrupt disable */
		out_w(I2C_INTR_MASK(unit), 0);
		out_w(I2C_ENABLE(unit), 0);
		break;

	case LLD_I2C_READ:
		set_com_start( unit, p1, 0, p2, NULL, (UB*)pp);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p2 - ll_devcb[unit].rdat_num;
		break;

	case LLD_I2C_WRITE:
		set_com_start( unit, p1, p2, 0, (UB*)pp, NULL);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p2 - ll_devcb[unit].sdat_num;
		break;

	case LLD_I2C_EXEC:
		set_com_start( unit, p_ex->sadr, p_ex->snd_size, p_ex->rcv_size, p_ex->snd_data, p_ex->rcv_data);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p_ex->snd_size + p_ex->rcv_size;
		break;
	}

	return (W)err;
}

/*----------------------------------------------------------------------
 * Device initialization
 */
EXPORT ER dev_i2c_llinit( T_I2C_DCB *p_dcb)
{
	T_DINT	dint;
	T_CFLG	cflg = {0};
	UW	unit;
	ER	err	= E_OK;

	unit = p_dcb->unit;

#if TK_SUPPORT_SMP
	(void)InitSpinLock(&ll_lock[unit]);
#endif
	cflg.flgatr = TA_TFIFO;
	ll_devcb[unit].done_flgid = tk_cre_flg(&cflg);
	if(ll_devcb[unit].done_flgid < E_OK) {
		return ll_devcb[unit].done_flgid;
	}

	switch(unit) {
	case DEV_I2C_0:

#if DEVCNF_I2C_RESET
		/* Reset I2C module*/
		set_w( RESETS_RESET, RESETS_RESET_I2C0);
		clr_w( RESETS_RESET, RESETS_RESET_I2C0);
		while((in_w(RESETS_RESET_DONE)&(RESETS_RESET_I2C0))==0);
#endif

#if DEVCNF_I2C_SETPINFUNC
		out_w(GPIO_CTRL(DEVCNF_I2C0_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(DEVCNF_I2C0_SDA_PIN), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		out_w(GPIO_CTRL(DEVCNF_I2C0_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(DEVCNF_I2C0_SCL_PIN), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
#endif
		break;
	case DEV_I2C_1:
#if DEVCNF_I2C_RESET
		/* Reset I2C module*/
		set_w( RESETS_RESET, RESETS_RESET_I2C1);
		clr_w( RESETS_RESET, RESETS_RESET_I2C1);
		while((in_w(RESETS_RESET_DONE)&(RESETS_RESET_I2C1))==0);
#endif

#if DEVCNF_I2C_SETPINFUNC
		out_w(GPIO_CTRL(6), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(6), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		out_w(GPIO_CTRL(7), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(7), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		break;
#endif
	default:
		(void)tk_del_flg(ll_devcb[unit].done_flgid);
		return E_PAR;
	}

	out_w(I2C_ENABLE(unit), 0);			// Disable I2C

	/* Interrupt handler definition */
	dint.intatr	= TA_HLNG;
	dint.inthdr	= i2c_inthdr;

	err = tk_def_int(ll_devdat[unit].intno, &dint);
	if(err == E_OK) {
		ClearInt(ll_devdat[unit].intno);
		EnableInt(ll_devdat[unit].intno, ll_devdat[unit].intpri);
	} else {
		(void)tk_del_flg(ll_devcb[unit].done_flgid);
	}

	return err;
}

#endif		/* DEV_IIC_ENABLE */
#endif		/* CPU_RP2040 */
