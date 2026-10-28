#define BUDDY4_NATIVE_I2C_HOST_TEST
#include "../../../device/i2c/sysdepend/rp2040/i2c_rp2040.c"
#include <stdio.h>
typedef struct {UW address,value;} reg_t;
static reg_t regs[100];static unsigned reg_count;
static UW interrupt_status,flag,rx_byte;
static UW commands[32];static unsigned command_count;
static int forced_timeout,forced_abort,disabled_before_stop;
static unsigned received;
static UW *reg(UW address){for(unsigned i=0;i<reg_count;i++)if(regs[i].address==address)return &regs[i].value;assert(reg_count<100);regs[reg_count]=(reg_t){address,0};return &regs[reg_count++].value;}
UW in_w(UW address){
 if(address==RESETS_RESET_DONE)return RESETS_RESET_I2C0|RESETS_RESET_I2C1;
 if(address==I2C_INTR_STAT(0))return interrupt_status;
 if(address==I2C_DATA_CMD(0))return rx_byte++;
 return *reg(address);
}
void out_w(UW address,UW value){
 if(address==I2C_DATA_CMD(0)){assert(command_count<32);commands[command_count++]=value;}
 if(address==I2C_ENABLE(0)&&value==0&&command_count&&ll_devcb[0].state!=I2C_STS_STOP)disabled_before_stop++;
 *reg(address)=value;
}
void set_w(UW a,UW v){*reg(a)|=v;}
void clr_w(UW a,UW v){*reg(a)&=~v;}
void ClearInt(UINT n){(void)n;}
void EnableInt(UINT n,PRI p){(void)n;(void)p;}
ID tk_cre_flg(const T_CFLG *f){(void)f;return 1;}
ER tk_del_flg(ID id){(void)id;return 0;}
ER tk_def_int(UINT n,const T_DINT*d){(void)n;(void)d;return 0;}
ER tk_set_flg(ID id,UINT flags){(void)id;flag|=flags;return 0;}
ER tk_clr_flg(ID id,UINT flags){(void)id;flag&=flags;return 0;}
ER tk_wai_flg(ID id,UINT pattern,UINT mode,UINT *result,TMO timeout){
 (void)id;(void)pattern;(void)mode;assert(timeout==10);
 if(forced_timeout)return E_TMOUT;
 for(unsigned i=0;!flag;i++){
   assert(i<20);
   if(forced_abort)interrupt_status=I2C_INT_TX_ABRT;
   else if(ll_devcb[0].state==I2C_STS_SEND)interrupt_status=I2C_INT_TX_EMPTY;
   else if(ll_devcb[0].state==I2C_STS_RECV){interrupt_status=I2C_INT_RX_FULL;received++;}
   else {assert(ll_devcb[0].state==I2C_STS_WAIT_STOP);interrupt_status=I2C_INT_STOP_DET;}
   i2c_inthdr(INTNO_I2C0);
 }
 *result=flag;flag=0;return 0;
}
static void reset_test(void){command_count=received=0;disabled_before_stop=0;flag=0;interrupt_status=0;forced_timeout=forced_abort=0;}
int main(void){
 T_I2C_DCB d={0};assert(dev_i2c_llinit(&d)==0);
 assert(*reg(GPIO_CTRL(16))==3&&*reg(GPIO_CTRL(17))==3);
 assert(*reg(GPIO_CTRL(8))==0&&*reg(GPIO_CTRL(9))==0);
 assert(dev_i2c_llctl(0,LLD_I2C_OPEN,0,0,NULL)==0);
 assert((*reg(I2C_CON(0))&I2C_CON_SPEED_FAST)==0);
 assert(*reg(I2C_SS_SCL_HCNT(0))==500&&*reg(I2C_SS_SCL_LCNT(0))==750);
 UB tx[]={0x20,0x6f};
 reset_test();assert(dev_i2c_llctl(0,LLD_I2C_WRITE,0x1d,2,(UW*)tx)==2);
 assert(command_count==2&&(commands[1]&I2C_DATA_CMD_STOP));
 assert(disabled_before_stop==0);
 for(unsigned n=1;n<=6;n+=5){
   UB address=0xa8,rx[6]={0};T_I2C_EXEC ex={0x1d,1,&address,(SZ)n,rx};
   reset_test();assert(dev_i2c_llctl(0,LLD_I2C_EXEC,0,0,(UW*)&ex)==(W)(n+1));
   assert(command_count==n+1&&received==n&&disabled_before_stop==0);
   assert(!(commands[0]&I2C_DATA_CMD_STOP));
   assert(commands[1]&I2C_DATA_CMD_RESTART);
   assert(commands[n]&I2C_DATA_CMD_STOP);
   for(unsigned i=1;i<n;i++)assert(!(commands[i]&I2C_DATA_CMD_STOP));
 }
 reset_test();forced_timeout=1;
 assert(dev_i2c_llctl(0,LLD_I2C_WRITE,0x1d,2,(UW*)tx)==E_TMOUT);
 assert(*reg(I2C_INTR_MASK(0))==0&&*reg(I2C_ENABLE(0))==0);
 reset_test();forced_abort=1;
 assert(dev_i2c_llctl(0,LLD_I2C_WRITE,0x1d,2,(UW*)tx)==E_IO);
 assert(*reg(I2C_ENABLE(0))==0);
 puts("Professor-derived native I2C regressions passed: pin map, STOP completion, restart, six-byte read, timeout and abort");
 return 0;
}
