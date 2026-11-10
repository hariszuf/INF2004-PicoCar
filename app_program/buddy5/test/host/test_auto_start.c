/* Host test of the start/stop policy (auto_start.h). gcc -std=c11 -Wall -Wextra -I../../vehicle test_auto_start.c */
#include <stdio.h>
#include "auto_start.h"
static int fails;
#define CHECK(c) do{ if(!(c)){ printf("FAIL line %d: %s\n",__LINE__,#c); fails++; } }while(0)
static uint32_t now;
/* Run the controller policy for ms milliseconds; returns the last non-trivial action seen. */
static auto_action_t run(auto_start_t *s,uint32_t ms,bool ready,int *resets,int *starts) {
 auto_action_t last=AUTO_NOTHING;
 for(uint32_t i=0;i<ms;i+=20) { now+=20; auto_action_t a=auto_start_update(s,now,true,ready);
  if(a==AUTO_RESET) ++*resets;
  if(a==AUTO_START) ++*starts;
  if(a) last=a; }
 return last;
}
int main(void) {
 auto_start_t s; int rs=0,st=0;
 /* power-on starts automatically after the gates */
 now=1000; auto_start_init(&s,now);
 CHECK(auto_start_exec_state(&s)==EXEC_STARTUP_WAIT);
 run(&s,B5_BOOT_DELAY_MS+40,false,&rs,&st); CHECK(rs==1 && st==0 && auto_start_exec_state(&s)==EXEC_WAIT_READY);
 run(&s,2000,false,&rs,&st); CHECK(st==0);                       /* MQTT/readiness gate not passed: stationary */
 run(&s,200,true,&rs,&st);   CHECK(st==1 && auto_start_exec_state(&s)==EXEC_RUNNING);
 /* START while running: no reset, no duplicate */
 CHECK(auto_start_request(&s,now)==START_ALREADY_ACTIVE); run(&s,400,true,&rs,&st);
 CHECK(rs==1 && st==1 && s.phase==AUTO_RUNNING);
 /* STOP holds the car stopped */
 auto_start_stop(&s); run(&s,3000,true,&rs,&st); CHECK(st==1 && auto_start_exec_state(&s)==EXEC_STOPPED);
 /* START after STOP: reset, wait for readiness, then run again */
 CHECK(auto_start_request(&s,now)==START_ACCEPTED && auto_start_exec_state(&s)==EXEC_RESTART_PENDING);
 run(&s,100,false,&rs,&st); CHECK(rs==2 && st==1);
 run(&s,1000,false,&rs,&st); CHECK(st==1 && auto_start_exec_state(&s)==EXEC_RESTART_PENDING);
 run(&s,400,true,&rs,&st); CHECK(st==2 && auto_start_exec_state(&s)==EXEC_RUNNING && !s.restarting);
 /* recoverable uncommanded stop is recorded and retryable */
 auto_start_halt(&s,9,false,now); CHECK(s.phase==AUTO_LATCHED && s.reason==9);
 run(&s,3000,true,&rs,&st); CHECK(st==2);
 CHECK(auto_start_request(&s,now)==START_ACCEPTED); run(&s,2000,true,&rs,&st); CHECK(st==3);
 /* STOP during a pending restart wins */
 auto_start_halt(&s,4,false,now); CHECK(auto_start_request(&s,now)==START_ACCEPTED);
 auto_start_stop(&s); run(&s,3000,true,&rs,&st); CHECK(st==3 && s.phase==AUTO_LATCHED);
 /* STOP during startup prevents automatic movement */
 now=50000; auto_start_init(&s,now); rs=st=0; run(&s,100,true,&rs,&st); auto_start_stop(&s);
 run(&s,5000,true,&rs,&st); CHECK(st==0);
 /* persistent fault (motor stall): STOP does not downgrade it, START rejected, stays stopped */
 now=90000; auto_start_init(&s,now); run(&s,B5_BOOT_DELAY_MS+B5_READY_DELAY_MS+100,true,&rs,&st);
 auto_start_halt(&s,8,true,now); CHECK(s.phase==AUTO_FAULTED && auto_start_exec_state(&s)==EXEC_FAULTED);
 auto_start_stop(&s); CHECK(s.phase==AUTO_FAULTED);
 auto_start_halt(&s,4,false,now); CHECK(s.phase==AUTO_FAULTED && s.reason==8);
 CHECK(auto_start_request(&s,now)==START_REJECTED_FAULT);
 { int st0=st; run(&s,5000,true,&rs,&st); CHECK(st==st0); }
 CHECK(s.last_start==START_REJECTED_FAULT && s.start_count>=1);
 /* automatic restart after an uncommanded, recoverable stop; never after a user STOP or a fault */
 now=200000; auto_start_init(&s,now); rs=st=0; run(&s,B5_BOOT_DELAY_MS+B5_READY_DELAY_MS+100,true,&rs,&st); CHECK(st==1);
 auto_start_halt(&s,10,false,now);
 CHECK(!auto_start_auto_retry(&s,now+B5_AUTO_RETRY_MS-20,true));
 CHECK(!auto_start_auto_retry(&s,now+B5_AUTO_RETRY_MS,false));            /* still moving: wait */
 now+=B5_AUTO_RETRY_MS; CHECK(auto_start_auto_retry(&s,now,true) && s.auto_retries==1);
 CHECK(auto_start_exec_state(&s)==EXEC_RESTART_PENDING);
 run(&s,B5_READY_DELAY_MS+100,true,&rs,&st); CHECK(st==2 && s.phase==AUTO_RUNNING);
 auto_start_stop(&s); now+=10*B5_AUTO_RETRY_MS; CHECK(!auto_start_auto_retry(&s,now,true) && s.phase==AUTO_LATCHED);
 CHECK(auto_start_request(&s,now)==START_ACCEPTED); run(&s,B5_READY_DELAY_MS+100,true,&rs,&st); CHECK(st==3);
 auto_start_halt(&s,8,true,now); now+=10*B5_AUTO_RETRY_MS; CHECK(!auto_start_auto_retry(&s,now,true) && s.phase==AUTO_FAULTED);
 printf(fails?"FAILED (%d)\n":"auto_start: all tests passed\n",fails); return fails!=0;
}
