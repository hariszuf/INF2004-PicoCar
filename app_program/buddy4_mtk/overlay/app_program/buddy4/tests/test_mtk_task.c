#define main original_imu_test_main
#include "test_imu.c"
#undef main
#include "../buddy4_task.c"

static fake_t bus_sensor;
static uint64_t fake_clock;
static int lock_held, lock_failure, open_failure, start_failure;
static int deleted_mutex, deleted_task, device_calls, fail_device, short_device;
static int clock_failure;
ID tk_cre_mtx(const T_CMTX *m){assert(m->mtxatr==TA_INHERIT);return 1;}
ER tk_del_mtx(ID id){assert(id==1);deleted_mutex++;return 0;}
ER tk_loc_mtx(ID id,int timeout){(void)timeout;assert(id==1&&!lock_held);if(lock_failure)return lock_failure;lock_held=1;return 0;}
ER tk_unl_mtx(ID id){assert(id==1&&lock_held);lock_held=0;return 0;}
ID tk_opn_dev(UB *name,UINT mode){assert(strcmp((char*)name,"iica")==0&&mode==(TD_UPDATE|TD_EXCL));return open_failure?open_failure:3;}
ER tk_cls_dev(ID id,UINT option){(void)id;(void)option;return 0;}
ER tk_get_otm(SYSTIM *t){if(clock_failure)return clock_failure;t->hi=(int32_t)((fake_clock/1000)>>32);t->lo=(uint32_t)(fake_clock/1000);return 0;}
ID tk_cre_tsk(const T_CTSK *t){assert(t->stksz==8192&&t->task);return 2;}
ER tk_sta_tsk(ID id,INT code){assert(id==2&&code==0);return start_failure;}
ER tk_del_tsk(ID id){assert(id==2);deleted_task++;return 0;}
ER tk_dly_tsk(RELTIM delay){assert(!lock_held&&delay>0);fake_clock+=(uint64_t)delay*1000;return 0;}
ER tk_swri_dev(ID dd,int start,const void *buf,SZ size,SZ *actual)
{
    assert(dd==3&&!lock_held);device_calls++;
    if(fail_device)return E_IO;
    *actual=short_device?size-1:size;
    if(start==TDN_I2C_EXEC){
        const T_I2C_EXEC *r=buf;
        assert(size==(SZ)sizeof(*r)&&r->sadr==0x1d&&r->snd_size==1);
        assert(r->rcv_size==1||r->rcv_size==6);
        if(r->rcv_size==6)assert(r->snd_data[0]==0xa8);
        for(SZ i=0;i<r->rcv_size;i++)r->rcv_data[i]=bus_sensor.regs[(r->snd_data[0]&0x7f)+i];
    }else{
        const uint8_t *p=buf;
        assert(start==0x1d&&size==2);
        bus_sensor.regs[p[0]]=p[1];
    }
    return 0;
}
static void tick(uint64_t now){fake_clock=now;process_once(now);}
static void calibrated_roll(float deg){
    float x=sinf(0.97f*0.01745329252f);
    float y=sinf((deg+0.61f)*0.01745329252f);
    float z=-sqrtf(1-x*x-y*y);
    float a[]={x,y,z};
    const float bias[]={0.03799f,0.03790f,0.08411f};
    const float scale[]={1.01301f,1.03419f,1.01223f};
    vector(&bus_sensor,a[0]/scale[0]+bias[0],a[1]/scale[1]+bias[1],a[2]/scale[2]+bias[2]);
}
int main(void)
{
    buddy4_task_snapshot_t out;
    assert(buddy4_task_get(&out)==E_OBJ);
    assert(buddy4_task_start()==E_OBJ);
    open_failure=-33;
    assert(buddy4_task_init()==-33&&deleted_mutex==1);
    open_failure=0;
    assert(buddy4_task_init()==0&&buddy4_task_init()==E_OBJ);
    assert(buddy4_task_get(NULL)==E_PAR);
    start_failure=-34;assert(buddy4_task_start()==-34&&deleted_task==1);
    start_failure=0;assert(buddy4_task_start()==0&&buddy4_task_start()==E_OBJ);
    assert(imu.config.roll_offset_deg==0.61f&&imu.config.centre_spacing_cm==11.3f);
    bus_sensor.regs[15]=0x49;bus_sensor.regs[0x27]=8;
    vector(&bus_sensor,0.04f,0.04f,-0.9f);
    tick(20000);assert(buddy4_task_get(&out)==0&&out.valid);
    assert(!out.motion_valid&&!out.yaw_valid&&!out.possible_impact);
    buddy4_task_speed_t speed={true,40000,3,3};
    assert(buddy4_task_submit_speed(&speed)==0);
    tick(40000);assert(yaw.output.valid&&!next.motion_valid);
    tick(60000);assert(!next.motion_valid); /* Duplicate is not another observation. */
    for(uint64_t t=80000;t<=140000;t+=20000){speed.sample_us=t;buddy4_task_submit_speed(&speed);tick(t);}
    assert(next.motion_valid&&next.motion_state==BUDDY4_MOTION_FORWARD);
    tick(239000);assert(next.motion_valid); /* Just below the 100 ms boundary. */
    tick(240000);assert(!next.motion_valid); /* Boundary is stale in the core API. */
    tick(260000);assert(!next.motion_valid&&!next.yaw_valid);
    speed.sample_us=100000;buddy4_task_submit_speed(&speed);tick(280000);
    assert(!next.motion_valid&&!next.yaw_valid);
    bus_sensor.regs[0x27]=0;tick(300000);
    assert(!next.valid&&!next.impact_valid);
    bus_sensor.regs[0x27]=8;short_device=1;tick(320000);
    assert(!next.valid&&next.io_errors==1&&!next.side_event_ended);
    int calls=device_calls;tick(340000);assert(device_calls==calls);
    short_device=0;tick(1020000);assert(next.valid&&next.io_errors==1);
    fake_clock=1061000;assert(buddy4_task_get(&out)==0&&!out.valid&&!out.impact_valid);
    assert(next.valid); /* Getter must expire a copy only. */
    calibrated_roll(6);
    for(uint64_t t=1040000;t<=1640000;t+=20000)tick(t);
    assert(next.side==1&&imu.output.event.peak_confirmed);
    calibrated_roll(0);
    for(uint64_t t=1660000;t<=2100000;t+=20000)tick(t);
    assert(next.completed_events==1&&next.last_event.side==1);
    assert(next.last_event.stable_valid&&next.last_event.median_valid);
    assert(next.last_event.stable_cm>1&&next.last_event.median_cm>1);
    assert(buddy4_task_get(&out)==0&&out.completed_events==1);
    assert(buddy4_task_get(&out)==0&&out.completed_events==1);
    calls=device_calls;clock_failure=-5;tick(2120000);
    assert(device_calls==calls&&!next.valid&&!next.side_event_ended);
    assert(next.completed_events==1&&next.device_error==-5);
    clock_failure=0;
    lock_failure=E_TMOUT;assert(buddy4_task_get(&out)==E_TMOUT);lock_failure=0;
    next.skipped_slots=0;assert(advance_release(0,20000)==40000&&next.skipped_slots==1);
    assert(advance_release(40000,105000)==120000&&next.skipped_slots==4);
    next.skipped_slots=UINT32_MAX-1;(void)advance_release(0,1000000);assert(next.skipped_slots==UINT32_MAX);
    /* Combined bridge never consumes an old or mismatched register pointer. */
    buddy4_bus_t b=buddy4_device_bus(&device);uint8_t reg=15,value;
    assert(b.read(b.context,0x1d,&value,1,false)==-1);
    assert(b.write(b.context,0x1d,&reg,1,true)==1);
    assert(b.read(b.context,0x1e,&value,1,false)==-1);
    assert(b.read(b.context,0x1d,&value,1,false)==-1);
    fail_device=1;assert(b.write(b.context,0x1d,(uint8_t[]){0x20,0x6f},2,false)==-1);
    puts("Native MTK task/device bridge tests passed: lifecycle, combined I/O, stale/reversed/duplicate input, retry and snapshots");
    return 0;
}
