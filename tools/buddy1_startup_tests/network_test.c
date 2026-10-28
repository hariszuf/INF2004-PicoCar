/* Real lwIP and real mqtt_utk.c, with a virtual wire and no USB dependency. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/etharp.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/dhcp.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/tcp.h"
#include "lwip/apps/mqtt.h"
#include "netif/ethernet.h"

static u32_t test_now, attempt_time[32];
static unsigned attempts, allocations, force_connect_error, force_alloc_error;
static int app_enabled;
u32_t sys_now(void) { return test_now; }
uint32_t lwip_utk_rand(void) {
    static uint32_t state = 0x12345678;
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    return state;
}
void lwip_utk_assert(const char *message) {
    fprintf(stderr, "lwIP assertion: %s\n", message); abort();
}
int tm_printf(const unsigned char *format, ...) { (void)format; return 0; }
int tm_sprintf(unsigned char *buffer, const unsigned char *format, ...) {
    va_list args; va_start(args, format);
    int n = vsprintf((char *)buffer, (const char *)format, args);
    va_end(args); return n;
}
static err_t observed_connect(mqtt_client_t *client, const ip_addr_t *ip, u16_t port,
    mqtt_connection_cb_t cb, void *arg, const struct mqtt_connect_client_info_t *info) {
    assert(attempts < 32); attempt_time[attempts++] = test_now;
    if(force_connect_error) { force_connect_error--; return ERR_RTE; }
    return mqtt_client_connect(client, ip, port, cb, arg, info);
}
static mqtt_client_t *observed_new(void) {
    allocations++;
    if(force_alloc_error) { force_alloc_error--; return NULL; }
    return mqtt_client_new();
}
#define mqtt_client_connect observed_connect
#define mqtt_client_new observed_new
#include "firmware_mqtt.c"
#undef mqtt_client_connect
#undef mqtt_client_new

void test_buddy_init(void);
void test_queue_telemetry(void);
void test_consume_commands(void);

static struct netif iface;
static struct dhcp lease;
static ip4_addr_t client_ip, broker_ip, router_ip;
static const unsigned char client_mac[6] = {2,0,0,0,0,4};
static const unsigned char peer_mac[6] = {2,0,0,0,0,2};
static u32_t syn_seq, sent_next;
static u16_t client_port;
static unsigned syns, connects, publishes, discovers;
static unsigned status_messages, telemetry_messages;
static unsigned subscribe_id;

static void inspect_mqtt(const unsigned char *p, unsigned length) {
    while(length) {
        unsigned remaining=0, scale=1, offset=1;
        do {
            assert(offset<length && offset<=4);
            remaining+=(p[offset]&127)*scale; scale*=128;
        } while(p[offset++]&128);
        assert(offset+remaining<=length); /* Test messages fit in one TCP segment. */
        const unsigned char *body=p+offset;
        unsigned type=p[0]>>4;
        if(type==1) connects++;
        if(type==8) {
            assert(remaining>=5); subscribe_id=(body[0]<<8)|body[1];
            unsigned topic_length=(body[2]<<8)|body[3];
            assert(topic_length==strlen(MQTT_COMMAND_TOPIC));
            assert(remaining==topic_length+5 && !memcmp(body+4,MQTT_COMMAND_TOPIC,topic_length));
        }
        if(type==3) {
            publishes++; assert(remaining>=2);
            unsigned topic_length=(body[0]<<8)|body[1];
            assert(remaining>=topic_length+2);
            unsigned payload_length=remaining-topic_length-2;
            const unsigned char *payload=body+2+topic_length;
            if(topic_length==strlen(MQTT_STATUS_TOPIC) && !memcmp(body+2,MQTT_STATUS_TOPIC,topic_length)) {
                assert(payload_length>=14 && !memcmp(payload,"heartbeat seq=",14)); status_messages++;
            }
            if(topic_length==strlen(MQTT_TELEMETRY_TOPIC) && !memcmp(body+2,MQTT_TELEMETRY_TOPIC,topic_length)) {
                const char *expected="src=1 type=1 seq=42 value1=7 value2=-9";
                assert(payload_length==strlen(expected) && !memcmp(payload,expected,payload_length));
                telemetry_messages++;
            }
        }
        length-=offset+remaining; p+=offset+remaining;
    }
}

static err_t wire_output(struct netif *n, struct pbuf *p) {
    unsigned char frame[1600];
    (void)n; assert(p->tot_len <= sizeof(frame));
    pbuf_copy_partial(p, frame, p->tot_len, 0);
    const struct eth_hdr *eth = (const void *)frame;
    if(lwip_ntohs(eth->type) != ETHTYPE_IP) return ERR_OK;
    const struct ip_hdr *ip = (const void *)(frame + SIZEOF_ETH_HDR);
    unsigned ip_len = IPH_HL_BYTES(ip);
    if(IPH_PROTO(ip) == IP_PROTO_UDP) {
        const struct udp_hdr *udp = (const void *)((const unsigned char *)ip + ip_len);
        if(lwip_ntohs(udp->dest) == 67) discovers++;
    }
    if(IPH_PROTO(ip) == IP_PROTO_TCP) {
        const struct tcp_hdr *tcp = (const void *)((const unsigned char *)ip + ip_len);
        unsigned len = lwip_ntohs(IPH_LEN(ip)) - ip_len - TCPH_HDRLEN_BYTES(tcp);
        client_port = lwip_ntohs(tcp->src);
        if(TCPH_FLAGS(tcp) & TCP_SYN) {
            syns++; syn_seq = lwip_ntohl(tcp->seqno); sent_next = syn_seq + 1;
        }
        if(len) {
            const unsigned char *payload = (const unsigned char *)tcp + TCPH_HDRLEN_BYTES(tcp);
            sent_next = lwip_ntohl(tcp->seqno) + len;
            inspect_mqtt(payload,len);
        }
    }
    return ERR_OK; /* Peer responses are injected after output returns. */
}
static err_t init_interface(struct netif *n) {
    n->name[0]='t'; n->name[1]='0'; n->mtu=1500;
    n->hwaddr_len=6; memcpy(n->hwaddr, client_mac, 6);
    n->flags=NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
    n->output=etharp_output; n->linkoutput=wire_output;
    return ERR_OK;
}
static void advance(unsigned ms) {
    assert(ms % 10 == 0);
    for(unsigned i=0; i<ms; i+=10) {
        test_now+=10; sys_check_timeouts();
        if(app_enabled) mqtt_utk_poll(&iface);
    }
}
static void deliver(const void *frame, unsigned size) {
    struct pbuf *p=pbuf_alloc(PBUF_RAW, (u16_t)size, PBUF_RAM); assert(p);
    assert(pbuf_take(p, frame, (u16_t)size) == ERR_OK);
    assert(iface.input(p, &iface) == ERR_OK);
}
static struct ip_hdr *ip_header(unsigned char *frame, unsigned size, u8_t proto,
    const ip4_addr_t *src, const ip4_addr_t *dst) {
    struct eth_hdr *eth=(void *)frame;
    memcpy(eth->src.addr, peer_mac, 6); memcpy(eth->dest.addr, client_mac, 6);
    eth->type=lwip_htons(ETHTYPE_IP);
    struct ip_hdr *ip=(void *)(frame+SIZEOF_ETH_HDR);
    IPH_VHL_SET(ip,4,5); IPH_LEN_SET(ip,lwip_htons(size-SIZEOF_ETH_HDR));
    IPH_TTL_SET(ip,64); IPH_PROTO_SET(ip,proto);
    ip->src.addr=src->addr; ip->dest.addr=dst->addr;
    IPH_CHKSUM_SET(ip,inet_chksum(ip,IP_HLEN));
    return ip;
}
static void dhcp_reply(u8_t type) {
    unsigned char frame[SIZEOF_ETH_HDR+IP_HLEN+UDP_HLEN+sizeof(struct dhcp_msg)]={0};
    ip4_addr_t broadcast; IP4_ADDR(&broadcast,255,255,255,255);
    struct ip_hdr *ip=ip_header(frame,sizeof(frame),IP_PROTO_UDP,&router_ip,&broadcast);
    struct udp_hdr *udp=(void *)((unsigned char *)ip+IP_HLEN);
    udp->src=lwip_htons(67); udp->dest=lwip_htons(68);
    udp->len=lwip_htons(UDP_HLEN+sizeof(struct dhcp_msg));
    struct dhcp_msg *msg=(void *)((unsigned char *)udp+UDP_HLEN);
    msg->op=DHCP_BOOTREPLY; msg->htype=1; msg->hlen=6; msg->xid=lwip_htonl(lease.xid);
    msg->yiaddr.addr=client_ip.addr; msg->siaddr.addr=router_ip.addr;
    memcpy(msg->chaddr,client_mac,6); msg->cookie=lwip_htonl(DHCP_MAGIC_COOKIE);
    unsigned char options[]={53,1,type,54,4,172,20,10,1,1,4,255,255,255,240,
        3,4,172,20,10,1,6,4,172,20,10,1,51,4,0,0,14,16,255};
    memcpy(msg->options,options,sizeof(options)); deliver(frame,sizeof(frame));
}
static void arp_reply(void) {
    unsigned char frame[SIZEOF_ETH_HDR+SIZEOF_ETHARP_HDR]={0};
    struct eth_hdr *eth=(void *)frame;
    memcpy(eth->src.addr,peer_mac,6); memcpy(eth->dest.addr,client_mac,6);
    eth->type=lwip_htons(ETHTYPE_ARP);
    struct etharp_hdr *arp=(void *)(frame+SIZEOF_ETH_HDR);
    arp->hwtype=lwip_htons(1); arp->proto=lwip_htons(ETHTYPE_IP);
    arp->hwlen=6; arp->protolen=4; arp->opcode=lwip_htons(ARP_REPLY);
    memcpy(arp->shwaddr.addr,peer_mac,6); memcpy(arp->dhwaddr.addr,client_mac,6);
    memcpy(&arp->sipaddr,&broker_ip,4); memcpy(&arp->dipaddr,&client_ip,4);
    deliver(frame,sizeof(frame));
}
static void tcp_reply(unsigned flags, u32_t seq, u32_t ack, const void *payload, unsigned len) {
    unsigned char frame[SIZEOF_ETH_HDR+IP_HLEN+TCP_HLEN+256]={0}; assert(len<=256);
    unsigned size=SIZEOF_ETH_HDR+IP_HLEN+TCP_HLEN+len;
    struct ip_hdr *ip=ip_header(frame,size,IP_PROTO_TCP,&broker_ip,&client_ip);
    struct tcp_hdr *tcp=(void *)((unsigned char *)ip+IP_HLEN);
    tcp->src=lwip_htons(1883); tcp->dest=lwip_htons(client_port);
    tcp->seqno=lwip_htonl(seq); tcp->ackno=lwip_htonl(ack);
    TCPH_HDRLEN_FLAGS_SET(tcp,5,flags); tcp->wnd=lwip_htons(4096);
    if(len) memcpy((unsigned char *)tcp+TCP_HLEN,payload,len);
    struct pbuf *sum=pbuf_alloc(PBUF_RAW,(u16_t)(TCP_HLEN+len),PBUF_RAM); assert(sum);
    pbuf_take(sum,tcp,(u16_t)(TCP_HLEN+len));
    tcp->chksum=inet_chksum_pseudo(sum,IP_PROTO_TCP,(u16_t)(TCP_HLEN+len),&broker_ip,&client_ip);
    pbuf_free(sum); deliver(frame,size);
}
static void acquire_lease(int late) {
    IP4_ADDR(&client_ip,172,20,10,4); IP4_ADDR(&broker_ip,172,20,10,2);
    IP4_ADDR(&router_ip,172,20,10,1);
    lwip_init();
    assert(netif_add(&iface,NULL,NULL,NULL,NULL,init_interface,ethernet_input));
    netif_set_default(&iface); netif_set_up(&iface);
    dhcp_set_struct(&iface,&lease); assert(dhcp_start(&iface)==ERR_OK);
    assert(lease.state==DHCP_STATE_INIT && discovers==0);
    mqtt_utk_poll(NULL); mqtt_utk_poll(&iface); assert(attempts==0);
    netif_set_link_up(&iface); assert(lease.state==DHCP_STATE_SELECTING && discovers>0);
    if(late) { advance(45000); assert(discovers>1); }
    mqtt_utk_poll(&iface); assert(attempts==0);
    dhcp_reply(DHCP_OFFER); assert(lease.state==DHCP_STATE_REQUESTING);
    dhcp_reply(DHCP_ACK); advance(12000); /* Actual lwIP address-conflict probes. */
    assert(dhcp_supplied_address(&iface)); assert(ip4_addr_cmp(netif_ip4_addr(&iface),&client_ip));
    assert(ip4_addr_cmp(netif_ip4_gw(&iface),&router_ip));
    assert(ip4_addr_cmp(ip_2_ip4(dns_getserver(0)),&router_ip));
}
int main(int argc, char **argv) {
    assert(argc==2); const char *mode=argv[1];
    test_buddy_init();
    acquire_lease(!strcmp(mode,"late_dhcp"));
    if(!strcmp(mode,"dhcp") || !strcmp(mode,"late_dhcp")) {
        printf("PASS %-16s DHCP bound at %u ms with no USB code; DHCP packets=%u\n",mode,test_now,discovers);
        return 0;
    }
    if(!strcmp(mode,"sync_error")) force_connect_error=1;
    if(!strcmp(mode,"alloc_error")) force_alloc_error=1;
    app_enabled=1; advance(3010);
    if(!strcmp(mode,"alloc_error")) {
        assert(allocations==2 && attempts==0);
        advance(2990); assert(attempts==0); advance(10); assert(attempts==1);
        puts("PASS alloc_error      allocation retries after 3000 ms, connect 3000 ms later"); return 0;
    }
    assert(attempts==1);
    if(!strcmp(mode,"sync_error")) {
        u32_t first=attempt_time[0]; advance(2990); assert(attempts==1);
        advance(10); assert(attempts==2 && attempt_time[1]-first==3000);
        puts("PASS sync_error       immediate connection error retried after 3000 ms"); return 0;
    }
    assert(connecting && !connected);
    if(!strcmp(mode,"no_arp")) {
        u32_t first=attempt_time[0]; advance(60000);
        assert(attempts>=2 && syns==0);
        printf("PASS no_arp           missing ARP reply eventually retries, interval=%u ms\n",attempt_time[1]-first);
        return 0;
    }
    arp_reply(); assert(syns>0);
    if(!strcmp(mode,"tcp_timeout")) {
        u32_t first=attempt_time[0]; advance(250000);
        assert(attempts>=2);
        printf("PASS tcp_timeout      unanswered SYN eventually retries, interval=%u ms\n",attempt_time[1]-first);
        return 0;
    }
    if(!strcmp(mode,"tcp_reset")) {
        tcp_reply(TCP_RST|TCP_ACK,0,syn_seq+1,NULL,0);
        assert(!connecting && !connected); u32_t failed=test_now;
        advance(2990); assert(attempts==1); advance(10);
        assert(attempts==2 && attempt_time[1]-failed==3000);
        puts("PASS tcp_reset        real TCP refusal callback, retry after 3000 ms"); return 0;
    }
    tcp_reply(TCP_SYN|TCP_ACK,9000,syn_seq+1,NULL,0); assert(connects>0);
    tcp_reply(TCP_ACK,9001,sent_next,NULL,0);
    if(!strcmp(mode,"connack_timeout")) {
        u32_t first=attempt_time[0]; advance(110000); assert(attempts>=2);
        printf("PASS connack_timeout  TCP accepted but no CONNACK, retry interval=%u ms\n",attempt_time[1]-first);
        return 0;
    }
    if(!strcmp(mode,"connack_refused")) {
        unsigned char refused[]={0x20,2,0,3}; /* Server unavailable. */
        tcp_reply(TCP_ACK|TCP_PSH,9001,sent_next,refused,sizeof(refused));
        assert(!connected && !connecting); u32_t failed=test_now;
        advance(2990); assert(attempts==1); advance(10);
        assert(attempts==2 && attempt_time[1]-failed==3000);
        puts("PASS connack_refused  broker rejection callback, retry after 3000 ms"); return 0;
    }
    assert(!strcmp(mode,"success") || !strcmp(mode,"buddy_io"));
    unsigned char connack[]={0x20,2,0,0};
    tcp_reply(TCP_ACK|TCP_PSH,9001,sent_next,connack,sizeof(connack));
    assert(connected && !connecting && mqtt_client_is_connected(mqtt_client));
    assert(subscribe_id);
    unsigned char suback[]={0x90,3,(unsigned char)(subscribe_id>>8),(unsigned char)subscribe_id,0};
    tcp_reply(TCP_ACK|TCP_PSH,9005,sent_next,suback,sizeof(suback));
    u32_t server_seq=9010;
    if(!strcmp(mode,"buddy_io")) test_queue_telemetry();
    advance(10); tcp_reply(TCP_ACK,server_seq,sent_next,NULL,0); advance(4990);
    assert(status_messages>0 && heartbeat_sequence==1);
    if(!strcmp(mode,"buddy_io")) {
        assert(telemetry_messages>0);
        const char *topics[]={MQTT_COMMAND_TOPIC,MQTT_COMMAND_TOPIC,MQTT_STATUS_TOPIC,MQTT_COMMAND_TOPIC};
        const char *payloads[]={"START","STOP","START","INVALID"};
        for(unsigned i=0;i<4;i++) {
            unsigned char packet[128];
            unsigned topic_len=strlen(topics[i]), payload_len=strlen(payloads[i]);
            unsigned size=4+topic_len+payload_len; assert(size<128);
            packet[0]=0x30; packet[1]=size-2; packet[2]=0; packet[3]=topic_len;
            memcpy(packet+4,topics[i],topic_len); memcpy(packet+4+topic_len,payloads[i],payload_len);
            /* Split START across real TCP receives to exercise MQTT reassembly. */
            unsigned first=i==0 ? size-2 : size;
            tcp_reply(TCP_ACK|TCP_PSH,server_seq,sent_next,packet,first); server_seq+=first;
            if(first<size) {
                tcp_reply(TCP_ACK|TCP_PSH,server_seq,sent_next,packet+first,size-first);
                server_seq+=size-first;
            }
        }
        test_consume_commands();
        puts("PASS buddy_io         real telemetry queue -> MQTT payload; START/STOP -> real command queue/consumer; wrong topic and invalid command rejected");
    } else puts("PASS success          real TCP/MQTT handshake, subscription and heartbeat, no USB code");
    return 0;
}
