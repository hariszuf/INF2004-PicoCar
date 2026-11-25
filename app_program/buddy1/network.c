#include "../barrc_types.h"
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "network.h"
#include "../usb_console_compat.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif


#define NETWORK_TASK_PRIORITY  9
#define NETWORK_STACK_SIZE     4096


#if TM_WIFI_CYW43

LOCAL void network_status_task (INT stacd, void * p_exinf)
{
    T_CYW43_UTK_STATUS status;

    uint32_t last_join_complete = 0xFFFFFFFFU;
    uint32_t last_link_up = 0xFFFFFFFFU;

    uint32_t dhcp_reported = 0U;
    uint32_t dns_reported = 0U;

    (void)stacd;
    (void)p_exinf;


    /*
     * Wait until the USB CDC console is connected.
     * This prevents early Wi-Fi / DHCP / DNS messages
     * from being printed before the terminal is ready.
     */
    while (tm_usb_state() < 2)
    {
        tk_dly_tsk(100);
    }

    /*
     * Give the serial terminal a little extra time
     * after USB enumeration.
     */
    tk_dly_tsk(500);


    for (;;)
    {

        /*
         * Read a safe snapshot of the current
         * CYW43439 / lwIP networking state.
         */
        cyw43_utk_get_status(&status);


        /*
         * ---------------------------------------------------------
         * Wi-Fi association status
         * ---------------------------------------------------------
         *
         * Print whenever the join/link state changes.
         */
        if ((status.join_complete != last_join_complete) ||
            (status.link_up != last_link_up))
        {

            tm_printf(
                      (UB *)"[WiFi] join_started=%u "
                      "join_complete=%u "
                      "join_result=%d "
                      "link_up=%u "
                      "link_status=%d "
                      "RSSI=%d "
                      "elapsed=%u ms\n",

                      (uint32_t)status.join_started,
                      (uint32_t)status.join_complete,
                      (int32_t)status.join_result,
                      (uint32_t)status.link_up,
                      (int32_t)status.link_status,
                      (int32_t)status.link_rssi,
                      (uint32_t)status.join_elapsed_ms
                      );


            last_join_complete = status.join_complete;
            last_link_up = status.link_up;
        }


        /*
         * ---------------------------------------------------------
         * DHCP status
         * ---------------------------------------------------------
         *
         * Print once after DHCP successfully reaches
         * its completed state.
         */
        if ((status.dhcp_complete) && (!dhcp_reported))
        {

            tm_printf(
                      (UB *)"[DHCP] result=%d "
                      "ip=%u.%u.%u.%u "
                      "netmask=%u.%u.%u.%u "
                      "gateway=%u.%u.%u.%u "
                      "dns=%u.%u.%u.%u "
                      "elapsed=%u ms\n",

                      (int32_t)status.dhcp_result,

                      (uint32_t)status.dhcp_address[0],
                      (uint32_t)status.dhcp_address[1],
                      (uint32_t)status.dhcp_address[2],
                      (uint32_t)status.dhcp_address[3],

                      (uint32_t)status.dhcp_netmask[0],
                      (uint32_t)status.dhcp_netmask[1],
                      (uint32_t)status.dhcp_netmask[2],
                      (uint32_t)status.dhcp_netmask[3],

                      (uint32_t)status.dhcp_gateway[0],
                      (uint32_t)status.dhcp_gateway[1],
                      (uint32_t)status.dhcp_gateway[2],
                      (uint32_t)status.dhcp_gateway[3],

                      (uint32_t)status.dhcp_dns[0],
                      (uint32_t)status.dhcp_dns[1],
                      (uint32_t)status.dhcp_dns[2],
                      (uint32_t)status.dhcp_dns[3],

                      (uint32_t)status.dhcp_elapsed_ms
                      );


            dhcp_reported = 1U;
        }


        /*
         * ---------------------------------------------------------
         * DNS status
         * ---------------------------------------------------------
         *
         * The µT-Kernel Wi-Fi layer currently tests DNS
         * resolution using CYW43_UTK_DNS_TEST_HOSTNAME.
         *
         * At the moment that is:
         *
         *     example.com
         */
        if ((status.dns_complete) && (!dns_reported))
        {

            tm_printf(
                      (UB *)"[DNS] result=%d "
                      "host=%s "
                      "ip=%u.%u.%u.%u "
                      "elapsed=%u ms\n",

                      (int32_t)status.dns_result,

                      (UB *)CYW43_UTK_DNS_TEST_HOSTNAME,

                      (uint32_t)status.dns_address[0],
                      (uint32_t)status.dns_address[1],
                      (uint32_t)status.dns_address[2],
                      (uint32_t)status.dns_address[3],

                      (uint32_t)status.dns_elapsed_ms
                      );


            dns_reported = 1U;
        }


        /*
         * Poll networking status twice per second.
         *
         * The actual CYW43439/lwIP work is handled by
         * the µT-Kernel Wi-Fi service task. This task
         * only reads the published status snapshot.
         */
        tk_dly_tsk(500);
    }
}


/*
 * Buddy 1 network monitoring task configuration.
 */
LOCAL T_CTSK network_task_config = {
    .itskpri = NETWORK_TASK_PRIORITY,
    .stksz   = NETWORK_STACK_SIZE,
    .task    = network_status_task,
    .tskatr  = TA_HLNG | TA_RNG3
};

#endif


ER buddy1_network_start (void)
{

#if TM_WIFI_CYW43

    ID task_id;


    /*
     * Create the Buddy 1 networking/status task.
     */
    task_id = tk_cre_tsk(
                         &network_task_config
                         );


    if (task_id < E_OK)
    {
        return task_id;
    }


    /*
     * Start the task.
     */
    return tk_sta_tsk(
                      task_id,
                      0
                      );


#else

    /*
     * A non-Wi-Fi build does not require a
     * network monitoring task.
     */
    return E_OK;

#endif
}

/*** end of file ***/
