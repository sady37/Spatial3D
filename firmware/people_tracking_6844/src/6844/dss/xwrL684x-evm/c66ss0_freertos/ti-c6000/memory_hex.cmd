/*----------------------------------------------------------------------------*/
/* memory_hex.cmd                                                             */
/*                                                                            */
/* (c) Texas Instruments 2024, All rights reserved.                           */
/*----------------------------------------------------------------------------*/

/*
 * SPECIFY THE SYSTEM MEMORY MAP, KEEP ALL MEMORY SIZE multiple of 8 bytes(64 bits) to generate ECC
 */

ROMS
{
    ROW1        : org = 0x00800000     len = 0x00060000     romwidth=32        /* Exception vectors */
    files = { temp/ipc_notify_echo_dss_l2.hex }
    ROW2        : org = 0x44000000     len = 0x000003CE     romwidth=32        /* DSS L3 including shared RAMs is 1408 KB */
    files = { temp/ipc_notify_echo_mailbox_hsm.hex }
    ROW3        : org = 0x44000400     len = 0x000003CE     romwidth=32        /* DSS L3 including shared RAMs is 1408 KB */
    files = { temp/ipc_notify_echo_mailbox_r5f.hex }
    ROW4        : org = 0x88000000     len = 0x00200000     romwidth=32        /* TCMA RAM 512 KB in eclipsed mode */
    files = { temp/ipc_notify_echo_dss_l3.hex }
    ROW5        : org = 0xC02E8000     len = 0x00004000     romwidth=32        /* TCMB RAM 36 KB used by RBL. Do not use for Code and Data Sections */
    files = { temp/ipc_notify_echo_user_shm_mem.hex }
    ROW6        : org = 0xC02EC000     len = 0x00004000     romwidth=32        /* TCMB RAM 256 KB */
    files = { temp/ipc_notify_echo_log_shm_mem.hex }
    ROW7        : org = 0xC5000200     len = 0x00001C80     romwidth=32        /* DSS L3 including shared RAMs is 1408 KB */
    files = { temp/ipc_notify_echo_rtos_nortos_ipc_shm_mem.hex }
}


/*
 * END OF .cmd file
 */
