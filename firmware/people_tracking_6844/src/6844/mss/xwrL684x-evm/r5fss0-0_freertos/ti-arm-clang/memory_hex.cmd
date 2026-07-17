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
    ROW1        : org = 0x00000000     len = 0x00000100     romwidth=32        /* Exception vectors */
    files = { temp/mmwave_demo_xwrL684x_reset_vectors.hex }
    ROW2        : org = 0x00000100     len = 0x0007ff00     romwidth=32        /* TCMA RAM 512 KB in eclipsed mode */
    files = { temp/mmwave_demo_xwrL684x_tcma_ram.hex }
    ROW3        : org = 0x08000000     len = 0x00009000     romwidth=32        /* TCMB RAM 36 KB used by RBL. Do not use for Code and Data Sections */
    files = { temp/mmwave_demo_xwrL684x_tcmb_rbl_reserv.hex }
    ROW4        : org = 0x08009000     len = 0x00037000     romwidth=32        /* TCMB RAM 256 KB */
    files = { temp/mmwave_demo_xwrL684x_tcmb_ram.hex }
    ROW5        : org = 0x88000000     len = 0x00160000     romwidth=32        /* DSS L3 including shared RAMs is 1408 KB */
    files = { temp/mmwave_demo_xwrL684x_dss_l3.hex }
}


/*
 * END OF .cmd file
 */
