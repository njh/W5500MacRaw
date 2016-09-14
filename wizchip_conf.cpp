//****************************************************************************/
//!
//! \file wizchip_conf.c
//! \brief WIZCHIP Config Header File.
//! \version 1.0.1
//! \date 2013/10/21
//! \author MidnightCow
//! \copyright
//!
//! Copyright (c)  2013, WIZnet Co., LTD.
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************/

#include <stddef.h>

#include "wizchip_conf.h"




void wizchip_sw_reset(void)
{
    uint8_t gw[4], sn[4], sip[4];
    uint8_t mac[6];
    getSHAR(mac);
    getGAR(gw);
    getSUBR(sn);
    getSIPR(sip);
    setMR(MR_RST);
    getMR(); // for delay
    setSHAR(mac);
    setGAR(gw);
    setSUBR(sn);
    setSIPR(sip);
}

int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize)
{
    int8_t i;
    int8_t tmp = 0;
    wizchip_sw_reset();
    if(txsize)
    {
        tmp = 0;
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
        {
            tmp += txsize[i];
            if(tmp > 16) return -1;
        }
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
            setSn_TXBUF_SIZE(i, txsize[i]);
    }
    if(rxsize)
    {
        tmp = 0;
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
        {
            tmp += rxsize[i];
            if(tmp > 16) return -1;
        }

        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
            setSn_RXBUF_SIZE(i, rxsize[i]);
    }
    return 0;
}


int8_t wizphy_getphylink(void)
{
    int8_t tmp;
    if(getPHYCFGR() & PHYCFGR_LNK_ON)
        tmp = PHY_LINK_ON;
    else
        tmp = PHY_LINK_OFF;
    return tmp;
}

int8_t wizphy_getphypmode(void)
{
    int8_t tmp = 0;
    if(getPHYCFGR() & PHYCFGR_OPMDC_PDOWN)
        tmp = PHY_POWER_DOWN;
    else
        tmp = PHY_POWER_NORM;
    return tmp;
}

void wizphy_reset(void)
{
    uint8_t tmp = getPHYCFGR();
    tmp &= PHYCFGR_RST;
    setPHYCFGR(tmp);
    tmp = getPHYCFGR();
    tmp |= ~PHYCFGR_RST;
    setPHYCFGR(tmp);
}

void wizphy_setphyconf(wiz_PhyConf* phyconf)
{
    uint8_t tmp = 0;
    if(phyconf->by == PHY_CONFBY_SW)
        tmp |= PHYCFGR_OPMD;
    else
        tmp &= ~PHYCFGR_OPMD;
    if(phyconf->mode == PHY_MODE_AUTONEGO)
        tmp |= PHYCFGR_OPMDC_ALLA;
    else
    {
        if(phyconf->duplex == PHY_DUPLEX_FULL)
        {
            if(phyconf->speed == PHY_SPEED_100)
                tmp |= PHYCFGR_OPMDC_100F;
            else
                tmp |= PHYCFGR_OPMDC_10F;
        }
        else
        {
            if(phyconf->speed == PHY_SPEED_100)
                tmp |= PHYCFGR_OPMDC_100H;
            else
                tmp |= PHYCFGR_OPMDC_10H;
        }
    }
    setPHYCFGR(tmp);
    wizphy_reset();
}

void wizphy_getphyconf(wiz_PhyConf* phyconf)
{
    uint8_t tmp = 0;
    tmp = getPHYCFGR();
    phyconf->by   = (tmp & PHYCFGR_OPMD) ? PHY_CONFBY_SW : PHY_CONFBY_HW;
    switch(tmp & PHYCFGR_OPMDC_ALLA)
    {
    case PHYCFGR_OPMDC_ALLA:
    case PHYCFGR_OPMDC_100FA:
        phyconf->mode = PHY_MODE_AUTONEGO;
        break;
    default:
        phyconf->mode = PHY_MODE_MANUAL;
        break;
    }
    switch(tmp & PHYCFGR_OPMDC_ALLA)
    {
    case PHYCFGR_OPMDC_100FA:
    case PHYCFGR_OPMDC_100F:
    case PHYCFGR_OPMDC_100H:
        phyconf->speed = PHY_SPEED_100;
        break;
    default:
        phyconf->speed = PHY_SPEED_10;
        break;
    }
    switch(tmp & PHYCFGR_OPMDC_ALLA)
    {
    case PHYCFGR_OPMDC_100FA:
    case PHYCFGR_OPMDC_100F:
    case PHYCFGR_OPMDC_10F:
        phyconf->duplex = PHY_DUPLEX_FULL;
        break;
    default:
        phyconf->duplex = PHY_DUPLEX_HALF;
        break;
    }
}

void wizphy_getphystat(wiz_PhyConf* phyconf)
{
    uint8_t tmp = getPHYCFGR();
    phyconf->duplex = (tmp & PHYCFGR_DPX_FULL) ? PHY_DUPLEX_FULL : PHY_DUPLEX_HALF;
    phyconf->speed  = (tmp & PHYCFGR_SPD_100) ? PHY_SPEED_100 : PHY_SPEED_10;
}

int8_t wizphy_setphypmode(uint8_t pmode)
{
    uint8_t tmp = 0;
    tmp = getPHYCFGR();
    if((tmp & PHYCFGR_OPMD)== 0) return -1;
    tmp &= ~PHYCFGR_OPMDC_ALLA;
    if( pmode == PHY_POWER_DOWN)
        tmp |= PHYCFGR_OPMDC_PDOWN;
    else
        tmp |= PHYCFGR_OPMDC_ALLA;
    setPHYCFGR(tmp);
    wizphy_reset();
    tmp = getPHYCFGR();
    if( pmode == PHY_POWER_DOWN)
    {
        if(tmp & PHYCFGR_OPMDC_PDOWN) return 0;
    }
    else
    {
        if(tmp & PHYCFGR_OPMDC_ALLA) return 0;
    }
    return -1;
}

