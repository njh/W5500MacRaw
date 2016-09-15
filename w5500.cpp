//*****************************************************************************
//
//! \file w5500.c
//! \brief W5500 HAL Interface.
//! \version 1.0.2
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
//*****************************************************************************

#include "w5500.h"
#include <SPI.h>



uint8_t  Wiznet5500::wizchip_read(uint32_t AddrSel)
{
    uint8_t ret;

    wizchip_cs_select();

    AddrSel |= _W5500_SPI_READ_;

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);

    ret = wizchip_spi_read_byte();

    wizchip_cs_deselect();
    return ret;
}

void Wiznet5500::wizchip_write(uint32_t AddrSel, uint8_t wb )
{
    wizchip_cs_select();

    AddrSel |= _W5500_SPI_WRITE_;

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    wizchip_spi_write_byte(wb);

    wizchip_cs_deselect();
}

void Wiznet5500::wizchip_read_buf(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i;

    wizchip_cs_select();

    AddrSel |= _W5500_SPI_READ_;

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    for(i = 0; i < len; i++)
        pBuf[i] = wizchip_spi_read_byte();

    wizchip_cs_deselect();
}

void Wiznet5500::wizchip_write_buf(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i;

    wizchip_cs_select();

    AddrSel |= _W5500_SPI_WRITE_;

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    for(i = 0; i < len; i++)
        wizchip_spi_write_byte(pBuf[i]);

    wizchip_cs_deselect();
}


uint16_t Wiznet5500::getSn_TX_FSR(uint8_t sn)
{
    uint16_t val=0,val1=0;

    do
    {
        val1 = wizchip_read(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        if (val1 != 0)
        {
            val = wizchip_read(Sn_TX_FSR(sn));
            val = (val << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        }
    } while (val != val1);
    return val;
}


uint16_t Wiznet5500::getSn_RX_RSR(uint8_t sn)
{
    uint16_t val=0,val1=0;

    do
    {
        val1 = wizchip_read(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        if (val1 != 0)
        {
            val = wizchip_read(Sn_RX_RSR(sn));
            val = (val << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        }
    } while (val != val1);
    return val;
}

void Wiznet5500::wizchip_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if(len == 0)  return;
    ptr = getSn_TX_WR(sn);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
    wizchip_write_buf(addrsel,wizdata, len);

    ptr += len;
    setSn_TX_WR(sn,ptr);
}

void Wiznet5500::wizchip_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if(len == 0) return;
    ptr = getSn_RX_RD(sn);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);

    wizchip_read_buf(addrsel, wizdata, len);
    ptr += len;

    setSn_RX_RD(sn,ptr);
}


void Wiznet5500::wizchip_recv_ignore(uint8_t sn, uint16_t len)
{
    uint16_t ptr = 0;

    ptr = getSn_RX_RD(sn);
    ptr += len;
    setSn_RX_RD(sn,ptr);
}


void Wiznet5500::wizchip_sw_reset(void)
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

int8_t Wiznet5500::wizchip_init(uint8_t* txsize, uint8_t* rxsize)
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


int8_t Wiznet5500::wizphy_getphylink(void)
{
    int8_t tmp;
    if(getPHYCFGR() & PHYCFGR_LNK_ON)
        tmp = PHY_LINK_ON;
    else
        tmp = PHY_LINK_OFF;
    return tmp;
}

int8_t Wiznet5500::wizphy_getphypmode(void)
{
    int8_t tmp = 0;
    if(getPHYCFGR() & PHYCFGR_OPMDC_PDOWN)
        tmp = PHY_POWER_DOWN;
    else
        tmp = PHY_POWER_NORM;
    return tmp;
}

void Wiznet5500::wizphy_reset(void)
{
    uint8_t tmp = getPHYCFGR();
    tmp &= PHYCFGR_RST;
    setPHYCFGR(tmp);
    tmp = getPHYCFGR();
    tmp |= ~PHYCFGR_RST;
    setPHYCFGR(tmp);
}

void Wiznet5500::wizphy_setphyconf(wiz_PhyConf* phyconf)
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

void Wiznet5500::wizphy_getphyconf(wiz_PhyConf* phyconf)
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

void Wiznet5500::wizphy_getphystat(wiz_PhyConf* phyconf)
{
    uint8_t tmp = getPHYCFGR();
    phyconf->duplex = (tmp & PHYCFGR_DPX_FULL) ? PHY_DUPLEX_FULL : PHY_DUPLEX_HALF;
    phyconf->speed  = (tmp & PHYCFGR_SPD_100) ? PHY_SPEED_100 : PHY_SPEED_10;
}

int8_t Wiznet5500::wizphy_setphypmode(uint8_t pmode)
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


Wiznet5500::Wiznet5500(int8_t cs)
{
    _cs = cs;
}

boolean Wiznet5500::begin(const uint8_t *mac_address)
{
    memcpy(_mac_address, mac_address, 6);

    pinMode(_cs, OUTPUT);
    wizchip_cs_deselect();

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz?
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    wizchip_sw_reset();

    // Set the size of the Rx and Tx buffers
    //wizchip_write(RMSR, RxBufferSize);
    //wizchip_write(TMSR, TxBufferSize);

    // Set our local MAC address
    setSHAR(_mac_address);

    // Open Socket 0 in MACRaw mode
    setSn_MR(0, Sn_MR_MACRAW);
    setSn_CR(0, Sn_CR_OPEN);
    if (getSn_SR(0) != SOCK_MACRAW) {
        // Failed to put socket 0 into MACRaw mode
        return false;
    }

    // Success
    return true;
}

void Wiznet5500::end()
{
    setSn_CR(0, Sn_CR_CLOSE);

    // clear all interrupt of the socket
    setSn_IR(0, 0xFF);

    // Wait for socket to change to closed
    while(getSn_SR(0) != SOCK_CLOSED);
}

uint16_t Wiznet5500::readFrame(uint8_t *buffer, uint16_t bufsize)
{
    uint16_t len = getSn_RX_RSR(0);
    if ( len > 0 )
    {
        uint8_t head[2];
        uint16_t data_len=0;

        wizchip_recv_data(0, head, 2);
        setSn_CR(0, Sn_CR_RECV);

        data_len = head[0];
        data_len = (data_len<<8) + head[1];
        data_len -= 2;

        if(data_len > bufsize)
        {
            // Packet is bigger than buffer - drop the packet
            wizchip_recv_ignore(0, data_len);
            setSn_CR(0, Sn_CR_RECV);
            return 0;
        }

        wizchip_recv_data(0, buffer, data_len);
        setSn_CR(0, Sn_CR_RECV);

        // W5500 doesn't have any built-in MAC address filtering
        if ((buffer[0] & 0x01) || memcmp(&buffer[0], _mac_address, 6) == 0)
        {
            // Addressed to an Ethernet multicast address or our unicast address
            return data_len;
        } else {
            return 0;
        }
    }

    return 0;
}

uint16_t Wiznet5500::sendFrame(const uint8_t *buf, uint16_t len)
{
    // Wait for space in the transmit buffer
    while(1)
    {
        uint16_t freesize = getSn_TX_FSR(0);
        if(getSn_SR(0) == SOCK_CLOSED) {
            return -1;
        }
        if (len <= freesize) break;
    };

    wizchip_send_data(0, buf, len);
    setSn_CR(0, Sn_CR_SEND);

    while(1)
    {
        uint8_t tmp = getSn_IR(0);
        if (tmp & Sn_IR_SENDOK)
        {
            setSn_IR(0, Sn_IR_SENDOK);
            // Packet sent ok
            break;
        }
        else if (tmp & Sn_IR_TIMEOUT)
        {
            setSn_IR(0, Sn_IR_TIMEOUT);
            // There was a timeout
            return -1;
        }
    }

    return len;
}
