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
//#include <stdio.h>
#include "w5500.h"
#include <SPI.h>


#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03


/**
 * Default function to select chip.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
inline void wizchip_cs_select()
{
    digitalWrite(SS, LOW);
}

/**
 * Default function to deselect chip.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
inline void wizchip_cs_deselect()
{
    digitalWrite(SS, HIGH);
}

/**
 * @brief Default function to read in SPI interface.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
inline uint8_t wizchip_spi_read_byte(void)
{
    return SPI.transfer(0);
}

/**
 * @brief Default function to write in SPI interface.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
inline void wizchip_spi_write_byte(uint8_t wb)
{
    SPI.transfer(wb);
}


////////////////////////////////////////////////////

uint8_t  WIZCHIP_READ(uint32_t AddrSel)
{
    uint8_t ret;

    wizchip_cs_select();

    AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);

    ret = wizchip_spi_read_byte();

    wizchip_cs_deselect();
    return ret;
}

void     WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb )
{
    wizchip_cs_select();

    AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    wizchip_spi_write_byte(wb);

    wizchip_cs_deselect();
}

void     WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i;

    wizchip_cs_select();

    AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    for(i = 0; i < len; i++)
        pBuf[i] = wizchip_spi_read_byte();

    wizchip_cs_deselect();
}

void     WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i;

    wizchip_cs_select();

    AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

    wizchip_spi_write_byte((AddrSel & 0x00FF0000) >> 16);
    wizchip_spi_write_byte((AddrSel & 0x0000FF00) >>  8);
    wizchip_spi_write_byte((AddrSel & 0x000000FF) >>  0);
    for(i = 0; i < len; i++)
        wizchip_spi_write_byte(pBuf[i]);

    wizchip_cs_deselect();
}


uint16_t getSn_TX_FSR(uint8_t sn)
{
    uint16_t val=0,val1=0;

    do
    {
        val1 = WIZCHIP_READ(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        if (val1 != 0)
        {
            val = WIZCHIP_READ(Sn_TX_FSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        }
    } while (val != val1);
    return val;
}


uint16_t getSn_RX_RSR(uint8_t sn)
{
    uint16_t val=0,val1=0;

    do
    {
        val1 = WIZCHIP_READ(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        if (val1 != 0)
        {
            val = WIZCHIP_READ(Sn_RX_RSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        }
    } while (val != val1);
    return val;
}

void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if(len == 0)  return;
    ptr = getSn_TX_WR(sn);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
    WIZCHIP_WRITE_BUF(addrsel,wizdata, len);

    ptr += len;
    setSn_TX_WR(sn,ptr);
}

void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr = 0;
    uint32_t addrsel = 0;

    if(len == 0) return;
    ptr = getSn_RX_RD(sn);
    addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);

    WIZCHIP_READ_BUF(addrsel, wizdata, len);
    ptr += len;

    setSn_RX_RD(sn,ptr);
}


void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
    uint16_t ptr = 0;

    ptr = getSn_RX_RD(sn);
    ptr += len;
    setSn_RX_RD(sn,ptr);
}

#endif
