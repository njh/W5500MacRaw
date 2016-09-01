//*****************************************************************************
//
//! \file wizchip_conf.h
//! \brief WIZCHIP Config Header File.
//! \version 1.0.0
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
//!       <2013/10/21> 1st Release
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

/**
 * @defgroup extra_functions 2. WIZnet Extra Functions
 *
 * @brief These functions is optional function. It could be replaced at WIZCHIP I/O function because they were made by WIZCHIP I/O functions.  
 * @details There are functions of configuring WIZCHIP, network, interrupt, phy, network information and timer. \n
 * 
 */

#ifndef  _WIZCHIP_CONF_H_
#define  _WIZCHIP_CONF_H_

#include <stdint.h>
/**
 * @brief Select WIZCHIP.
 * @todo You should select one, \b 5100, \b 5200, \b 5300, \b 5500 or etc. \n\n
 *       ex> <code> #define \_WIZCHIP_      5500 </code>
 */
#ifndef _WIZCHIP_
#define _WIZCHIP_                      5500   // 5100, 5200, 5300, 5500
#endif

#define _WIZCHIP_IO_MODE_NONE_         0x0000
#define _WIZCHIP_IO_MODE_BUS_          0x0100 /**< Bus interface mode */
#define _WIZCHIP_IO_MODE_SPI_          0x0200 /**< SPI interface mode */
//#define _WIZCHIP_IO_MODE_IIC_          0x0400
//#define _WIZCHIP_IO_MODE_SDIO_         0x0800
// Add to
//

#define _WIZCHIP_IO_MODE_BUS_DIR_      (_WIZCHIP_IO_MODE_BUS_ + 1) /**< BUS interface mode for direct  */
#define _WIZCHIP_IO_MODE_BUS_INDIR_    (_WIZCHIP_IO_MODE_BUS_ + 2) /**< BUS interface mode for indirect */

#define _WIZCHIP_IO_MODE_SPI_VDM_      (_WIZCHIP_IO_MODE_SPI_ + 1) /**< SPI interface mode for variable length data*/
#define _WIZCHIP_IO_MODE_SPI_FDM_      (_WIZCHIP_IO_MODE_SPI_ + 2) /**< SPI interface mode for fixed length data mode*/


#if   (_WIZCHIP_ == 5100)
   #define _WIZCHIP_ID_                "W5100\0"
/**
 * @brief Define interface mode.
 * @todo you should select interface mode as chip. Select one of @ref \_WIZCHIP_IO_MODE_SPI_ , @ref \_WIZCHIP_IO_MODE_BUS_DIR_ or @ref \_WIZCHIP_IO_MODE_BUS_INDIR_
 */
// #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_BUS_DIR_
// #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_BUS_INDIR_
   #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_SPI_

//A20150601 : Define the unit of IO DATA.   
   typedef   uint8_t   iodata_t;
//A20150401 : Indclude W5100.h file   
   #include "W5100/w5100.h"

#elif (_WIZCHIP_ == 5200)
   #define _WIZCHIP_ID_                "W5200\0"
/**
 * @brief Define interface mode.
 * @todo you should select interface mode as chip. Select one of @ref \_WIZCHIP_IO_MODE_SPI_ or @ref \_WIZCHIP_IO_MODE_BUS_INDIR_
 */
#ifndef _WIZCHIP_IO_MODE_
// #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_BUS_INDIR_
   #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_SPI_
#endif
//A20150601 : Define the unit of IO DATA.   
   typedef   uint8_t   iodata_t;
   #include "W5200/w5200.h"
#elif (_WIZCHIP_ == 5500)
  #define _WIZCHIP_ID_                 "W5500\0"
  
/**
 * @brief Define interface mode. \n
 * @todo Should select interface mode as chip. 
 *        - @ref \_WIZCHIP_IO_MODE_SPI_ \n
 *          -@ref \_WIZCHIP_IO_MODE_SPI_VDM_ : Valid only in @ref \_WIZCHIP_ == 5500 \n
 *          -@ref \_WIZCHIP_IO_MODE_SPI_FDM_ : Valid only in @ref \_WIZCHIP_ == 5500 \n
 *        - @ref \_WIZCHIP_IO_MODE_BUS_ \n
 *          - @ref \_WIZCHIP_IO_MODE_BUS_DIR_ \n
 *          - @ref \_WIZCHIP_IO_MODE_BUS_INDIR_ \n
 *        - Others will be defined in future. \n\n
 *        ex> <code> #define \_WIZCHIP_IO_MODE_ \_WIZCHIP_IO_MODE_SPI_VDM_ </code>
 *       
 */
#ifndef _WIZCHIP_IO_MODE_
   //#define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_SPI_FDM_
   #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_SPI_VDM_
#endif
//A20150601 : Define the unit of IO DATA.   
   typedef   uint8_t   iodata_t;
   #include "W5500/w5500.h"
#elif ( _WIZCHIP_ == 5300)
   #define _WIZCHIP_ID_                 "W5300\0"
/**
 * @brief Define interface mode.
 * @todo you should select interface mode as chip. Select one of @ref \_WIZCHIP_IO_MODE_SPI_ , @ref \_WIZCHIP_IO_MODE_BUS_DIR_ or @ref \_WIZCHIP_IO_MODE_BUS_INDIR_
 */
#ifndef _WIZCHIP_IO_MODE_
//   #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_BUS_DIR_
 #define _WIZCHIP_IO_MODE_           _WIZCHIP_IO_MODE_BUS_INDIR_
#endif

//A20150601 : Define the unit and bus width of IO DATA. 
   /**
    * @brief Select the data width 8 or 16 bits.
    * @todo you should select the bus width. Select one of 8 or 16.
    */
   #ifndef _WIZCHIP_IO_BUS_WIDTH_
   #define _WIZCHIP_IO_BUS_WIDTH_       8  // 16
   #endif
   #if _WIZCHIP_IO_BUS_WIDTH_ == 8
      typedef   uint8_t   iodata_t;
   #elif _WIZCHIP_IO_BUS_WIDTH_ == 16
      typedef   uint16_t   iodata_t;
   #else
      #error "Unknown _WIZCHIP_IO_BUS_WIDTH_. It should be 8 or 16."	
   #endif
//
   #include "W5300/w5300.h"
#else
   #error "Unknown defined _WIZCHIP_. You should define one of 5100, 5200, and 5500 !!!"
#endif

#ifndef _WIZCHIP_IO_MODE_
   #error "Undefined _WIZCHIP_IO_MODE_. You should define it !!!"
#endif

/**
 * @brief Define I/O base address when BUS IF mode.
 * @todo Should re-define it to fit your system when BUS IF Mode (@ref \_WIZCHIP_IO_MODE_BUS_,
 *       @ref \_WIZCHIP_IO_MODE_BUS_DIR_, @ref \_WIZCHIP_IO_MODE_BUS_INDIR_). \n\n
 *       ex> <code> #define \_WIZCHIP_IO_BASE_      0x00008000 </code>
 */
#ifndef _WIZCHIP_IO_BASE_
#define _WIZCHIP_IO_BASE_              0x00000000  // 0x8000
#endif

//M20150401 : Typing Error
//#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_BUS
#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_BUS_
   #ifndef _WIZCHIP_IO_BASE_
      #error "You should be define _WIZCHIP_IO_BASE to fit your system memory map."
   #endif
#endif   

#if _WIZCHIP_ > 5100
   #define _WIZCHIP_SOCK_NUM_   8   ///< The count of independant socket of @b WIZCHIP
#else
   #define _WIZCHIP_SOCK_NUM_   4   ///< The count of independant socket of @b WIZCHIP
#endif      


/********************************************************
* WIZCHIP BASIC IF functions for SPI, SDIO, I2C , ETC.
*********************************************************/
/**
 * @ingroup DATA_TYPE
 * @brief The set of callback functions for W5500:@ref WIZCHIP_IO_Functions W5200:@ref WIZCHIP_IO_Functions_W5200
 */
typedef struct __WIZCHIP
{
   uint16_t  if_mode;               ///< host interface mode
   uint8_t   id[6];                 ///< @b WIZCHIP ID such as @b 5100, @b 5200, @b 5500, and so on.
   /**
    * The set of critical section callback func.
    */
   struct _CRIS
   {
      void (*_enter)  (void);       ///< crtical section enter 
      void (*_exit) (void);         ///< critial section exit  
   }CRIS;  
   /**
    *  The set of @ref \_WIZCHIP_ select control callback func.
    */
   struct _CS
   {
      void (*_select)  (void);      ///< @ref \_WIZCHIP_ selected
      void (*_deselect)(void);      ///< @ref \_WIZCHIP_ deselected
   }CS;  
   /**
    * The set of interface IO callback func.
    */
   union _IF
   {	 
      /**
       * For BUS interface IO
       */
      //M20156501 : Modify the function name for integrating with W5300
      //struct
      //{
      //   uint8_t  (*_read_byte)  (uint32_t AddrSel);
      //   void     (*_write_byte) (uint32_t AddrSel, uint8_t wb);
      //}BUS;      
      struct
      {
         iodata_t  (*_read_data)   (uint32_t AddrSel);
         void      (*_write_data)  (uint32_t AddrSel, iodata_t wb);
      }BUS;      

      /**
       * For SPI interface IO
       */
      struct
      {
         uint8_t (*_read_byte)   (void);
         void    (*_write_byte)  (uint8_t wb);
         void    (*_read_burst)  (uint8_t* pBuf, uint16_t len);
         void    (*_write_burst) (uint8_t* pBuf, uint16_t len);
      }SPI;
      // To be added
      //
   }IF;
}_WIZCHIP;

extern _WIZCHIP  WIZCHIP;

/**
 * @ingroup DATA_TYPE
 *  WIZCHIP control type enumration used in @ref ctlwizchip().
 */
typedef enum
{
   CW_RESET_WIZCHIP,   ///< Resets WIZCHIP by softly
   CW_INIT_WIZCHIP,    ///< Initializes to WIZCHIP with SOCKET buffer size 2 or 1 dimension array typed uint8_t.
   CW_GET_INTERRUPT,   ///< Get Interrupt status of WIZCHIP
   CW_CLR_INTERRUPT,   ///< Clears interrupt
   CW_SET_INTRMASK,    ///< Masks interrupt
   CW_GET_INTRMASK,    ///< Get interrupt mask
   CW_SET_INTRTIME,    ///< Set interval time between the current and next interrupt. 
   CW_GET_INTRTIME,    ///< Set interval time between the current and next interrupt. 
   CW_GET_ID,          ///< Gets WIZCHIP name.

//D20150601 : For no modification your application code
//#if _WIZCHIP_ ==  5500
   CW_RESET_PHY,       ///< Resets internal PHY. Valid Only W5500
   CW_SET_PHYCONF,     ///< When PHY configured by internal register, PHY operation mode (Manual/Auto, 10/100, Half/Full). Valid Only W5000
   CW_GET_PHYCONF,     ///< Get PHY operation mode in internal register. Valid Only W5500
   CW_GET_PHYSTATUS,   ///< Get real PHY status on operating. Valid Only W5500
   CW_SET_PHYPOWMODE,  ///< Set PHY power mode as normal and down when PHYSTATUS.OPMD == 1. Valid Only W5500
//#endif
//D20150601 : For no modification your application code
//#if _WIZCHIP_ == 5200 || _WIZCHIP_ == 5500
   CW_GET_PHYPOWMODE,  ///< Get PHY Power mode as down or normal, Valid Only W5100, W5200
   CW_GET_PHYLINK      ///< Get PHY Link status, Valid Only W5100, W5200
//#endif
}ctlwizchip_type;

/**
 * @ingroup DATA_TYPE
 *  Network control type enumration used in @ref ctlnetwork().
 */
typedef enum
{
   CN_SET_NETINFO,  ///< Set Network with @ref wiz_NetInfo
   CN_GET_NETINFO,  ///< Get Network with @ref wiz_NetInfo
   CN_SET_NETMODE,  ///< Set network mode as WOL, PPPoE, Ping Block, and Force ARP mode
   CN_GET_NETMODE,  ///< Get network mode as WOL, PPPoE, Ping Block, and Force ARP mode
   CN_SET_TIMEOUT,  ///< Set network timeout as retry count and time.
   CN_GET_TIMEOUT,  ///< Get network timeout as retry count and time.
}ctlnetwork_type;

/**
 * @ingroup DATA_TYPE
 *  Interrupt kind when CW_SET_INTRRUPT, CW_GET_INTERRUPT, CW_SET_INTRMASK
 *  and CW_GET_INTRMASK is used in @ref ctlnetwork().
 *  It can be used with OR operation.
 */
typedef enum
{
#if   _WIZCHIP_ == 5500
   IK_WOL               = (1 << 4),   ///< Wake On Lan by receiving the magic packet. Valid in W500.
#elif _WIZCHIP_ == 5300
   IK_FMTU              = (1 << 4),   ///< Received a ICMP message (Fragment MTU)   
#endif   

   IK_PPPOE_TERMINATED  = (1 << 5),   ///< PPPoE Disconnected

#if _WIZCHIP_ != 5200
   IK_DEST_UNREACH      = (1 << 6),   ///< Destination IP & Port Unreachable, No use in W5200
#endif   

   IK_IP_CONFLICT       = (1 << 7),   ///< IP conflict occurred

   IK_SOCK_0            = (1 << 8),   ///< Socket 0 interrupt
   IK_SOCK_1            = (1 << 9),   ///< Socket 1 interrupt
   IK_SOCK_2            = (1 << 10),  ///< Socket 2 interrupt
   IK_SOCK_3            = (1 << 11),  ///< Socket 3 interrupt
#if _WIZCHIP_ > 5100   
   IK_SOCK_4            = (1 << 12),  ///< Socket 4 interrupt, No use in 5100
   IK_SOCK_5            = (1 << 13),  ///< Socket 5 interrupt, No use in 5100
   IK_SOCK_6            = (1 << 14),  ///< Socket 6 interrupt, No use in 5100
   IK_SOCK_7            = (1 << 15),  ///< Socket 7 interrupt, No use in 5100
#endif   

#if _WIZCHIP_ > 5100
   IK_SOCK_ALL          = (0xFF << 8) ///< All Socket interrupt
#else
   IK_SOCK_ALL          = (0x0F << 8) ///< All Socket interrupt
#endif      
}intr_kind;

#define PHY_CONFBY_HW            0     ///< Configured PHY operation mode by HW pin
#define PHY_CONFBY_SW            1     ///< Configured PHY operation mode by SW register   
#define PHY_MODE_MANUAL          0     ///< Configured PHY operation mode with user setting.
#define PHY_MODE_AUTONEGO        1     ///< Configured PHY operation mode with auto-negotiation
#define PHY_SPEED_10             0     ///< Link Speed 10
#define PHY_SPEED_100            1     ///< Link Speed 100
#define PHY_DUPLEX_HALF          0     ///< Link Half-Duplex
#define PHY_DUPLEX_FULL          1     ///< Link Full-Duplex
#define PHY_LINK_OFF             0     ///< Link Off
#define PHY_LINK_ON              1     ///< Link On
#define PHY_POWER_NORM           0     ///< PHY power normal mode
#define PHY_POWER_DOWN           1     ///< PHY power down mode 


#if _WIZCHIP_ == 5500 
/**
 * @ingroup DATA_TYPE
 *  It configures PHY configuration when CW_SET PHYCONF or CW_GET_PHYCONF in W5500,  
 *  and it indicates the real PHY status configured by HW or SW in all WIZCHIP. \n
 *  Valid only in W5500.
 */
typedef struct wiz_PhyConf_t
{
      uint8_t by;       ///< set by @ref PHY_CONFBY_HW or @ref PHY_CONFBY_SW
      uint8_t mode;     ///< set by @ref PHY_MODE_MANUAL or @ref PHY_MODE_AUTONEGO
      uint8_t speed;    ///< set by @ref PHY_SPEED_10 or @ref PHY_SPEED_100
      uint8_t duplex;   ///< set by @ref PHY_DUPLEX_HALF @ref PHY_DUPLEX_FULL 
      //uint8_t power;  ///< set by @ref PHY_POWER_NORM or @ref PHY_POWER_DOWN
      //uint8_t link;   ///< Valid only in CW_GET_PHYSTATUS. set by @ref PHY_LINK_ON or PHY_DUPLEX_OFF 
   }wiz_PhyConf;
#endif   



/* 
 * The following functions are implemented for internal use. 
 * but You can call these functions for code size reduction instead of ctlwizchip() and ctlnetwork().
 */
 
/**
 * @ingroup extra_functions
 * @brief Reset WIZCHIP by softly.
 */ 
void   wizchip_sw_reset(void);

/**
 * @ingroup extra_functions
 * @brief Initializes WIZCHIP with socket buffer size
 * @param txsize Socket tx buffer sizes. If null, initialized the default size 2KB.
 * @param rxsize Socket rx buffer sizes. If null, initialized the default size 2KB.
 * @return 0 : succcess \n
 *        -1 : fail. Invalid buffer size
 */
int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize);

#if _WIZCHIP_ > 5100
   int8_t wizphy_getphylink(void);              ///< get the link status of phy in WIZCHIP. No use in W5100
   int8_t wizphy_getphypmode(void);             ///< get the power mode of PHY in WIZCHIP. No use in W5100
#endif

#if _WIZCHIP_ == 5500
   void   wizphy_reset(void);                   ///< Reset phy. Vailid only in W5500
/**
 * @ingroup extra_functions
 * @brief Set the phy information for WIZCHIP without power mode
 * @param phyconf : @ref wiz_PhyConf
 */
   void   wizphy_setphyconf(wiz_PhyConf* phyconf);  
 /**
 * @ingroup extra_functions
 * @brief Get phy configuration information.
 * @param phyconf : @ref wiz_PhyConf
 */
   void   wizphy_getphyconf(wiz_PhyConf* phyconf); 
 /**
 * @ingroup extra_functions
 * @brief Get phy status.
 * @param phyconf : @ref wiz_PhyConf
 */ 
   void   wizphy_getphystat(wiz_PhyConf* phyconf);
 /**
 * @ingroup extra_functions
 * @brief set the power mode of phy inside WIZCHIP. Refer to @ref PHYCFGR in W5500, @ref PHYSTATUS in W5200
 * @param pmode Settig value of power down mode.
 */   
   int8_t wizphy_setphypmode(uint8_t pmode);    
#endif

#endif   // _WIZCHIP_CONF_H_
