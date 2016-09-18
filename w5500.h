//*****************************************************************************
//
//! \file w5500.h
//! \brief W5500 HAL Header File.
//! \version 1.0.0
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

//

#ifndef  _W5500_H_
#define  _W5500_H_

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

#define _W5500_SPI_READ_			   (0x00 << 2) //< SPI interface Read operation in Control Phase
#define _W5500_SPI_WRITE_			   (0x01 << 2) //< SPI interface Write operation in Control Phase

#define WIZCHIP_CREG_BLOCK          (0x00 << 3)    //< Common register block
#define WIZCHIP_SREG_BLOCK(N)       ((1+4*N) << 3) //< Socket N register block
#define WIZCHIP_TXBUF_BLOCK(N)      ((2+4*N) << 3) //< Socket N Tx buffer address block
#define WIZCHIP_RXBUF_BLOCK(N)      ((3+4*N) << 3) //< Socket N Rx buffer address block

#define WIZCHIP_OFFSET_INC(ADDR, N)    (ADDR + (N<<8)) //< Increase offset address


//----------------------------- W5500 Common Registers IOMAP -----------------------------
/**
 * @brief Mode Register address(R/W)\n
 * @ref MR is used for S/W reset, ping block mode, PPPoE mode and etc.
 * @details Each bit of @ref MR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>RST</td> <td>Reserved</td> <td>WOL</td> <td>PB</td> <td>PPPoE</td> <td>Reserved</td> <td>FARP</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref MR_RST		 	: Reset
 * - \ref MR_WOL       	: Wake on LAN
 * - \ref MR_PB         : Ping block
 * - \ref MR_PPPOE      : PPPoE mode
 * - \ref MR_FARP			: Force ARP mode
 */
#define MR                 ((0x0000 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Gateway IP Register address(R/W)
 * @details @ref GAR configures the default gateway address.
 */
#define GAR                ((0x0001 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Subnet mask Register address(R/W)
 * @details @ref SUBR configures the subnet mask address.
 */
#define SUBR               ((0x0005 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Source MAC Register address(R/W)
 * @details @ref SHAR configures the source hardware address.
 */
#define SHAR               ((0x0009 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Source IP Register address(R/W)
 * @details @ref SIPR configures the source IP address.
 */
#define SIPR               ((0x000F << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Set Interrupt low level timer register address(R/W)
 * @details @ref INTLEVEL configures the Interrupt Assert Time.
 */
#define INTLEVEL           ((0x0013 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Interrupt Register(R/W)
 * @details @ref IR indicates the interrupt status. Each bit of @ref IR will be still until the bit will be written to by the host.
 * If @ref IR is not equal to x00 INTn PIN is asserted to low until it is x00\n\n
 * Each bit of @ref IR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>CONFLICT</td> <td>UNREACH</td> <td>PPPoE</td> <td>MP</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IR_CONFLICT : IP conflict
 * - \ref IR_UNREACH  : Destination unreachable
 * - \ref IR_PPPoE	  : PPPoE connection close
 * - \ref IR_MP		  : Magic packet
 */
#define IR                 ((0x0015 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Interrupt mask register(R/W)
 * @details @ref _IMR_ is used to mask interrupts. Each bit of @ref _IMR_ corresponds to each bit of @ref IR.
 * When a bit of @ref _IMR_ is and the corresponding bit of @ref IR is  an interrupt will be issued. In other words,
 * if a bit of @ref _IMR_ is  an interrupt will not be issued even if the corresponding bit of @ref IR is \n\n
 * Each bit of @ref _IMR_ defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>IM_IR7</td> <td>IM_IR6</td> <td>IM_IR5</td> <td>IM_IR4</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IM_IR7 : IP Conflict Interrupt Mask
 * - \ref IM_IR6 : Destination unreachable Interrupt Mask
 * - \ref IM_IR5 : PPPoE Close Interrupt Mask
 * - \ref IM_IR4 : Magic Packet Interrupt Mask
 */
#define _IMR_                ((0x0016 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Socket Interrupt Register(R/W)
 * @details @ref SIR indicates the interrupt status of Socket.\n
 * Each bit of @ref SIR be still until @ref Sn_IR is cleared by the host.\n
 * If @ref Sn_IR is not equal to x00 the n-th bit of @ref SIR is and INTn PIN is asserted until @ref SIR is x00 */
#define SIR                ((0x0017 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Socket Interrupt Mask Register(R/W)
 * @details Each bit of @ref SIMR corresponds to each bit of @ref SIR.
 * When a bit of @ref SIMR is and the corresponding bit of @ref SIR is  Interrupt will be issued.
 * In other words, if a bit of @ref SIMR is  an interrupt will be not issued even if the corresponding bit of @ref SIR is
 */
#define SIMR               ((0x0018 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Timeout register address( 1 is 100us )(R/W)
 * @details @ref _RTR_ configures the retransmission timeout period. The unit of timeout period is 100us and the default of @ref _RTR_ is x07D0.
 * And so the default timeout period is 200ms(100us X 2000). During the time configured by @ref _RTR_, W5500 waits for the peer response
 * to the packet that is transmitted by \ref Sn_CR (CONNECT, DISCON, CLOSE, SEND, SEND_MAC, SEND_KEEP command).
 * If the peer does not respond within the @ref _RTR_ time, W5500 retransmits the packet or issues timeout.
 */
#define _RTR_                ((0x0019 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Retry count register(R/W)
 * @details @ref _RCR_ configures the number of time of retransmission.
 * When retransmission occurs as many as ref _RCR_+1 Timeout interrupt is issued (@ref Sn_IR_TIMEOUT = '1').
 */
#define _RCR_                ((0x001B << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PPP LCP Request Timer register  in PPPoE mode(R/W)
 * @details @ref PTIMER configures the time for sending LCP echo request. The unit of time is 25ms.
 */
#define PTIMER             ((0x001C << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PPP LCP Magic number register  in PPPoE mode(R/W)
 * @details @ref PMAGIC configures the 4bytes magic number to be used in LCP negotiation.
 */
#define PMAGIC             ((0x001D << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PPP Destination MAC Register address(R/W)
 * @details @ref PHAR configures the PPPoE server hardware address that is acquired during PPPoE connection process.
 */
#define PHAR                ((0x001E << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PPP Session Identification Register(R/W)
 * @details @ref PSID configures the PPPoE sever session ID acquired during PPPoE connection process.
 */
#define PSID               ((0x0024 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PPP Maximum Segment Size(MSS) register(R/W)
 * @details @ref PMRU configures the maximum receive unit of PPPoE.
 */
#define PMRU               ((0x0026 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Unreachable IP register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref IR_UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR indicates
 * the destination IP address & port number respectively.
 */
#define UIPR               ((0x0028 << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief Unreachable Port register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref IR_UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR
 * indicates the destination IP address & port number respectively.
 */
#define UPORTR              ((0x002C << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief PHY Status Register(R/W)
 * @details @ref PHYCFGR configures PHY operation mode and resets PHY. In addition, @ref PHYCFGR indicates the status of PHY such as duplex, Speed, Link.
 */
#define PHYCFGR            ((0x002E << 8) + WIZCHIP_CREG_BLOCK)

/**
 * @brief chip version register address(R)
 * @details @ref VERSIONR always indicates the W5500 version as @b 0x04.
 */
#define VERSIONR           ((0x0039 << 8) + WIZCHIP_CREG_BLOCK)


//----------------------------- W5500 Socket Registers IOMAP -----------------------------
/**
 * @brief socket Mode register(R/W)
 * @details @ref Sn_MR configures the option or protocol type of Socket n.\n\n
 * Each bit of @ref Sn_MR defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>MULTI/MFEN</td> <td>BCASTB</td> <td>ND/MC/MMB</td> <td>UCASTB/MIP6B</td> <td>Protocol[3]</td> <td>Protocol[2]</td> <td>Protocol[1]</td> <td>Protocol[0]</td> </tr>
 * </table>
 * - @ref Sn_MR_MULTI	: Support UDP Multicasting
 * - @ref Sn_MR_BCASTB	: Broadcast block <b>in UDP Multicasting</b>
 * - @ref Sn_MR_ND		: No Delayed Ack(TCP) flag
 * - @ref Sn_MR_MC   	: IGMP version used <b>in UDP mulitcasting</b>
 * - @ref Sn_MR_MMB    	: Multicast Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - @ref Sn_MR_UCASTB	: Unicast Block <b>in UDP Multicating</b>
 * - @ref Sn_MR_MIP6B   : IPv6 packet Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - <b>Protocol</b>
 * <table>
 * 		<tr>   <td><b>Protocol[3]</b></td> <td><b>Protocol[2]</b></td> <td><b>Protocol[1]</b></td> <td><b>Protocol[0]</b></td> <td>@b Meaning</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>0</td> <td>Closed</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>1</td> <td>TCP</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>1</td> <td>0</td> <td>UDP</td>   </tr>
 * 		<tr>   <td>0</td> <td>1</td> <td>0</td> <td>0</td> <td>MACRAW</td>   </tr>
 * </table>
 *	- @ref Sn_MR_MACRAW	: MAC LAYER RAW SOCK \n
 *  - @ref Sn_MR_UDP		: UDP
 *  - @ref Sn_MR_TCP		: TCP
 *  - @ref Sn_MR_CLOSE	: Unused socket
 *  @note MACRAW mode should be only used in Socket 0.
 */
#define Sn_MR(N)           ((0x0000 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Socket command register(R/W)
 * @details This is used to set the command for Socket n such as OPEN, CLOSE, CONNECT, LISTEN, SEND, and RECEIVE.\n
 * After W5500 accepts the command, the @ref Sn_CR register is automatically cleared to 0x00.
 * Even though @ref Sn_CR is cleared to 0x00, the command is still being processed.\n
 * To check whether the command is completed or not, please check the @ref Sn_IR or @ref Sn_SR.
 * - @ref Sn_CR_OPEN 		: Initialize or open socket.
 * - @ref Sn_CR_LISTEN 		: Wait connection request in TCP mode(<b>Server mode</b>)
 * - @ref Sn_CR_CONNECT 	: Send connection request in TCP mode(<b>Client mode</b>)
 * - @ref Sn_CR_DISCON 		: Send closing request in TCP mode.
 * - @ref Sn_CR_CLOSE   	: Close socket.
 * - @ref Sn_CR_SEND    	: Update TX buffer pointer and send data.
 * - @ref Sn_CR_SEND_MAC	: Send data with MAC address, so without ARP process.
 * - @ref Sn_CR_SEND_KEEP 	: Send keep alive message.
 * - @ref Sn_CR_RECV		: Update RX buffer pointer and receive data.
 */
#define Sn_CR(N)           ((0x0001 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Socket interrupt register(R)
 * @details @ref Sn_IR indicates the status of Socket Interrupt such as establishment, termination, receiving data, timeout).\n
 * When an interrupt occurs and the corresponding bit of @ref Sn_IMR is  the corresponding bit of @ref Sn_IR becomes \n
 * In order to clear the @ref Sn_IR bit, the host should write the bit to \n
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>SEND_OK</td> <td>TIMEOUT</td> <td>RECV</td> <td>DISCON</td> <td>CON</td> </tr>
 * </table>
 * - \ref Sn_IR_SENDOK : <b>SEND_OK Interrupt</b>
 * - \ref Sn_IR_TIMEOUT : <b>TIMEOUT Interrupt</b>
 * - \ref Sn_IR_RECV : <b>RECV Interrupt</b>
 * - \ref Sn_IR_DISCON : <b>DISCON Interrupt</b>
 * - \ref Sn_IR_CON : <b>CON Interrupt</b>
 */
#define Sn_IR(N)           ((0x0002 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Socket status register(R)
 * @details @ref Sn_SR indicates the status of Socket n.\n
 * The status of Socket n is changed by @ref Sn_CR or some special control packet as SYN, FIN packet in TCP.
 * @par Normal status
 * - @ref SOCK_CLOSED 		: Closed
 * - @ref SOCK_INIT   		: Initiate state
 * - @ref SOCK_LISTEN    	: Listen state
 * - @ref SOCK_ESTABLISHED 	: Success to connect
 * - @ref SOCK_CLOSE_WAIT   : Closing state
 * - @ref SOCK_UDP   		: UDP socket
 * - @ref SOCK_MACRAW  		: MAC raw mode socket
 *@par Temporary status during changing the status of Socket n.
 * - @ref SOCK_SYNSENT   	: This indicates Socket n sent the connect-request packet (SYN packet) to a peer.
 * - @ref SOCK_SYNRECV    	: It indicates Socket n successfully received the connect-request packet (SYN packet) from a peer.
 * - @ref SOCK_FIN_WAIT		: Connection state
 * - @ref SOCK_CLOSING		: Closing state
 * - @ref SOCK_TIME_WAIT	: Closing state
 * - @ref SOCK_LAST_ACK 	: Closing state
 */
#define Sn_SR(N)           ((0x0003 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief source port register(R/W)
 * @details @ref Sn_PORT configures the source port number of Socket n.
 * It is valid when Socket n is used in TCP/UDP mode. It should be set before OPEN command is ordered.
 */
#define Sn_PORT(N)         ((0x0004 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Peer MAC register address(R/W)
 * @details @ref Sn_DHAR configures the destination hardware address of Socket n when using SEND_MAC command in UDP mode or
 * it indicates that it is acquired in ARP-process by CONNECT/SEND command.
 */
#define Sn_DHAR(N)         ((0x0006 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Peer IP register address(R/W)
 * @details @ref Sn_DIPR configures or indicates the destination IP address of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In TCP client mode, it configures an IP address of TCP serverbefore CONNECT command.
 * In TCP server mode, it indicates an IP address of TCP clientafter successfully establishing connection.
 * In UDP mode, it configures an IP address of peer to be received the UDP packet by SEND or SEND_MAC command.
 */
#define Sn_DIPR(N)         ((0x000C << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Peer port register address(R/W)
 * @details @ref Sn_DPORT configures or indicates the destination port number of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In TCP clientmode, it configures the listen port number of TCP serverbefore CONNECT command.
 * In TCP Servermode, it indicates the port number of TCP client after successfully establishing connection.
 * In UDP mode, it configures the port number of peer to be transmitted the UDP packet by SEND/SEND_MAC command.
 */
#define Sn_DPORT(N)        ((0x0010 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Maximum Segment Size(Sn_MSSR0) register address(R/W)
 * @details @ref Sn_MSSR configures or indicates the MTU(Maximum Transfer Unit) of Socket n.
 */
#define Sn_MSSR(N)         ((0x0012 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief IP Type of Service(TOS) Register(R/W)
 * @details @ref Sn_TOS configures the TOS(Type Of Service field in IP Header) of Socket n.
 * It is set before OPEN command.
 */
#define Sn_TOS(N)          ((0x0015 << 8) + WIZCHIP_SREG_BLOCK(N))
/**
 * @brief IP Time to live(TTL) Register(R/W)
 * @details @ref Sn_TTL configures the TTL(Time To Live field in IP header) of Socket n.
 * It is set before OPEN command.
 */
#define Sn_TTL(N)          ((0x0016 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Receive memory size register(R/W)
 * @details @ref Sn_RXBUF_SIZE configures the RX buffer block size of Socket n.
 * Socket n RX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data cannot be normally received from a peer.
 * Although Socket n RX Buffer Block size is initially configured to 2Kbytes,
 * user can re-configure its size using @ref Sn_RXBUF_SIZE. The total sum of @ref Sn_RXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data reception error is occurred.
 */
#define Sn_RXBUF_SIZE(N)   ((0x001E << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Transmit memory size register(R/W)
 * @details @ref Sn_TXBUF_SIZE configures the TX buffer block size of Socket n. Socket n TX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data can•t be normally transmitted to a peer.
 * Although Socket n TX Buffer Block size is initially configured to 2Kbytes,
 * user can be re-configure its size using @ref Sn_TXBUF_SIZE. The total sum of @ref Sn_TXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data transmission error is occurred.
 */
#define Sn_TXBUF_SIZE(N)   ((0x001F << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Transmit free memory size register(R)
 * @details @ref Sn_TX_FSR indicates the free size of Socket n TX Buffer Block. It is initialized to the configured size by @ref Sn_TXBUF_SIZE.
 * Data bigger than @ref Sn_TX_FSR should not be saved in the Socket n TX Buffer because the bigger data overwrites the previous saved data not yet sent.
 * Therefore, check before saving the data to the Socket n TX Buffer, and if data is equal or smaller than its checked size,
 * transmit the data with SEND/SEND_MAC command after saving the data in Socket n TX buffer. But, if data is bigger than its checked size,
 * transmit the data after dividing into the checked size and saving in the Socket n TX buffer.
 */
#define Sn_TX_FSR(N)      ((0x0020 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Transmit memory read pointer register address(R)
 * @details @ref Sn_TX_RD is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.
 * After its initialization, it is auto-increased by SEND command.
 * SEND command transmits the saved data from the current @ref Sn_TX_RD to the @ref Sn_TX_WR in the Socket n TX Buffer.
 * After transmitting the saved data, the SEND command increases the @ref Sn_TX_RD as same as the @ref Sn_TX_WR.
 * If its increment value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
#define Sn_TX_RD(N)        ((0x0022 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Transmit memory write pointer register address(R/W)
 * @details @ref Sn_TX_WR is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.\n
 * It should be read or be updated like as follows.\n
 * 1. Read the starting address for saving the transmitting data.\n
 * 2. Save the transmitting data from the starting address of Socket n TX buffer.\n
 * 3. After saving the transmitting data, update @ref Sn_TX_WR to the increased value as many as transmitting data size.
 * If the increment value exceeds the maximum value 0xFFFF(greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.\n
 * 4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND command
 */
#define Sn_TX_WR(N)        ((0x0024 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Received data size register(R)
 * @details @ref Sn_RX_RSR indicates the data size received and saved in Socket n RX Buffer.
 * @ref Sn_RX_RSR does not exceed the @ref Sn_RXBUF_SIZE and is calculated as the difference between
 * •Socket n RX Write Pointer (@ref Sn_RX_WR)and •Socket n RX Read Pointer (@ref Sn_RX_RD)
 */
#define Sn_RX_RSR(N)       ((0x0026 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Read point of Receive memory(R/W)
 * @details @ref Sn_RX_RD is initialized by OPEN command. Make sure to be read or updated as follows.\n
 * 1. Read the starting save address of the received data.\n
 * 2. Read data from the starting address of Socket n RX Buffer.\n
 * 3. After reading the received data, Update @ref Sn_RX_RD to the increased value as many as the reading size.
 * If the increment value exceeds the maximum value 0xFFFF, that is, is greater than 0x10000 and the carry bit occurs,
 * update with the lower 16bits value ignored the carry bit.\n
 * 4. Order RECV command is for notifying the updated @ref Sn_RX_RD to W5500.
 */
#define Sn_RX_RD(N)        ((0x0028 << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Write point of Receive memory(R)
 * @details @ref Sn_RX_WR is initialized by OPEN command and it is auto-increased by the data reception.
 * If the increased value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
#define Sn_RX_WR(N)        ((0x002A << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief socket interrupt mask register(R)
 * @details @ref Sn_IMR masks the interrupt of Socket n.
 * Each bit corresponds to each bit of @ref Sn_IR. When a Socket n Interrupt is occurred and the corresponding bit of @ref Sn_IMR is
 * the corresponding bit of @ref Sn_IR becomes  When both the corresponding bit of @ref Sn_IMR and @ref Sn_IR are and the n-th bit of @ref IR is
 * Host is interrupted by asserted INTn PIN to low.
 */
#define Sn_IMR(N)          ((0x002C << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Fragment field value in IP header register(R/W)
 * @details @ref Sn_FRAG configures the FRAG(Fragment field in IP header).
 */
#define Sn_FRAG(N)         ((0x002D << 8) + WIZCHIP_SREG_BLOCK(N))

/**
 * @brief Keep Alive Timer register(R/W)
 * @details @ref Sn_KPALVTR configures the transmitting timer of •KEEP ALIVE(KA)packet of SOCKETn. It is valid only in TCP mode,
 * and ignored in other modes. The time unit is 5s.
 * KA packet is transmittable after @ref Sn_SR is changed to SOCK_ESTABLISHED and after the data is transmitted or received to/from a peer at least once.
 * In case of '@ref Sn_KPALVTR > 0', W5500 automatically transmits KA packet after time-period for checking the TCP connection (Auto-keepalive-process).
 * In case of '@ref Sn_KPALVTR = 0', Auto-keep-alive-process will not operate,
 * and KA packet can be transmitted by SEND_KEEP command by the host (Manual-keep-alive-process).
 * Manual-keep-alive-process is ignored in case of '@ref Sn_KPALVTR > 0'.
 */
#define Sn_KPALVTR(N)      ((0x002F << 8) + WIZCHIP_SREG_BLOCK(N))



/////////////////////////////////
// Common Register I/O function //
/////////////////////////////////
/**
 * @brief Set Mode Register
 * @param (uint8_t)mr The value to be set.
 * @sa getMR()
 */
#define setMR(mr) \
	wizchip_write(WIZCHIP_CREG_BLOCK, MR, mr)


/**
 * @brief Get Mode Register
 * @return uint8_t. The value of Mode register.
 * @sa setMR()
 */
#define getMR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, MR)

/**
 * @brief Set gateway IP address
 * @param (uint8_t*)gar Pointer variable to set gateway IP address. It should be allocated 4 bytes.
 * @sa getGAR()
 */
#define setGAR(gar) \
		wizchip_write_buf(WIZCHIP_CREG_BLOCK, GAR, gar, 4)

/**
 * @brief Get gateway IP address
 * @param (uint8_t*)gar Pointer variable to get gateway IP address. It should be allocated 4 bytes.
 * @sa setGAR()
 */
#define getGAR(gar) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, GAR, gar, 4)

/**
 * @brief Set subnet mask address
 * @param (uint8_t*)subr Pointer variable to set subnet mask address. It should be allocated 4 bytes.
 * @sa getSUBR()
 */
#define setSUBR(subr) \
		wizchip_write_buf(WIZCHIP_CREG_BLOCK, SUBR, subr, 4)


/**
 * @brief Get subnet mask address
 * @param (uint8_t*)subr Pointer variable to get subnet mask address. It should be allocated 4 bytes.
 * @sa setSUBR()
 */
#define getSUBR(subr) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, SUBR, subr, 4)

/**
 * @brief Set local MAC address
 * @param (uint8_t*)shar Pointer variable to set local MAC address. It should be allocated 6 bytes.
 * @sa getSHAR()
 */
#define setSHAR(shar) \
		wizchip_write_buf(WIZCHIP_CREG_BLOCK, SHAR, shar, 6)

/**
 * @brief Get local MAC address
 * @param (uint8_t*)shar Pointer variable to get local MAC address. It should be allocated 6 bytes.
 * @sa setSHAR()
 */
#define getSHAR(shar) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, SHAR, shar, 6)

/**
 * @brief Set local IP address
 * @param (uint8_t*)sipr Pointer variable to set local IP address. It should be allocated 4 bytes.
 * @sa getSIPR()
 */
#define setSIPR(sipr) \
		wizchip_write_buf(WIZCHIP_CREG_BLOCK, SIPR, sipr, 4)

/**
 * @brief Get local IP address
 * @param (uint8_t*)sipr Pointer variable to get local IP address. It should be allocated 4 bytes.
 * @sa setSIPR()
 */
#define getSIPR(sipr) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, SIPR, sipr, 4)

/**
 * @brief Set INTLEVEL register
 * @param (uint16_t)intlevel Value to set @ref INTLEVEL register.
 * @sa getINTLEVEL()
 */
#define setINTLEVEL(intlevel)  {\
		wizchip_write(WIZCHIP_CREG_BLOCK, INTLEVEL,   (uint8_t)(intlevel >> 8)); \
		wizchip_write(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(INTLEVEL,1), (uint8_t) intlevel); \
	}


/**
 * @brief Get INTLEVEL register
 * @return uint16_t. Value of @ref INTLEVEL register.
 * @sa setINTLEVEL()
 */
#define getINTLEVEL() \
		(((uint16_t)wizchip_read(WIZCHIP_CREG_BLOCK, INTLEVEL) << 8) + wizchip_read(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(INTLEVEL,1)))

/**
 * @brief Set @ref IR register
 * @param (uint8_t)ir Value to set @ref IR register.
 * @sa getIR()
 */
#define setIR(ir) \
		wizchip_write(WIZCHIP_CREG_BLOCK, IR, (ir & 0xF0))

/**
 * @brief Get @ref IR register
 * @return uint8_t. Value of @ref IR register.
 * @sa setIR()
 */
#define getIR() \
		(wizchip_read(WIZCHIP_CREG_BLOCK, IR) & 0xF0)
/**
 * @brief Set @ref _IMR_ register
 * @param (uint8_t)imr Value to set @ref _IMR_ register.
 * @sa getIMR()
 */
#define setIMR(imr) \
		wizchip_write(WIZCHIP_CREG_BLOCK, _IMR_, imr)

/**
 * @brief Get @ref _IMR_ register
 * @return uint8_t. Value of @ref _IMR_ register.
 * @sa setIMR()
 */
#define getIMR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, _IMR_)

/**
 * @brief Set @ref SIR register
 * @param (uint8_t)sir Value to set @ref SIR register.
 * @sa getSIR()
 */
#define setSIR(sir) \
		wizchip_write(WIZCHIP_CREG_BLOCK, SIR, sir)

/**
 * @brief Get @ref SIR register
 * @return uint8_t. Value of @ref SIR register.
 * @sa setSIR()
 */
#define getSIR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, SIR)
/**
 * @brief Set @ref SIMR register
 * @param (uint8_t)simr Value to set @ref SIMR register.
 * @sa getSIMR()
 */
#define setSIMR(simr) \
		wizchip_write(WIZCHIP_CREG_BLOCK, SIMR, simr)

/**
 * @brief Get @ref SIMR register
 * @return uint8_t. Value of @ref SIMR register.
 * @sa setSIMR()
 */
#define getSIMR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, SIMR)

/**
 * @brief Set @ref _RTR_ register
 * @param (uint16_t)rtr Value to set @ref _RTR_ register.
 * @sa getRTR()
 */
#define setRTR(rtr)   {\
		wizchip_write(WIZCHIP_CREG_BLOCK, _RTR_,   (uint8_t)(rtr >> 8)); \
		wizchip_write(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(_RTR_,1), (uint8_t) rtr); \
	}

/**
 * @brief Get @ref _RTR_ register
 * @return uint16_t. Value of @ref _RTR_ register.
 * @sa setRTR()
 */
#define getRTR() \
		(((uint16_t)wizchip_read(WIZCHIP_CREG_BLOCK, _RTR_) << 8) + wizchip_read(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(_RTR_,1)))


/**
 * @brief Set @ref _RCR_ register
 * @param (uint8_t)rcr Value to set @ref _RCR_ register.
 * @sa getRCR()
 */
#define setRCR(rcr) \
		wizchip_write(WIZCHIP_CREG_BLOCK, _RCR_, rcr)

/**
 * @brief Get @ref _RCR_ register
 * @return uint8_t. Value of @ref _RCR_ register.
 * @sa setRCR()
 */
#define getRCR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, _RCR_)

//================================================== test done ===========================================================

/**
 * @brief Set @ref PTIMER register
 * @param (uint8_t)ptimer Value to set @ref PTIMER register.
 * @sa getPTIMER()
 */
#define setPTIMER(ptimer) \
		wizchip_write(WIZCHIP_CREG_BLOCK, PTIMER, ptimer)

/**
 * @brief Get @ref PTIMER register
 * @return uint8_t. Value of @ref PTIMER register.
 * @sa setPTIMER()
 */
#define getPTIMER() \
		wizchip_read(WIZCHIP_CREG_BLOCK, PTIMER)

/**
 * @brief Set @ref PMAGIC register
 * @param (uint8_t)pmagic Value to set @ref PMAGIC register.
 * @sa getPMAGIC()
 */
#define setPMAGIC(pmagic) \
		wizchip_write(WIZCHIP_CREG_BLOCK, PMAGIC, pmagic)

/**
 * @brief Get @ref PMAGIC register
 * @return uint8_t. Value of @ref PMAGIC register.
 * @sa setPMAGIC()
 */
#define getPMAGIC() \
		wizchip_read(WIZCHIP_CREG_BLOCK, PMAGIC)

/**
 * @brief Set @ref PHAR address
 * @param (uint8_t*)phar Pointer variable to set PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa getPHAR()
 */
#define setPHAR(phar) \
		wizchip_write_buf(WIZCHIP_CREG_BLOCK, PHAR, phar, 6)

/**
 * @brief Get @ref PHAR address
 * @param (uint8_t*)phar Pointer variable to PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa setPHAR()
 */
#define getPHAR(phar) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, PHAR, phar, 6)

/**
 * @brief Set @ref PSID register
 * @param (uint16_t)psid Value to set @ref PSID register.
 * @sa getPSID()
 */
#define setPSID(psid)  {\
		wizchip_write(WIZCHIP_CREG_BLOCK, PSID,   (uint8_t)(psid >> 8)); \
		wizchip_write(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(PSID,1), (uint8_t) psid); \
	}

/**
 * @brief Get @ref PSID register
 * @return uint16_t. Value of @ref PSID register.
 * @sa setPSID()
 */
//uint16_t getPSID(void);
#define getPSID() \
		(((uint16_t)wizchip_read(WIZCHIP_CREG_BLOCK, PSID) << 8) + wizchip_read(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(PSID,1)))

/**
 * @brief Set @ref PMRU register
 * @param (uint16_t)pmru Value to set @ref PMRU register.
 * @sa getPMRU()
 */
#define setPMRU(pmru) { \
		wizchip_write(WIZCHIP_CREG_BLOCK, PMRU,   (uint8_t)(pmru>>8)); \
		wizchip_write(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(PMRU,1), (uint8_t) pmru); \
	}

/**
 * @brief Get @ref PMRU register
 * @return uint16_t. Value of @ref PMRU register.
 * @sa setPMRU()
 */
#define getPMRU() \
		(((uint16_t)wizchip_read(WIZCHIP_CREG_BLOCK, PMRU) << 8) + wizchip_read(WIZCHIP_OFFSET_INC(PMRU,1)))

/**
 * @brief Get unreachable IP address
 * @param (uint8_t*)uipr Pointer variable to get unreachable IP address. It should be allocated 4 bytes.
 */
#define getUIPR(uipr) \
		wizchip_read_buf(WIZCHIP_CREG_BLOCK, UIPR,uipr,4)

/**
 * @brief Get @ref UPORTR register
 * @return uint16_t. Value of @ref UPORTR register.
 */
#define getUPORTR() \
	(((uint16_t)wizchip_read(WIZCHIP_CREG_BLOCK, UPORTR) << 8) + wizchip_read(WIZCHIP_CREG_BLOCK, WIZCHIP_OFFSET_INC(UPORTR,1)))	

/**
 * @brief Set @ref PHYCFGR register
 * @param (uint8_t)phycfgr Value to set @ref PHYCFGR register.
 * @sa getPHYCFGR()
 */
#define setPHYCFGR(phycfgr) \
		wizchip_write(WIZCHIP_CREG_BLOCK, PHYCFGR, phycfgr)

/**
 * @brief Get @ref PHYCFGR register
 * @return uint8_t. Value of @ref PHYCFGR register.
 * @sa setPHYCFGR()
 */
#define getPHYCFGR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, PHYCFGR)

/**
 * @brief Get @ref VERSIONR register
 * @return uint8_t. Value of @ref VERSIONR register.
 */
#define getVERSIONR() \
		wizchip_read(WIZCHIP_CREG_BLOCK, VERSIONR)

/////////////////////////////////////

///////////////////////////////////
// Socket N register I/O function //
///////////////////////////////////
/**
 * @brief Set @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)mr Value to set @ref Sn_MR
 * @sa getSn_MR()
 */
#define setSn_MR(sn, mr) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_MR(sn), mr)

/**
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_MR.
 * @sa setSn_MR()
 */
#define getSn_MR(sn) \
	wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_MR(sn))

/**
 * @brief Set @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)cr Value to set @ref Sn_CR
 * @sa getSn_CR()
 */
#define setSn_CR(sn, cr) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_CR(sn), cr)

/**
 * @brief Get @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_CR.
 * @sa setSn_CR()
 */
#define getSn_CR(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_CR(sn))

/**
 * @brief Set @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ir Value to set @ref Sn_IR
 * @sa getSn_IR()
 */
#define setSn_IR(sn, ir) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_IR(sn), (ir & 0x1F))

/**
 * @brief Get @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IR.
 * @sa setSn_IR()
 */
#define getSn_IR(sn) \
		(wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_IR(sn)) & 0x1F)

/**
 * @brief Set @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)imr Value to set @ref Sn_IMR
 * @sa getSn_IMR()
 */
#define setSn_IMR(sn, imr) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_IMR(sn), (imr & 0x1F))

/**
 * @brief Get @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IMR.
 * @sa setSn_IMR()
 */
#define getSn_IMR(sn) \
		(wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_IMR(sn)) & 0x1F)

/**
 * @brief Get @ref Sn_SR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_SR.
 */
#define getSn_SR(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_SR(sn))

/**
 * @brief Set @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)port Value to set @ref Sn_PORT.
 * @sa getSn_PORT()
 */
#define setSn_PORT(sn, port)  { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_PORT(sn),   (uint8_t)(port >> 8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_PORT(sn),1), (uint8_t) port); \
	}

/**
 * @brief Get @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_PORT.
 * @sa setSn_PORT()
 */
#define getSn_PORT(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_PORT(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn),WIZCHIP_OFFSET_INC(Sn_PORT,1)))		

/**
 * @brief Set @ref Sn_DHAR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to set socket n destination hardware address. It should be allocated 6 bytes.
 * @sa getSn_DHAR()
 */
#define setSn_DHAR(sn, dhar) \
		wizchip_write_buf(WIZCHIP_SREG_BLOCK(sn), Sn_DHAR(sn), dhar, 6)

/**
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to get socket n destination hardware address. It should be allocated 6 bytes.
 * @sa setSn_DHAR()
 */
#define getSn_DHAR(sn, dhar) \
		wizchip_read_buf(WIZCHIP_SREG_BLOCK(sn), Sn_DHAR(sn), dhar, 6)

/**
 * @brief Set @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to set socket n destination IP address. It should be allocated 4 bytes.
 * @sa getSn_DIPR()
 */
#define setSn_DIPR(sn, dipr) \
		wizchip_write_buf(WIZCHIP_SREG_BLOCK(sn), Sn_DIPR(sn), dipr, 4)

/**
 * @brief Get @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to get socket n destination IP address. It should be allocated 4 bytes.
 * @sa setSn_DIPR()
 */
#define getSn_DIPR(sn, dipr) \
		wizchip_read_buf(WIZCHIP_SREG_BLOCK(sn), Sn_DIPR(sn), dipr, 4)

/**
 * @brief Set @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)dport Value to set @ref Sn_DPORT
 * @sa getSn_DPORT()
 */
#define setSn_DPORT(sn, dport) { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_DPORT(sn),   (uint8_t) (dport>>8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_DPORT(sn),1), (uint8_t)  dport); \
	}

/**
 * @brief Get @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_DPORT.
 * @sa setSn_DPORT()
 */
#define getSn_DPORT(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_DPORT(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn),WIZCHIP_OFFSET_INC(Sn_DPORT,1)))		

/**
 * @brief Set @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)mss Value to set @ref Sn_MSSR
 * @sa setSn_MSSR()
 */
#define setSn_MSSR(sn, mss) { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_MSSR(sn),   (uint8_t)(mss>>8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_MSSR(sn),1), (uint8_t) mss); \
	}

/**
 * @brief Get @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_MSSR.
 * @sa setSn_MSSR()
 */
#define getSn_MSSR(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_MSSR(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn),WIZCHIP_OFFSET_INC(Sn_MSSR,1)))		

/**
 * @brief Set @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)tos Value to set @ref Sn_TOS
 * @sa getSn_TOS()
 */
#define setSn_TOS(sn, tos) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_TOS(sn), tos)

/**
 * @brief Get @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of Sn_TOS.
 * @sa setSn_TOS()
 */
#define getSn_TOS(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_TOS(sn))

/**
 * @brief Set @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ttl Value to set @ref Sn_TTL
 * @sa getSn_TTL()
 */
#define setSn_TTL(sn, ttl) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_TTL(sn), ttl)


/**
 * @brief Get @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TTL.
 * @sa setSn_TTL()
 */
#define getSn_TTL(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_TTL(sn))


/**
 * @brief Set @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)rxbufsize Value to set @ref Sn_RXBUF_SIZE
 * @sa getSn_RXBUF_SIZE()
 */
#define setSn_RXBUF_SIZE(sn, rxbufsize) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_RXBUF_SIZE(sn),rxbufsize)


/**
 * @brief Get @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_RXBUF_SIZE.
 * @sa setSn_RXBUF_SIZE()
 */
#define getSn_RXBUF_SIZE(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_RXBUF_SIZE(sn))

/**
 * @brief Set @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)txbufsize Value to set @ref Sn_TXBUF_SIZE
 * @sa getSn_TXBUF_SIZE()
 */
#define setSn_TXBUF_SIZE(sn, txbufsize) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_TXBUF_SIZE(sn), txbufsize)

/**
 * @brief Get @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TXBUF_SIZE.
 * @sa setSn_TXBUF_SIZE()
 */
#define getSn_TXBUF_SIZE(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_TXBUF_SIZE(sn))

/**
 * @brief Get @ref Sn_TX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_RD.
 */
#define getSn_TX_RD(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_TX_RD(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_TX_RD,1)))		

/**
 * @brief Set @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)txwr Value to set @ref Sn_TX_WR
 * @sa GetSn_TX_WR()
 */
#define setSn_TX_WR(sn, txwr) { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_TX_WR(sn),   (uint8_t)(txwr>>8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_TX_WR(sn),1), (uint8_t) txwr); \
		}

/**
 * @brief Get @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_WR.
 * @sa setSn_TX_WR()
 */
#define getSn_TX_WR(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_TX_WR(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_TX_WR(sn), 1)))		



/**
 * @brief Set @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)rxrd Value to set @ref Sn_RX_RD
 * @sa getSn_RX_RD()
 */
#define setSn_RX_RD(sn, rxrd) { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_RX_RD(sn),   (uint8_t)(rxrd>>8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_RX_RD(sn),1), (uint8_t) rxrd); \
	}

/**
 * @brief Get @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_RD.
 * @sa setSn_RX_RD()
 */
#define getSn_RX_RD(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_RX_RD(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_RX_RD(sn),1)))		

/**
 * @brief Get @ref Sn_RX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_WR.
 */
#define getSn_RX_WR(sn) \
		(((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_RX_WR(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_RX_WR(sn),1)))		

/**
 * @brief Set @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)frag Value to set @ref Sn_FRAG
 * @sa getSn_FRAD()
 */
#define setSn_FRAG(sn, frag) { \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_FRAG(sn),  (uint8_t)(frag >>8)); \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_FRAG(sn),1), (uint8_t) frag); \
	}

/**
 * @brief Get @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_FRAG.
 * @sa setSn_FRAG()
 */
#define getSn_FRAG(sn) \
      (((uint16_t)wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_FRAG(sn)) << 8) + wizchip_read(WIZCHIP_SREG_BLOCK(sn), WIZCHIP_OFFSET_INC(Sn_FRAG,1)))		

/**
 * @brief Set @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)kpalvt Value to set @ref Sn_KPALVTR
 * @sa getSn_KPALVTR()
 */
#define setSn_KPALVTR(sn, kpalvt) \
		wizchip_write(WIZCHIP_SREG_BLOCK(sn), Sn_KPALVTR(sn), kpalvt)

/**
 * @brief Get @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_KPALVTR.
 * @sa setSn_KPALVTR()
 */
#define getSn_KPALVTR(sn) \
		wizchip_read(WIZCHIP_SREG_BLOCK(sn), Sn_KPALVTR(sn))

//////////////////////////////////////

/////////////////////////////////////
// Sn_TXBUF & Sn_RXBUF IO function //
/////////////////////////////////////
/**
 * @brief Socket_register_access_function
 * @brief Gets the max buffer size of socket sn passed as parameter.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n RX max buffer size.
 */
#define getSn_RxMAX(sn) \
		(((uint16_t)getSn_RXBUF_SIZE(sn)) << 10)		

/**
 * @brief Socket_register_access_function
 * @brief Gets the max buffer size of socket sn passed as parameters.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n TX max buffer size.
 */
#define getSn_TxMAX(sn) \
		(((uint16_t)getSn_TXBUF_SIZE(sn)) << 10)



#define _WIZCHIP_SOCK_NUM_   8   ///< The count of independant socket of @b WIZCHIP


/**
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
} wiz_PhyConf;


class Wiznet5500 {

public:
    /**
     * Constructor that uses the default hardware SPI pins
     * @param cs the Arduino Chip Select / Slave Select pin (default 10)
     */
    Wiznet5500(int8_t cs=SS);


    /**
     * Initialise the Ethernet controller
     * Must be called before sending or receiving Ethernet frames
     *
     * @param address the local MAC address for the Ethernet interface
     * @return Returns true if setting up the Ethernet interface was successful
     */
    boolean begin(const uint8_t *address);

    /**
     * Shut down the Ethernet controlled
     */
    void end();
  
    /**
     * Send an Ethernet frame
     * @param data a pointer to the data to send
     * @param datalen the length of the data in the packet
     * @return the number of bytes transmitted
     */
    uint16_t sendFrame(const uint8_t *data, uint16_t datalen);

    /**
     * Read an Ethernet frame
     * @param buffer a pointer to a buffer to write the packet to
     * @param bufsize the available space in the buffer
     * @return the length of the received packet
     *         or 0 if no packet was received
     */
    uint16_t readFrame(uint8_t *buffer, uint16_t bufsize);


private:

    uint8_t _cs;
    uint8_t _mac_address[6];

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

    ////////////////////////
    // Basic I/O Function //
    ////////////////////////

    /**
     * @brief It reads 1 byte value from a register.
     * @param AddrSel Register address
     * @return The value of register
     */
    uint8_t wizchip_read(uint8_t block, uint32_t AddrSel);

    /**
     * @brief It writes 1 byte value to a register.
     * @param AddrSel Register address
     * @param wb Write data
     * @return void
     */
    void wizchip_write(uint8_t block, uint32_t AddrSel, uint8_t wb);

    /**
     * @brief It reads sequence data from registers.
     * @param AddrSel Register address
     * @param pBuf Pointer buffer to read data
     * @param len Data length
     */
    void wizchip_read_buf(uint8_t block, uint32_t AddrSel, uint8_t* pBuf, uint16_t len);

    /**
     * @brief It writes sequence data to registers.
     * @param AddrSel Register address
     * @param pBuf Pointer buffer to write data
     * @param len Data length
     */
    void wizchip_write_buf(uint8_t block, uint32_t AddrSel, const uint8_t* pBuf, uint16_t len);

    /**
     * @brief Get @ref Sn_TX_FSR register
     * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
     * @return uint16_t. Value of @ref Sn_TX_FSR.
     */
    uint16_t getSn_TX_FSR(uint8_t sn);


    /**
     * @brief Get @ref Sn_RX_RSR register
     * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
     * @return uint16_t. Value of @ref Sn_RX_RSR.
     */
    uint16_t getSn_RX_RSR(uint8_t sn);

    /**
     * @brief Reset WIZCHIP by softly.
     */
    void   wizchip_sw_reset(void);

    /**
     * @brief Initializes WIZCHIP with socket buffer size
     * @param txsize Socket tx buffer sizes. If null, initialized the default size 2KB.
     * @param rxsize Socket rx buffer sizes. If null, initialized the default size 2KB.
     * @return 0 : succcess \n
     *        -1 : fail. Invalid buffer size
     */
    int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize);

    int8_t wizphy_getphylink(void);              ///< get the link status of phy in WIZCHIP. No use in W5100
    int8_t wizphy_getphypmode(void);             ///< get the power mode of PHY in WIZCHIP. No use in W5100



    void   wizphy_reset(void);                   ///< Reset phy. Vailid only in W5500
    /**
     * @brief Set the phy information for WIZCHIP without power mode
     * @param phyconf : @ref wiz_PhyConf
     */
    void   wizphy_setphyconf(wiz_PhyConf* phyconf);

    /**
    * @brief Get phy configuration information.
    * @param phyconf : @ref wiz_PhyConf
    */
    void   wizphy_getphyconf(wiz_PhyConf* phyconf);

    /**
    * @brief Get phy status.
    * @param phyconf : @ref wiz_PhyConf
    */
    void   wizphy_getphystat(wiz_PhyConf* phyconf);

    /**
    * @brief set the power mode of phy inside WIZCHIP. Refer to @ref PHYCFGR in W5500, @ref PHYSTATUS in W5200
    * @param pmode Settig value of power down mode.
    */
    int8_t wizphy_setphypmode(uint8_t pmode);

    /**
     * @brief It copies data to internal TX memory
     *
     * @details This function reads the Tx write pointer register and after that,
     * it copies the <i>wizdata(pointer buffer)</i> of the length of <i>len(variable)</i> bytes to internal TX memory
     * and updates the Tx write pointer register.
     * This function is being called by send() and sendto() function also.
     *
     * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
     * @param wizdata Pointer buffer to write data
     * @param len Data length
     * @sa wiz_recv_data()
     */
    void wizchip_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len);

    /**
     * @brief It copies data to your buffer from internal RX memory
     *
     * @details This function read the Rx read pointer register and after that,
     * it copies the received data from internal RX memory
     * to <i>wizdata(pointer variable)</i> of the length of <i>len(variable)</i> bytes.
     * This function is being called by recv() also.
     *
     * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
     * @param wizdata Pointer buffer to read data
     * @param len Data length
     * @sa wiz_send_data()
     */
    void wizchip_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len);

    /**
     * @brief It discard the received data in RX memory.
     * @details It discards the data of the length of <i>len(variable)</i> bytes in internal RX memory.
     * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
     * @param len Data length
     */
    void wizchip_recv_ignore(uint8_t sn, uint16_t len);


    /** Mode register values */
    enum {
        MR_RST = 0x80,    ///< Reset
        MR_WOL = 0x20,    ///< Wake on LAN
        MR_PB = 0x10,     ///< Ping block
        MR_PPPOE = 0x08,  ///< Enable PPPoE
        MR_FARP = 0x02,   ///< Enable UDP_FORCE_ARP CHECHK
    };

    /* Interrupt Register values */
    enum {
        IR_CONFLICT = 0x80,  ///< Check IP conflict
        IR_UNREACH = 0x40,   ///< Get the destination unreachable message in UDP sending
        IR_PPPoE = 0x20,     ///< Get the PPPoE close message
        IR_MP = 0x10,        ///< Get the magic packet interrupt
    };

    /* Interrupt Mask Register values */
    enum {
        IM_IR7 = 0x80,   ///< IP Conflict Interrupt Mask
        IM_IR6 = 0x40,   ///< Destination unreachable Interrupt Mask
        IM_IR5 = 0x20,   ///< PPPoE Close Interrupt Mask
        IM_IR4 = 0x10,   ///< Magic Packet Interrupt Mask
    };

    /** Socket Mode Register values */
    enum {
        Sn_MR_CLOSE = 0x00,  ///< Unused socket
        Sn_MR_TCP = 0x01,    ///< TCP
        Sn_MR_UDP = 0x02,    ///< UDP
        Sn_MR_MACRAW = 0x04, ///< MAC LAYER RAW SOCK
        Sn_MR_UCASTB = 0x10, ///< Unicast Block in UDP Multicasting
        Sn_MR_ND = 0x20,     ///< No Delayed Ack(TCP), Multicast flag
        Sn_MR_BCASTB = 0x40, ///< Broadcast block in UDP Multicasting
        Sn_MR_MULTI = 0x80,  ///< Support UDP Multicasting
        Sn_MR_MIP6B = 0x10,  ///< IPv6 packet Blocking in @ref Sn_MR_MACRAW mode
        Sn_MR_MMB = 0x20,    ///< Multicast Blocking in @ref Sn_MR_MACRAW mode
        Sn_MR_MFEN = 0x80,   ///< MAC filter enable in @ref Sn_MR_MACRAW mode
    };

    /** Socket Command Register values */
    enum {
        Sn_CR_OPEN = 0x01,      ///< Initialise or open socket
        Sn_CR_LISTEN = 0x02,    ///< Wait connection request in TCP mode (Server mode)
        Sn_CR_CONNECT = 0x04,   ///< Send connection request in TCP mode (Client mode)
        Sn_CR_DISCON = 0x08,    ///< Send closing request in TCP mode
        Sn_CR_CLOSE = 0x10,     ///< Close socket
        Sn_CR_SEND = 0x20,      ///< Update TX buffer pointer and send data
        Sn_CR_SEND_MAC = 0x21,  ///< Send data with MAC address, so without ARP process
        Sn_CR_SEND_KEEP = 0x22, ///< Send keep alive message
        Sn_CR_RECV = 0x40,      ///< Update RX buffer pointer and receive data
    };

    /** Socket Interrupt register values */
    enum {
        Sn_IR_CON = 0x01,      ///< CON Interrupt
        Sn_IR_DISCON = 0x02,   ///< DISCON Interrupt
        Sn_IR_RECV = 0x04,     ///< RECV Interrupt
        Sn_IR_TIMEOUT = 0x08,  ///< TIMEOUT Interrupt
        Sn_IR_SENDOK = 0x10,   ///< SEND_OK Interrupt
    };

    /** Socket Status Register values */
    enum {
        SOCK_CLOSED = 0x00,      ///< Closed
        SOCK_INIT = 0x13,        ///< Initiate state
        SOCK_LISTEN = 0x14,      ///< Listen state
        SOCK_SYNSENT = 0x15,     ///< Connection state
        SOCK_SYNRECV = 0x16,     ///< Connection state
        SOCK_ESTABLISHED = 0x17, ///< Success to connect
        SOCK_FIN_WAIT = 0x18,    ///< Closing state
        SOCK_CLOSING = 0x1A,     ///< Closing state
        SOCK_TIME_WAIT = 0x1B,   ///< Closing state
        SOCK_CLOSE_WAIT = 0x1C,  ///< Closing state
        SOCK_LAST_ACK = 0x1D,    ///< Closing state
        SOCK_UDP = 0x22,         ///< UDP socket
        SOCK_MACRAW = 0x42,      ///< MAC raw mode socket
    };


    /* PHYCFGR register value */
    enum {
        PHYCFGR_RST = ~(1<<7),  //< For PHY reset, must operate AND mask.
        PHYCFGR_OPMD = (1<<6),   // Configre PHY with OPMDC value
        PHYCFGR_OPMDC_ALLA = (7<<3),
        PHYCFGR_OPMDC_PDOWN = (6<<3),
        PHYCFGR_OPMDC_NA = (5<<3),
        PHYCFGR_OPMDC_100FA = (4<<3),
        PHYCFGR_OPMDC_100F = (3<<3),
        PHYCFGR_OPMDC_100H = (2<<3),
        PHYCFGR_OPMDC_10F = (1<<3),
        PHYCFGR_OPMDC_10H = (0<<3),
        PHYCFGR_DPX_FULL = (1<<2),
        PHYCFGR_DPX_HALF = (0<<2),
        PHYCFGR_SPD_100 = (1<<1),
        PHYCFGR_SPD_10 = (0<<1),
        PHYCFGR_LNK_ON = (1<<0),
        PHYCFGR_LNK_OFF = (0<<0),
    };

    enum {
        PHY_CONFBY_HW = 0,     ///< Configured PHY operation mode by HW pin
        PHY_CONFBY_SW = 1,     ///< Configured PHY operation mode by SW register   
        PHY_MODE_MANUAL = 0,     ///< Configured PHY operation mode with user setting.
        PHY_MODE_AUTONEGO = 1,     ///< Configured PHY operation mode with auto-negotiation
        PHY_SPEED_10 = 0,     ///< Link Speed 10
        PHY_SPEED_100 = 1,     ///< Link Speed 100
        PHY_DUPLEX_HALF = 0,     ///< Link Half-Duplex
        PHY_DUPLEX_FULL = 1,     ///< Link Full-Duplex
        PHY_LINK_OFF = 0,     ///< Link Off
        PHY_LINK_ON = 1,     ///< Link On
        PHY_POWER_NORM = 0,     ///< PHY power normal mode
        PHY_POWER_DOWN = 1,     ///< PHY power down mode 
    };

};

#endif   // _W5500_H_
