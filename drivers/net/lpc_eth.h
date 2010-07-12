/*
 * drivers/net/lpc-eth.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LPC_ETH_H
#define __LPC_ETH_H

#include <linux/types.h>

/*
 * Ethernet MAC controller Register offsets
 */
#define LPC_ENET_MAC1(x)			(x + 0x000)
#define LPC_ENET_MAC2(x)			(x + 0x004)
#define LPC_ENET_IPGT(x)			(x + 0x008)
#define LPC_ENET_IPGR(x)			(x + 0x00C)
#define LPC_ENET_CLRT(x)			(x + 0x010)
#define LPC_ENET_MAXF(x)			(x + 0x014)
#define LPC_ENET_SUPP(x)			(x + 0x018)
#define LPC_ENET_TEST(x)			(x + 0x01C)
#define LPC_ENET_MCFG(x)			(x + 0x020)
#define LPC_ENET_MCMD(x)			(x + 0x024)
#define LPC_ENET_MADR(x)			(x + 0x028)
#define LPC_ENET_MWTD(x)			(x + 0x02C)
#define LPC_ENET_MRDD(x)			(x + 0x030)
#define LPC_ENET_MIND(x)			(x + 0x034)
#define LPC_ENET_SA0(x)				(x + 0x040)
#define LPC_ENET_SA1(x)				(x + 0x044)
#define LPC_ENET_SA2(x)				(x + 0x048)
#define LPC_ENET_COMMAND(x)			(x + 0x100)
#define LPC_ENET_STATUS(x)			(x + 0x104)
#define LPC_ENET_RXDESCRIPTOR(x)		(x + 0x108)
#define LPC_ENET_RXSTATUS(x)			(x + 0x10C)
#define LPC_ENET_RXDESCRIPTORNUMBER(x)		(x + 0x110)
#define LPC_ENET_RXPRODUCEINDEX(x)		(x + 0x114)
#define LPC_ENET_RXCONSUMEINDEX(x)		(x + 0x118)
#define LPC_ENET_TXDESCRIPTOR(x)		(x + 0x11C)
#define LPC_ENET_TXSTATUS(x)			(x + 0x120)
#define LPC_ENET_TXDESCRIPTORNUMBER(x)		(x + 0x124)
#define LPC_ENET_TXPRODUCEINDEX(x)		(x + 0x128)
#define LPC_ENET_TXCONSUMEINDEX(x)		(x + 0x12C)
#define LPC_ENET_TSV0(x)			(x + 0x158)
#define LPC_ENET_TSV1(x)			(x + 0x15C)
#define LPC_ENET_RSV(x)				(x + 0x160)
#define LPC_ENET_FLOWCONTROLCOUNTER(x)		(x + 0x170)
#define LPC_ENET_FLOWCONTROLSTATUS(x)		(x + 0x174)
#define LPC_ENET_RXFILTER_CTRL(x)		(x + 0x200)
#define LPC_ENET_RXFILTERWOLSTATUS(x)		(x + 0x204)
#define LPC_ENET_RXFILTERWOLCLEAR(x)		(x + 0x208)
#define LPC_ENET_HASHFILTERL(x)			(x + 0x210)
#define LPC_ENET_HASHFILTERH(x)			(x + 0x214)
#define LPC_ENET_INTSTATUS(x)			(x + 0xFE0)
#define LPC_ENET_INTENABLE(x)			(x + 0xFE4)
#define LPC_ENET_INTCLEAR(x)			(x + 0xFE8)
#define LPC_ENET_INTSET(x)			(x + 0xFEC)
#define LPC_ENET_POWERDOWN(x)			(x + 0xFF4)

/*
 * Structure of a TX/RX descriptors and RX status
 */
struct txrx_desc_t {
	volatile u32 packet;
	volatile u32 control;
};
struct rx_status_t {
	volatile u32 statusinfo;
	volatile u32 statushashcrc;
};

/*
 * mac1 register definitions
 */
#define LPC_MAC1_RECV_ENABLE			(1 << 0)
#define LPC_MAC1_PASS_ALL_RX_FRAMES		(1 << 1)
#define LPC_MAC1_RX_FLOW_CONTROL		(1 << 2)
#define LPC_MAC1_TX_FLOW_CONTROL		(1 << 3)
#define LPC_MAC1_LOOPBACK			(1 << 4)
#define LPC_MAC1_RESET_TX			(1 << 8)
#define LPC_MAC1_RESET_MCS_TX			(1 << 9)
#define LPC_MAC1_RESET_RX			(1 << 10)
#define LPC_MAC1_RESET_MCS_RX			(1 << 11)
#define LPC_MAC1_SIMULATION_RESET		(1 << 14)
#define LPC_MAC1_SOFT_RESET			(1 << 15)

/*
 * mac2 register definitions
 */
#define LPC_MAC2_FULL_DUPLEX			(1 << 0)
#define LPC_MAC2_FRAME_LENGTH_CHECKING		(1 << 1)
#define LPC_MAC2_HUGH_LENGTH_CHECKING		(1 << 2)
#define LPC_MAC2_DELAYED_CRC			(1 << 3)
#define LPC_MAC2_CRC_ENABLE			(1 << 4)
#define LPC_MAC2_PAD_CRC_ENABLE			(1 << 5)
#define LPC_MAC2_VLAN_PAD_ENABLE		(1 << 6)
#define LPC_MAC2_AUTO_DETECT_PAD_ENABLE		(1 << 7)
#define LPC_MAC2_PURE_PREAMBLE_ENFORCEMENT	(1 << 8)
#define LPC_MAC2_LONG_PREAMBLE_ENFORCEMENT	(1 << 9)
#define LPC_MAC2_NO_BACKOFF			(1 << 12)
#define LPC_MAC2_BACK_PRESSURE			(1 << 13)
#define LPC_MAC2_EXCESS_DEFER			(1 << 14)

/*
 * ipgt register definitions
 */
#define LPC_IPGT_LOAD(n)			((n) & 0x7F)

/*
 * ipgr register definitions
 */
#define LPC_IPGR_LOAD_PART2(n)			((n) & 0x7F)
#define LPC_IPGR_LOAD_PART1(n)			(((n) & 0x7F) << 8)

/*
 * clrt register definitions
 */
#define LPC_CLRT_LOAD_RETRY_MAX(n)		((n) & 0xF)
#define LPC_CLRT_LOAD_COLLISION_WINDOW(n)	(((n) & 0x3F) << 8)

/*
 * maxf register definitions
 */
#define LPC_MAXF_LOAD_MAX_FRAME_LEN(n)		((n) & 0xFFFF)

/*
 * supp register definitions
 */
#define LPC_SUPP_SPEED				(1 << 8)
#define LPC_SUPP_RESET_RMII			(1 << 11)

/*
 * test register definitions
 */
#define LPC_TEST_SHORTCUT_PAUSE_QUANTA		(1 << 0)
#define LPC_TEST_PAUSE				(1 << 1)
#define LPC_TEST_BACKPRESSURE			(1 << 2)

/*
 * mcfg register definitions
 */
#define LPC_MCFG_SCAN_INCREMENT			(1 << 0)
#define LPC_MCFG_SUPPRESS_PREAMBLE		(1 << 1)
#define LPC_MCFG_CLOCK_SELECT(n)		(((n) & 0x7) << 2)
#define LPC_MCFG_CLOCK_HOST_DIV_4		0
#define LPC_MCFG_CLOCK_HOST_DIV_6		2
#define LPC_MCFG_CLOCK_HOST_DIV_8		3
#define LPC_MCFG_CLOCK_HOST_DIV_10		4
#define LPC_MCFG_CLOCK_HOST_DIV_14		5
#define LPC_MCFG_CLOCK_HOST_DIV_20		6
#define LPC_MCFG_CLOCK_HOST_DIV_28		7
#define LPC_MCFG_RESET_MII_MGMT			(1 << 15)

/*
 * mcmd register definitions
 */
#define LPC_MCMD_READ				(1 << 0)
#define LPC_MCMD_SCAN				(1 << 1)

/*
 * madr register definitions
 */
#define LPC_MADR_REGISTER_ADDRESS(n)		((n) & 0x1F)
#define LPC_MADR_PHY_0ADDRESS(n)		(((n) & 0x1F) << 8)

/*
 * mwtd register definitions
 */
#define LPC_MWDT_WRITE(n)			((n) & 0xFFFF)

/*
 * mrdd register definitions
 */
#define LPC_MRDD_READ_MASK			0xFFFF

/*
 * mind register definitions
 */
#define LPC_MIND_BUSY				(1 << 0)
#define LPC_MIND_SCANNING			(1 << 1)
#define LPC_MIND_NOT_VALID			(1 << 2)
#define LPC_MIND_MII_LINK_FAIL			(1 << 3)

/*
 * command register definitions
 */
#define LPC_COMMAND_RXENABLE			(1 << 0)
#define LPC_COMMAND_TXENABLE			(1 << 1)
#define LPC_COMMAND_REG_RESET			(1 << 3)
#define LPC_COMMAND_TXRESET			(1 << 4)
#define LPC_COMMAND_RXRESET			(1 << 5)
#define LPC_COMMAND_PASSRUNTFRAME		(1 << 6)
#define LPC_COMMAND_PASSRXFILTER		(1 << 7)
#define LPC_COMMAND_TXFLOWCONTROL		(1 << 8)
#define LPC_COMMAND_RMII			(1 << 9)
#define LPC_COMMAND_FULLDUPLEX			(1 << 10)

/*
 * status register definitions
 */
#define LPC_STATUS_RXACTIVE			(1 << 0)
#define LPC_STATUS_TXACTIVE			(1 << 1)

/*
 * tsv0 register definitions
 */
#define LPC_TSV0_CRC_ERROR			(1 << 0)
#define LPC_TSV0_LENGTH_CHECK_ERROR		(1 << 1)
#define LPC_TSV0_LENGTH_OUT_OF_RANGE		(1 << 2)
#define LPC_TSV0_DONE				(1 << 3)
#define LPC_TSV0_MULTICAST			(1 << 4)
#define LPC_TSV0_BROADCAST			(1 << 5)
#define LPC_TSV0_PACKET_DEFER			(1 << 6)
#define LPC_TSV0_ESCESSIVE_DEFER		(1 << 7)
#define LPC_TSV0_ESCESSIVE_COLLISION		(1 << 8)
#define LPC_TSV0_LATE_COLLISION			(1 << 9)
#define LPC_TSV0_GIANT				(1 << 10)
#define LPC_TSV0_UNDERRUN			(1 << 11)
#define LPC_TSV0_TOTAL_BYTES(n)			(((n) >> 12) & 0xFFFF)
#define LPC_TSV0_CONTROL_FRAME			(1 << 28)
#define LPC_TSV0_PAUSE				(1 << 29)
#define LPC_TSV0_BACKPRESSURE			(1 << 30)
#define LPC_TSV0_VLAN				(1 << 31)

/*
 * tsv1 register definitions
 */
#define LPC_TSV1_TRANSMIT_BYTE_COUNT(n)		((n) & 0xFFFF)
#define LPC_TSV1_COLLISION_COUNT(n)		(((n) >> 16) & 0xF)

/*
 * rsv register definitions
 */
#define LPC_RSV_RECEIVED_BYTE_COUNT(n)		((n) & 0xFFFF)
#define LPC_RSV_RXDV_EVENT_IGNORED		(1 << 16)
#define LPC_RSV_RXDV_EVENT_PREVIOUSLY_SEEN	(1 << 17)
#define LPC_RSV_CARRIER_EVNT_PREVIOUS_SEEN	(1 << 18)
#define LPC_RSV_RECEIVE_CODE_VIOLATION		(1 << 19)
#define LPC_RSV_CRC_ERROR			(1 << 20)
#define LPC_RSV_LENGTH_CHECK_ERROR		(1 << 21)
#define LPC_RSV_LENGTH_OUT_OF_RANGE		(1 << 22)
#define LPC_RSV_RECEIVE_OK			(1 << 23)
#define LPC_RSV_MULTICAST			(1 << 24)
#define LPC_RSV_BROADCAST			(1 << 25)
#define LPC_RSV_DRIBBLE_NIBBLE			(1 << 26)
#define LPC_RSV_CONTROL_FRAME			(1 << 27)
#define LPC_RSV_PAUSE				(1 << 28)
#define LPC_RSV_UNSUPPORTED_OPCODE		(1 << 29)
#define LPC_RSV_VLAN				(1 << 30)

/*
 * flowcontrolcounter register definitions
 */
#define LPC_FCCR_MIRRORCOUNTER(n)		((n) & 0xFFFF)
#define LPC_FCCR_PAUSETIMER(n)			(((n) >> 16) & 0xFFFF)

/*
 * flowcontrolstatus register definitions
 */
#define LPC_FCCR_MIRRORCOUNTERCURRENT(n)	((n) & 0xFFFF)

/*
 * rxfliterctrl, rxfilterwolstatus, and rxfilterwolclear shared
 * register definitions
 */
#define LPC_RXFLTRW_ACCEPTUNICAST		(1 << 0)
#define LPC_RXFLTRW_ACCEPTUBROADCAST		(1 << 1)
#define LPC_RXFLTRW_ACCEPTUMULTICAST		(1 << 2)
#define LPC_RXFLTRW_ACCEPTUNICASTHASH		(1 << 3)
#define LPC_RXFLTRW_ACCEPTUMULTICASTHASH	(1 << 4)
#define LPC_RXFLTRW_ACCEPTPERFECT		(1 << 5)

/*
 * rxfliterctrl register definitions
 */
#define LPC_RXFLTRWSTS_MAGICPACKETENWOL		(1 << 12)
#define LPC_RXFLTRWSTS_RXFILTERENWOL		(1 << 13)

/*
 * rxfilterwolstatus/rxfilterwolclear register definitions
 */
#define LPC_RXFLTRWSTS_RXFILTERWOL		(1 << 7)
#define LPC_RXFLTRWSTS_MAGICPACKETWOL		(1 << 8)

/*
 * intstatus, intenable, intclear, and Intset shared register
 * definitions
 */
#define LPC_MACINT_RXOVERRUNINTEN		(1 << 0)
#define LPC_MACINT_RXERRORONINT			(1 << 1)
#define LPC_MACINT_RXFINISHEDINTEN		(1 << 2)
#define LPC_MACINT_RXDONEINTEN			(1 << 3)
#define LPC_MACINT_TXUNDERRUNINTEN		(1 << 4)
#define LPC_MACINT_TXERRORINTEN			(1 << 5)
#define LPC_MACINT_TXFINISHEDINTEN		(1 << 6)
#define LPC_MACINT_TXDONEINTEN			(1 << 7)
#define LPC_MACINT_SOFTINTEN			(1 << 12)
#define LPC_MACINT_WAKEUPINTEN			(1 << 13)

/*
 * powerdown register definitions
 */
#define LPC_POWERDOWN_MACAHB			(1 << 31)

#endif
