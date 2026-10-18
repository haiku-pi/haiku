/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _GENET_H
#define _GENET_H


/*!	Registers of the Broadcom GENET v5 Ethernet MAC ("brcm,bcm2711-genet-v5",
	the Raspberry Pi 4's on-board Ethernet) and of the BCM54213PE PHY behind it.

	All offsets are in bytes from the start of the 64 KiB register window.
	Only the GENET v5 layout is described; older GENET revisions move the DMA
	blocks and use 2-word descriptors. Names follow the Linux driver
	(drivers/net/ethernet/broadcom/genet/bcmgenet.h, ../unimac.h) so that the
	two can be compared side by side.
*/


// #pragma mark - Register blocks


#define GENET_SYS					0x0000
#define GENET_EXT					0x0080
#define GENET_INTRL2_0				0x0200
#define GENET_INTRL2_1				0x0240
#define GENET_RBUF					0x0300
#define GENET_TBUF					0x0600
#define GENET_UMAC					0x0800
#define GENET_RDMA					0x2000
#define GENET_TDMA					0x4000
#define GENET_HFB_REGS				0xfc00

#define GENET_REGISTERS_SIZE		0x10000


// #pragma mark - SYS


#define SYS_REV_CTRL				(GENET_SYS + 0x00)
#define	SYS_REV_MAJOR(x)			(((x) >> 24) & 0xf)
#define	SYS_REV_MINOR(x)			(((x) >> 16) & 0xf)
#define SYS_PORT_CTRL				(GENET_SYS + 0x04)
#define	PORT_MODE_EXT_GPHY			3
#define SYS_RBUF_FLUSH_CTRL			(GENET_SYS + 0x08)
#define	RBUF_FLUSH_DMA				(1 << 0)
#define	RBUF_FLUSH_UMAC				(1 << 1)
#define SYS_TBUF_FLUSH_CTRL			(GENET_SYS + 0x0c)


// #pragma mark - EXT


#define EXT_RGMII_OOB_CTRL			(GENET_EXT + 0x0c)
#define	RGMII_LINK					(1 << 4)
#define	OOB_DISABLE					(1 << 5)
#define	RGMII_MODE_EN				(1 << 6)
#define	ID_MODE_DIS					(1 << 16)


// #pragma mark - INTRL2 (two identical interrupt controllers)


#define INTRL2_CPU_STAT				0x00
#define INTRL2_CPU_SET				0x04
#define INTRL2_CPU_CLEAR			0x08
#define INTRL2_CPU_MASK_STATUS		0x0c
#define INTRL2_CPU_MASK_SET			0x10
#define INTRL2_CPU_MASK_CLEAR		0x14

// INTRL2_0 sources. The descriptor ring (ring 16) reports its completions
// here; the priority rings 0-15 report theirs on INTRL2_1, which this driver
// does not use.
#define UMAC_IRQ_RXDMA_MBDONE		(1 << 13)
#define UMAC_IRQ_TXDMA_MBDONE		(1 << 16)


// #pragma mark - RBUF / TBUF


#define RBUF_CTRL					(GENET_RBUF + 0x00)
#define	RBUF_64B_EN					(1 << 0)
#define	RBUF_ALIGN_2B				(1 << 1)
#define RBUF_CHK_CTRL				(GENET_RBUF + 0x14)
#define RBUF_ENERGY_CTRL			(GENET_RBUF + 0x9c)
#define	RBUF_EEE_EN					(1 << 0)
#define	RBUF_PM_EN					(1 << 1)
#define RBUF_TBUF_SIZE_CTRL			(GENET_RBUF + 0xb4)

#define TBUF_CTRL					(GENET_TBUF + 0x00)
#define	TBUF_64B_EN					(1 << 0)

// With RBUF_ALIGN_2B (and without RBUF_64B_EN) the controller writes two
// padding bytes in front of every received frame, so that the IP header
// lands 4-byte aligned. The descriptor's length includes them.
#define RX_ALIGN_PAD				2


// #pragma mark - UMAC (UniMAC)


#define UMAC_CMD					(GENET_UMAC + 0x008)
#define	CMD_TX_EN					(1 << 0)
#define	CMD_RX_EN					(1 << 1)
#define	CMD_SPEED_SHIFT				2
#define	CMD_SPEED_MASK				(3 << CMD_SPEED_SHIFT)
#define	CMD_SPEED_10				(0 << CMD_SPEED_SHIFT)
#define	CMD_SPEED_100				(1 << CMD_SPEED_SHIFT)
#define	CMD_SPEED_1000				(2 << CMD_SPEED_SHIFT)
#define	CMD_PROMISC					(1 << 4)
#define	CMD_CRC_FWD					(1 << 6)
#define	CMD_RX_PAUSE_IGNORE			(1 << 8)
#define	CMD_HD_EN					(1 << 10)
#define	CMD_SW_RESET				(1 << 13)
#define	CMD_TX_PAUSE_IGNORE			(1 << 28)
#define UMAC_MAC0					(GENET_UMAC + 0x00c)
#define UMAC_MAC1					(GENET_UMAC + 0x010)
#define UMAC_MAX_FRAME_LEN			(GENET_UMAC + 0x014)
#define UMAC_EEE_CTRL				(GENET_UMAC + 0x064)
#define	EEE_EN						(1 << 3)
#define UMAC_TX_FLUSH				(GENET_UMAC + 0x334)
#define UMAC_MIB_CTRL				(GENET_UMAC + 0x580)
#define	MIB_RESET_RX				(1 << 0)
#define	MIB_RESET_RUNT				(1 << 1)
#define	MIB_RESET_TX				(1 << 2)
#define UMAC_MDIO_CMD				(GENET_UMAC + 0x614)
#define	MDIO_START_BUSY				(1 << 29)
#define	MDIO_READ_FAIL				(1 << 28)
#define	MDIO_RD						(2 << 26)
#define	MDIO_WR						(1 << 26)
#define	MDIO_PMD_SHIFT				21
#define	MDIO_REG_SHIFT				16
#define	MDIO_DATA_MASK				0xffff
#define UMAC_MDF_CTRL				(GENET_UMAC + 0x650)
#define UMAC_MDF_ADDR				(GENET_UMAC + 0x654)
	// 17 address filters of two registers each

#define MDF_FILTER_COUNT			17

// The largest frame the MAC accepts: 1500 bytes of payload, Ethernet header,
// VLAN tag, Broadcom tag, FCS and padding (Linux ENET_MAX_MTU_SIZE).
#define GENET_MAX_MTU_SIZE			1536


// #pragma mark - HFB (hardware filter block)


#define HFB_CTRL					(GENET_HFB_REGS + 0x00)
#define HFB_FLT_ENABLE				(GENET_HFB_REGS + 0x04)
	// two registers


// #pragma mark - DMA


/*	Both DMA engines have the same layout: 256 buffer descriptors of three
	words each, then 17 blocks of per-ring registers (rings 0-15 are the
	priority rings, ring 16 is the descriptor ring every frame goes to unless
	the hardware filters steer it elsewhere), then the global registers.
*/
#define DMA_DESC_COUNT				256
#define DMA_DESC_SIZE				12
#define DMA_DESC_WORDS				(DMA_DESC_SIZE / 4)
#define DMA_DEFAULT_RING			16
#define DMA_RING_REGS_SIZE			0x40

#define DMA_DESCS_SIZE				(DMA_DESC_COUNT * DMA_DESC_SIZE)
#define DMA_RINGS_SIZE				(DMA_RING_REGS_SIZE * (DMA_DEFAULT_RING + 1))

#define RDMA_DESC(index)			(GENET_RDMA + (index) * DMA_DESC_SIZE)
#define TDMA_DESC(index)			(GENET_TDMA + (index) * DMA_DESC_SIZE)
#define RDMA_RING_REG(reg)			(GENET_RDMA + DMA_DESCS_SIZE \
										+ DMA_DEFAULT_RING * DMA_RING_REGS_SIZE + (reg))
#define TDMA_RING_REG(reg)			(GENET_TDMA + DMA_DESCS_SIZE \
										+ DMA_DEFAULT_RING * DMA_RING_REGS_SIZE + (reg))
#define RDMA_REG(reg)				(GENET_RDMA + DMA_DESCS_SIZE + DMA_RINGS_SIZE \
										+ (reg))
#define TDMA_REG(reg)				(GENET_TDMA + DMA_DESCS_SIZE + DMA_RINGS_SIZE \
										+ (reg))

// Descriptor words
#define DMA_DESC_LENGTH_STATUS		0x00
#define DMA_DESC_ADDRESS_LO			0x04
#define DMA_DESC_ADDRESS_HI			0x08

#define	DMA_BUFLENGTH_SHIFT			16
#define	DMA_BUFLENGTH_MASK			0x0fff
#define	DMA_OWN						0x8000
#define	DMA_EOP						0x4000
#define	DMA_SOP						0x2000
#define	DMA_TX_APPEND_CRC			0x0040
#define	DMA_TX_QTAG_SHIFT			7
#define	DMA_TX_QTAG_MASK			0x3f
#define	DMA_RX_LG					0x0010
#define	DMA_RX_NO					0x0008
#define	DMA_RX_RXER					0x0004
#define	DMA_RX_CRC_ERROR			0x0002
#define	DMA_RX_OV					0x0001
#define	DMA_RX_ERROR_MASK			(DMA_RX_LG | DMA_RX_NO | DMA_RX_RXER \
										| DMA_RX_CRC_ERROR | DMA_RX_OV)

/*	Per-ring registers. The two engines name the index pair from opposite
	sides: on the transmit side software produces and hardware consumes, on
	the receive side it is the other way around, so the producer and consumer
	index registers trade places.
*/
#define TDMA_READ_PTR				0x00
#define TDMA_CONS_INDEX				0x08	// written by the hardware
#define TDMA_PROD_INDEX				0x0c	// written by the driver
#define TDMA_FLOW_PERIOD			0x28
#define TDMA_WRITE_PTR				0x2c

#define RDMA_WRITE_PTR				0x00
#define RDMA_PROD_INDEX				0x08	// written by the hardware
#define RDMA_CONS_INDEX				0x0c	// written by the driver
#define RDMA_XON_XOFF_THRESH		0x28
#define RDMA_READ_PTR				0x2c

#define DMA_RING_BUF_SIZE			0x10
#define	DMA_RING_SIZE_SHIFT			16
#define DMA_START_ADDR				0x14
#define DMA_END_ADDR				0x1c
#define DMA_MBUF_DONE_THRESH		0x24

#define	DMA_INDEX_MASK				0xffff
#define	DMA_XOFF_THRESHOLD_SHIFT	16
#define	DMA_FC_THRESH_LO			5
#define	DMA_FC_THRESH_HI			(DMA_DESC_COUNT >> 4)

// Global registers
#define DMA_RING_CFG				0x00
#define DMA_CTRL					0x04
#define	DMA_EN						(1 << 0)
#define	DMA_RING_BUF_EN_SHIFT		1
#define DMA_STATUS					0x08
#define DMA_SCB_BURST_SIZE			0x0c
#define DMA_RING16_TIMEOUT			0x6c
#define DMA_INDEX2RING(n)			(0x70 + (n) * 4)	// n = 0..7

// The BCM2711 needs a shorter burst than the GENET default of 16.
#define DMA_BURST_LENGTH_BCM2711	8

// The ring timeout counts 8.192 us ticks: re-raise the receive interrupt
// when a frame has waited ~57 us (Linux' default rx-usecs of 50, rounded up).
#define DMA_RX_TIMEOUT_TICKS		7


// #pragma mark - PHY (IEEE 802.3 clause 22 + Broadcom extensions)


#define MII_BMCR					0x00
#define	BMCR_ANENABLE				(1 << 12)
#define	BMCR_PDOWN					(1 << 11)
#define	BMCR_ISOLATE				(1 << 10)
#define	BMCR_ANRESTART				(1 << 9)
#define MII_BMSR					0x01
#define	BMSR_ANEGCOMPLETE			(1 << 5)
#define	BMSR_LSTATUS				(1 << 2)
#define MII_PHYSID1					0x02
#define MII_PHYSID2					0x03
#define MII_ADVERTISE				0x04
#define	ADVERTISE_CSMA				0x0001
#define	ADVERTISE_10HALF			0x0020
#define	ADVERTISE_10FULL			0x0040
#define	ADVERTISE_100HALF			0x0080
#define	ADVERTISE_100FULL			0x0100
#define MII_LPA						0x05
	// same bit layout as MII_ADVERTISE
#define MII_CTRL1000				0x09
#define	ADVERTISE_1000FULL			0x0200
#define MII_STAT1000				0x0a
#define	LPA_1000FULL				0x0800
#define MII_MMD_CTRL				0x0d
#define	MMD_CTRL_DATA_NO_INC		0x4000
#define MII_MMD_DATA				0x0e

#define MDIO_MMD_AN					7
#define MDIO_AN_EEE_ADV				0x3c

#define BCM54213PE_PHY_ID			0x600d84a0
#define BCM54213PE_PHY_ID_MASK		0xfffffff0
	// the low four bits are the revision

#define BCM54XX_EXP_DATA			0x15
#define BCM54XX_EXP_SEL				0x17
#define BCM54XX_TOP_MISC_MII_BUF_CNTL0	0x0d00
#define	BCM54XX_AUTOGREEEN_EN		(1 << 0)

#define BCM54XX_AUX_CTL				0x18
#define	AUXCTL_SHDWSEL_MASK			0x0007
#define	AUXCTL_SHDWSEL_READ_SHIFT	12
#define	AUXCTL_SHDWSEL_MISC			0x0007
#define	AUXCTL_MISC_RGMII_SKEW_EN	0x0100
#define	AUXCTL_MISC_WREN			0x8000

#define BCM54XX_AUX_STATUS			0x19
#define	AUX_STATUS_HCD(x)			(((x) >> 8) & 0x7)

#define BCM54XX_SHD					0x1c
#define	SHD_WRITE					0x8000
#define	SHD_VAL(x)					(((x) & 0x1f) << 10)
#define	SHD_DATA(x)					((x) & 0x3ff)
#define BCM54XX_SHD_CLK_CTL			0x03
#define	SHD_CLK_CTL_GTXCLK_EN		(1 << 9)
#define BCM54XX_SHD_APD				0x0a
#define	SHD_APD_EN					0x0020

#define BCM_CL45VEN_EEE_CONTROL		0x803d
#define	LPI_FEATURE_EN				0x8000
#define	LPI_FEATURE_EN_DIG1000X		0x4000


#endif	// _GENET_H
