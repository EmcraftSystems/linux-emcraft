/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * This file is based on mcfqspi.h
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef SPI_MVF_H_
#define SPI_MVF_H_

struct spi_mvf_chip {
	u8 mode;
	u8 bits_per_word;
	u16 void_write_data;
	/* Only used in master mode */
	u8 dbr;		/* Double baud rate */
	u8 pbr;		/* Baud rate prescaler */
	u8 br;		/* Baud rate scaler */
	u8 pcssck;	/* PCS to SCK delay prescaler */
	u8 pasc;	/* After SCK delay prescaler */
	u8 pdt;		/* Delay after transfer prescaler */
	u8 cssck;	/* PCS to SCK delay scaler */
	u8 asc;		/* After SCK delay scaler */
	u8 dt;		/* Delay after transfer scaler */
};

struct spi_mvf_master {
	u16 bus_num;
	int *chipselect;
	int num_chipselect;
	void (*cs_control)(u8 cs, u8 command);
};


#define	SPI_MCR			0x00

#define SPI_TCR			0x08

#define SPI_CTAR(x)		(0x0c + (x * 4))
#define SPI_CTAR_FMSZ(x)	(((x) & 0x0000000f) << 27)
#define SPI_CTAR_CPOL(x)	((x) << 26)
#define SPI_CTAR_CPHA(x)	((x) << 25)
#define SPI_CTAR_PCSSCR(x)	(((x) & 0x00000003) << 22)
#define SPI_CTAR_PASC(x)	(((x) & 0x00000003) << 20)
#define SPI_CTAR_PDT(x)		(((x) & 0x00000003) << 18)
#define SPI_CTAR_PBR(x)		(((x) & 0x00000003) << 16)
#define SPI_CTAR_CSSCK(x)	(((x) & 0x0000000f) << 12)
#define SPI_CTAR_ASC(x)		(((x) & 0x0000000f) << 8)
#define SPI_CTAR_DT(x)		(((x) & 0x0000000f) << 4)
#define SPI_CTAR_BR(x)		((x) & 0x0000000f)

#define SPI_CTAR0_SLAVE		0x0c

#define SPI_SR			0x2c
#define SPI_SR_EOQF		0x10000000

#define SPI_RSER		0x30
#define SPI_RSER_EOQFE		(1 << 28)
#define SPI_RSER_TCFFE		(1 << 31)

#define SPI_PUSHR		0x34
#define SPI_PUSHR_CONT		(1 << 31)
#define SPI_PUSHR_CTAS(x)	(((x) & 0x00000007) << 28)
#define SPI_PUSHR_EOQ		(1 << 27)
#define SPI_PUSHR_CTCNT		(1 << 26)
#define SPI_PUSHR_PCS(x)	(((1 << x) & 0x0000003f) << 16)
#define SPI_PUSHR_TXDATA(x)	((x) & 0x0000ffff)

#define SPI_PUSHR_SLAVE		0x34

#define SPI_POPR		0x38
#define SPI_POPR_RXDATA(x)	((x) & 0x0000ffff)

#define SPI_TXFR0		0x3c
#define SPI_TXFR1		0x40
#define SPI_TXFR2		0x44
#define SPI_TXFR3		0x48
#define SPI_RXFR0		0x7c
#define SPI_RXFR1		0x80
#define SPI_RXFR2		0x84
#define SPI_RXFR3		0x88


#define SPI_FRAME_BITS		SPI_CTAR_FMSZ(0xf)
#define SPI_FRAME_BITS_16	SPI_CTAR_FMSZ(0xf)
#define SPI_FRAME_BITS_8	SPI_CTAR_FMSZ(0x7)

#define SPI_CS_INIT		0x01
#define SPI_CS_ASSERT		0x02
#define SPI_CS_DROP		0x04

/* Quad SPI */
#define INT_DLPFIE		(0x1 << 31)
#define INT_TBFIE		(0x1 << 27)
#define INT_TBUIE		(0x1 << 26)
#define INT_ILLINIE		(0x1 << 23)
#define INT_RBOIE		(0x1 << 17)
#define INT_RBDIE               (0x1 << 16)
#define INT_ABSEIE              (0x1 << 15)
#define INT_ABOIE               (0x1 << 12)
#define INT_IUEIE               (0x1 << 11)
#define INT_IPAEIE              (0x1 << 7)
#define INT_IPIEIE              (0x1 << 6)
#define INT_IPGEIE              (0x1 << 4)
#define INT_TFIE                (0x1 << 0)

#define QUADSPI_MCR             0x00
#define QUADSPI_IPCR            0x08
#define QUADSPI_FLSHCR          0x0c
#define QUADSPI_BUF0CR          0x10
#define QUADSPI_BUF1CR          0x14
#define QUADSPI_BUF2CR          0x18
#define QUADSPI_BUF3CR          0x1c
#define QUADSPI_BFGENCR         0x20
#define QUADSPI_SOCCR           0x24
#define QUADSPI_BUF0IND         0x30
#define QUADSPI_BUF1IND         0x34
#define QUADSPI_BUF2IND         0x38
#define QUADSPI_SFAR            0x100
#define QUADSPI_SMPR            0x108
#define QUADSPI_RBSR            0x10c
#define QUADSPI_RBCT            0x110
#define QUADSPI_TBSR            0x150
#define QUADSPI_TBDR            0x154
#define QUADSPI_SR              0x15c
#define QUADSPI_FR              0x160
#define QUADSPI_RSER            0x164
#define QUADSPI_SPNDST          0x168
#define QUADSPI_SPTRCLR         0x16c
#define QUADSPI_SFA1AD          0x180
#define QUADSPI_SFA2AD          0x184
#define QUADSPI_SFB1AD          0x188
#define QUADSPI_SFB2AD          0x18c
#define QUADSPI_RBDR            0x200
#define QUADSPI_LUTKEY          0x300
#define QUADSPI_LCKCR           0x304
#define QUADSPI_LUT(x)          (0x310 + (x) * 4)

#define OPRND0(x)               (((x) & 0xff) << 0)
#define PAD0(x)                 (((x) & 0x3) << 8)
#define INSTR0(x)               (((x) & 0x3f) << 10)

#define OPRND1(x)               (((x) & 0xff) << 16)
#define PAD1(x)                 (((x) & 0x3) << 24)
#define INSTR1(x)               (((x) & 0x3f) << 26)

#define SEQU_CMD		0x1
#define SEQU_ADDR		0x2
#define SEQU_DUMMY		0x3
#define SEQU_MODE		0x4
#define SEQU_MODE2		0x5
#define SEQU_MODE4		0x6
#define SEQU_READ		0x7
#define SEQU_WRITE		0x8
#define SEQU_JMP_ON_CS		0x9
#define SEQU_ADDR_DDR		0xa
#define SEQU_MODE_DDR		0xb
#define SEQU_MODE2_DDR		0xc
#define SEQU_MODE4_DDR		0xd
#define SEQU_READ_DDR		0xe
#define SEQU_WRITE_DDR		0xf
#define SEQU_DATA_LEARN		0x10
#define SEQU_STOP		0x0

#endif /* SPI_MVF_H_ */
