#ifndef _DMAUIO_H
#define _DMAUIO_H

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/mman.h>
#include <pthread.h>

#define MM2S_CONTROL_REGISTER       0x00 // MM2S DMA Control register, RW
#define MM2S_STATUS_REGISTER        0x04 // MM2S DMA Status registe, RO
// 0x08 - 0x14 : reserved, do not use, use in sg mode
#define MM2S_SRC_ADDRESS_REGISTER   0x18 // MM2S Source Address. 
#define MM2S_TRNSFR_LENGTH_REGISTER 0x28 // MM2S Transfer Length (number of bytes to transfer)

#define S2MM_CONTROL_REGISTER       0x30 // S2MM DMA Control register
#define S2MM_STATUS_REGISTER        0x34 // S2MM DMA Status register
#define S2MM_DST_ADDRESS_REGISTER   0x48 // S2MM Destination Address
#define S2MM_BUFF_LENGTH_REGISTER   0x58 // S2MM Buffer Length (number of bytes to transfer)

/* 	MM2S_DMACR (MM2S DMA Control Register – Offset 00h) R/W
   	This register provides control for the Memory Map to Stream DMA Channel */
#define IOC_IRQ_FLAG                1<<12 // IOC_IRQ: Interrupt on complete enable bit,1=enable,0=disable
#define IDLE_FLAG                   1<<1  // ?

/* 	MM2S_DMASR (MM2S DMA Status Register – Offset 04h) RO
	This register provides the status for the Memory Map to Stream DMA Channel */
// DMA Channel Halted. Indicates the run/stop state of the DMA channel.
// 0 = DMA channel running. 1 = DMA channel halted
#define STATUS_HALTED               0x00000001 
// DMA Channel Idle. Indicates the state of AXI DMA operations.
// 0 = Not Idle(when DMASR.Halted=1). 1 = Idle
#define STATUS_IDLE                 0x00000002
// 1= Scatter Gather Enabled, 0= Scatter Gather not enabled
#define STATUS_SG_INCLDED           0x00000008
#define STATUS_DMA_INTERNAL_ERR     0x00000010
#define STATUS_DMA_SLAVE_ERR        0x00000020
#define STATUS_DMA_DECODE_ERR       0x00000040
#define STATUS_SG_INTERNAL_ERR      0x00000100
#define STATUS_SG_SLAVE_ERR         0x00000200
#define STATUS_SG_DECODE_ERR        0x00000400
#define STATUS_IOC_IRQ              0x00001000
#define STATUS_DELAY_IRQ            0x00002000
#define STATUS_ERR_IRQ              0x00004000

// S2MM_DMACR (S2MM DMA Control Register – Offset 30h) R/W
#define HALT_DMA                    0x00000000 // Halt DMA
#define RUN_DMA                     0x00000001 // Run DMA
#define RESET_DMA                   0x00000004 // Reset DMA (Keyhole write)
#define ENABLE_IOC_IRQ              0x00001000 // Interrupt on Complete Interrupt Enable.
#define ENABLE_DELAY_IRQ            0x00002000 // Interrupt on Delay Timer Interrupt Enable. Applicable only when Scatter Gather is enabled
#define ENABLE_ERR_IRQ              0x00004000 // Error Interrupt Enable. Applicable only when Scatter Gather is enabled
#define ENABLE_ALL_IRQ              0x00007000 // Enable all interrupts

// void *uio_thread(void *data);
unsigned int write_dma(unsigned int *virtual_addr, int offset, unsigned int value);
unsigned int read_dma(unsigned int *virtual_addr, int offset);
void dma_s2mm_status(unsigned int *virtual_addr);
void dma_mm2s_status(unsigned int *virtual_addr);
int dma_mm2s_sync(unsigned int *virtual_addr);
int dma_s2mm_sync(unsigned int *virtual_addr);
void print_mem(void *virtual_address, int byte_count);

#endif