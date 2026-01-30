// SPDX-License-Identifier: GPL-2.0-or-later

/*
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
 *   Inspired from original code from Kolja Waschk's USB-JTAG project
 *   (http://www.ixo.de/info/usb_jtag/), and from openocd project.
 *
 *   Copyright (C) 2025 Ahmed Noman ahmed-noman3@outlook.com
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <stdint.h>
#include "stdio.h"

#include "../include/axi.h"
#include "../include/definitions.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/mman.h>


unsigned int write_dma(unsigned int *virtual_addr, int offset, unsigned int value)
{
    virtual_addr[offset>>2] = value;

    return 0;
}

unsigned int read_dma(unsigned int *virtual_addr, int offset)
{
    return virtual_addr[offset>>2];
}

void dma_s2mm_status(unsigned int *virtual_addr)
{
    unsigned int status = read_dma(virtual_addr, S2MM_STATUS_REGISTER);

    printf("Stream to memory-mapped status (0x%08x@0x%02x):", status, S2MM_STATUS_REGISTER);

    if (status & STATUS_HALTED) {
		printf(" Halted.\n");
	} else {
		printf(" Running.\n");
	}

    if (status & STATUS_IDLE) {
		printf(" Idle.\n");
	}

    if (status & STATUS_SG_INCLDED) {
		printf(" SG is included.\n");
	}

    if (status & STATUS_DMA_INTERNAL_ERR) {
		printf(" DMA internal error.\n");
	}

    if (status & STATUS_DMA_SLAVE_ERR) {
		printf(" DMA slave error.\n");
	}

    if (status & STATUS_DMA_DECODE_ERR) {
		printf(" DMA decode error.\n");
	}

    if (status & STATUS_SG_INTERNAL_ERR) {
		printf(" SG internal error.\n");
	}

    if (status & STATUS_SG_SLAVE_ERR) {
		printf(" SG slave error.\n");
	}

    if (status & STATUS_SG_DECODE_ERR) {
		printf(" SG decode error.\n");
	}

    if (status & STATUS_IOC_IRQ) {
		printf(" IOC interrupt occurred.\n");
	}

    if (status & STATUS_DELAY_IRQ) {
		printf(" Interrupt on delay occurred.\n");
	}

    if (status & STATUS_ERR_IRQ) {
		printf(" Error interrupt occurred.\n");
	}
}

void dma_mm2s_status(unsigned int *virtual_addr)
{
    unsigned int status = read_dma(virtual_addr, MM2S_STATUS_REGISTER);

    printf("Memory-mapped to stream status (0x%08x@0x%02x):", status, MM2S_STATUS_REGISTER);

    if (status & STATUS_HALTED) {
		printf(" Halted.\n");
	} else {
		printf(" Running.\n");
	}

    if (status & STATUS_IDLE) {
		printf(" Idle.\n");
	}

    if (status & STATUS_SG_INCLDED) {
		printf(" SG is included.\n");
	}

    if (status & STATUS_DMA_INTERNAL_ERR) {
		printf(" DMA internal error.\n");
	}

    if (status & STATUS_DMA_SLAVE_ERR) {
		printf(" DMA slave error.\n");
	}

    if (status & STATUS_DMA_DECODE_ERR) {
		printf(" DMA decode error.\n");
	}

    if (status & STATUS_SG_INTERNAL_ERR) {
		printf(" SG internal error.\n");
	}

    if (status & STATUS_SG_SLAVE_ERR) {
		printf(" SG slave error.\n");
	}

    if (status & STATUS_SG_DECODE_ERR) {
		printf(" SG decode error.\n");
	}

    if (status & STATUS_IOC_IRQ) {
		printf(" IOC interrupt occurred.\n");
	}

    if (status & STATUS_DELAY_IRQ) {
		printf(" Interrupt on delay occurred.\n");
	}

    if (status & STATUS_ERR_IRQ) {
		printf(" Error interrupt occurred.\n");
	}
}

int dma_mm2s_sync(unsigned int *virtual_addr)
{
    unsigned int mm2s_status =  read_dma(virtual_addr, MM2S_STATUS_REGISTER);

	// sit in this while loop as long as the status does not read back 0x00001002 (4098)
	// 0x00001002 = IOC interrupt has occured and DMA is idle
	while(!(mm2s_status & IOC_IRQ_FLAG) || !(mm2s_status & IDLE_FLAG))
	{
        dma_s2mm_status(virtual_addr);
        dma_mm2s_status(virtual_addr);

        mm2s_status =  read_dma(virtual_addr, MM2S_STATUS_REGISTER);
    }

	return 0;
}

int dma_s2mm_sync(unsigned int *virtual_addr)
{
    unsigned int s2mm_status = read_dma(virtual_addr, S2MM_STATUS_REGISTER);

	// sit in this while loop as long as the status does not read back 0x00001002 (4098)
	// 0x00001002 = IOC interrupt has occured and DMA is idle
	while(!(s2mm_status & IOC_IRQ_FLAG) || !(s2mm_status & IDLE_FLAG))
	{
        dma_s2mm_status(virtual_addr);
        dma_mm2s_status(virtual_addr);

        s2mm_status = read_dma(virtual_addr, S2MM_STATUS_REGISTER);
    }

	return 0;
}

void print_mem(void *virtual_address, int byte_count)
{
	char *data_ptr = virtual_address;

	for(int i=0;i<byte_count;i++){
		printf("%02X", data_ptr[i]);

		// print a space every 4 bytes (0 indexed)
		if(i%4==3){
			printf(" ");
		}
	}

	printf("\n");
}
/*
int main(int argc, char **argv)
{


	
	
	printf("Writing random data to source register block...\n");
	virtual_src_addr[0]= 0xEFBEADDE;
	virtual_src_addr[1]= 0x11223344;
	virtual_src_addr[2]= 0xABABABAB;
	virtual_src_addr[3]= 0xCDCDCDCD;
	virtual_src_addr[4]= 0x00001111;
	virtual_src_addr[5]= 0x22223333;
	virtual_src_addr[6]= 0x44445555;
	virtual_src_addr[7]= 0x66667777;

	printf("Clearing the destination register block...\n");
    memset(virtual_dst_addr, 0, 32);

    printf("Source memory block data:      ");
	print_mem(virtual_src_addr, 32);

    printf("Destination memory block data: ");
	print_mem(virtual_dst_addr, 32);

    printf("Reset the DMA.\n");
    write_dma(dma_virtual_addr, S2MM_CONTROL_REGISTER, RESET_DMA);
    write_dma(dma_virtual_addr, MM2S_CONTROL_REGISTER, RESET_DMA);
    dma_s2mm_status(dma_virtual_addr);
    dma_mm2s_status(dma_virtual_addr);

	printf("Halt the DMA.\n");
    write_dma(dma_virtual_addr, S2MM_CONTROL_REGISTER, HALT_DMA);
    write_dma(dma_virtual_addr, MM2S_CONTROL_REGISTER, HALT_DMA);
    dma_s2mm_status(dma_virtual_addr);
    dma_mm2s_status(dma_virtual_addr);

	printf("Enable all interrupts.\n");
    write_dma(dma_virtual_addr, S2MM_CONTROL_REGISTER, ENABLE_ALL_IRQ);
    write_dma(dma_virtual_addr, MM2S_CONTROL_REGISTER, ENABLE_ALL_IRQ);
    dma_s2mm_status(dma_virtual_addr);
    dma_mm2s_status(dma_virtual_addr);

    printf("Writing source address of the data from MM2S in DDR...\n");
    write_dma(dma_virtual_addr, MM2S_SRC_ADDRESS_REGISTER, base_src);
    dma_mm2s_status(dma_virtual_addr);

    printf("Writing the destination address for the data from S2MM in DDR...\n");
    write_dma(dma_virtual_addr, S2MM_DST_ADDRESS_REGISTER, base_dist);
    dma_s2mm_status(dma_virtual_addr);

	printf("Run the MM2S channel.\n");
    write_dma(dma_virtual_addr, MM2S_CONTROL_REGISTER, RUN_DMA);
    dma_mm2s_status(dma_virtual_addr);

	printf("Run the S2MM channel.\n");
    write_dma(dma_virtual_addr, S2MM_CONTROL_REGISTER, RUN_DMA);
    dma_s2mm_status(dma_virtual_addr);

    printf("Writing MM2S transfer length of 32 bytes...\n");
    write_dma(dma_virtual_addr, MM2S_TRNSFR_LENGTH_REGISTER, 32);
    dma_mm2s_status(dma_virtual_addr);

    printf("Writing S2MM transfer length of 32 bytes...\n");
    write_dma(dma_virtual_addr, S2MM_BUFF_LENGTH_REGISTER, 32);
    dma_s2mm_status(dma_virtual_addr);

    printf("Waiting for MM2S synchronization...\n");
    dma_mm2s_sync(dma_virtual_addr);

    printf("Waiting for S2MM sychronization...\n");
    dma_s2mm_sync(dma_virtual_addr);

    dma_s2mm_status(dma_virtual_addr);
    dma_mm2s_status(dma_virtual_addr);

    printf("Destination memory block: ");
	print_mem(virtual_dst_addr, 32);

	munmap (((void *)dma_base), 0xFFFF);
	munmap (((void *)base_src), 0xFFFF);
	munmap (((void *)base_dist), 0xFFFF);

	printf("\n");

    return 0;
}

*/
static int axi_read(struct axi_driver *low, uint8_t *buf,
			    unsigned int size, uint32_t *bytes_read)
{

	printf("read from axi is done now\n");
	// int retval;
	// int timeout = 100;
	// struct ftdi_context *ftdic = ublast_getftdic(low);
// 
	// *bytes_read = 0;
	// while ((*bytes_read < size) && timeout--) {
		// retval = ftdi_read_data(ftdic, buf + *bytes_read,
				// size - *bytes_read);
		// if (retval < 0)	{
			// *bytes_read = 0;
			// LOG_ERROR("ftdi_read_data: %s",
					// ftdi_get_error_string(ftdic));
			// return ERROR_JTAG_DEVICE_ERROR;
		// }
		// *bytes_read += retval;
	// }
	return ERROR_OK;
}

static int axi_write(struct axi_driver *low, uint8_t *buf, int size,
			     uint32_t *bytes_written)
{
	printf("write to axi is done now\n");
	// int retval;
	// struct ftdi_context *ftdic = ublast_getftdic(low);
// 
	// retval = ftdi_write_data(ftdic, buf, size);
	// if (retval < 0)	{
		// *bytes_written = 0;
		// LOG_ERROR("ftdi_write_data: %s",
			//   ftdi_get_error_string(ftdic));
		// return ERROR_JTAG_DEVICE_ERROR;
	// }
	// *bytes_written = retval;
	return ERROR_OK;
}

static int axi_init(struct axi_driver *low)
{
	printf("init axi\n");

	int dma_base 	= 0x40000000;
	int base_src 	= 0x0e000000;
	int base_dist 	= 0x0e001000;

	int dma_address_space = 0xFFFF;
	int src_address_space = 0xFFFF;
	int dist_address_space = 0xFFFF;

	int ddr_memory = open("/dev/mem", O_RDWR | O_SYNC);
	if (ddr_memory == NULL) {
		return ERROR_FAIL;
		printf("Fail opening a character device file of the Arty's DDR memeory...\n");
	}
	
    unsigned int *dma_virtual_addr = mmap(NULL, dma_address_space, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, dma_base);
	if (dma_virtual_addr < 0) {
		return ERROR_FAIL;
		printf("Fail to map the address of the DMA AXI IP via its AXI lite control interface register block.\n");
	}

    unsigned int *virtual_src_addr  = mmap(NULL, src_address_space, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, base_src);
	if (virtual_src_addr < 0) {
		return ERROR_FAIL;
		printf("Fail to map the MM2S source address register block.\n");

	}

    unsigned int *virtual_dst_addr = mmap(NULL, dist_address_space, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, base_dist);
	if (virtual_dst_addr < 0) {
		return ERROR_FAIL;
		printf("Fail to map the S2MM destination address register block.\n");
	}

	close(ddr_memory);
	/*
	uint8_t latency_timer;
	struct ftdi_context *ftdic = ublast_getftdic(low);

	LOG_INFO("usb blaster interface using libftdi");
	if (ftdi_init(ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	// context, vendor id, product id
	if (ftdi_usb_open(ftdic, low->ublast_vid, low->ublast_pid) < 0)	{
		LOG_ERROR("unable to open ftdi device: %s", ftdic->error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(ftdic) < 0) {
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(ftdic, 2) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(ftdic, &latency_timer) < 0)
		LOG_ERROR("unable to get latency timer");
	else
		LOG_DEBUG("current latency timer: %u", latency_timer);

	*/
	return ERROR_OK;
}

static int axi_quit(struct axi_driver *low)
{
	// struct ftdi_context *ftdic = ublast_getftdic(low);
	// 
	// ftdi_usb_close(ftdic);
	// ftdi_deinit(ftdic);
	printf("quiting axi now\n");
	return ERROR_OK;
};

static struct axi_driver low = {
	.open = axi_init,
	.close = axi_quit,
	.read = axi_read,
	.write = axi_write
};

struct axi_driver *axi_driver_register(void)
{
	return &low;
}
