/**
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <robin.lilja@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return. - Robin Lilja
 *
 * @see page 519 in ATMEL SAM3X/A datasheet/manual for Peripheral DMA Controller (PDC) usage.
 *
 * @file DmaSerial.cpp
 * @author Robin Lilja
 * @date 18 Jan 2015
 */
#include "Arduino.h"
#include "DmaSerial.h"


DmaSerial::DmaSerial(Uart* uart, uint32_t uart_id) {

								this->uart = uart;
								this->uart_id = uart_id;
}

void DmaSerial::begin(const uint32_t baud) {

								PMC->PMC_WPMR = 0x504D4300; // Disable PMC register write-protection (see page 581)
								PMC->PMC_PCER0 = (1 << uart_id); // Enable Peripheral Clock

								// Enabling the PMC write-protection may break other lazy drivers..
								//PMC->PMC_WPMR = 0x504D4301;		// Enable PMC register write-protection (see page 581)

								// Disable
								uart->UART_CR = UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA | US_CR_RTSDIS;

								// Set 8N1 and baud rate
								//uart->UART_MR = SERIAL_8N1 | US_MR_USART_MODE_HW_HANDSHAKING;
								uart->UART_MR = SERIAL_8N1 | US_MR_USART_MODE_HW_HANDSHAKING;
								uart->UART_BRGR = (DMA_SERIAL_CLK / baud) / 16;

								// Disable interrupts
								uart->UART_IDR = 0xFFFFFFFF;
								uart->UART_IER = 0x00000000;

								uart->UART_TPR = (uint32_t)tx_buffer; // TPR initially points to beginning of buffer
								uart->UART_TNPR = (uint32_t)tx_buffer; // TNPR always points to beginning of buffer

								uart->UART_RPR = (uint32_t)rx_buffer; // RPR initially points to beginning of buffer
								uart->UART_RNPR = (uint32_t)rx_buffer; // RNPR always points to beginning of buffer

								uart->UART_RCR = DMA_SERIAL_RX_BUFFER_LENGTH;
								uart->UART_RNCR = 0;

								// Assert initial state of ring buffers
								tx_head = 0;
								tx_tail = 0;
								tx_count = 0;

								rx_head = 0;
								rx_tail = 0;
								rx_count = 0;

								PIO_Configure(
																g_APinDescription[PIN_RTS0].pPort,
																g_APinDescription[PIN_RTS0].ulPinType,
																g_APinDescription[PIN_RTS0].ulPin,
																g_APinDescription[PIN_RTS0].ulPinConfiguration);
								PIO_Configure(
																g_APinDescription[PIN_CTS0].pPort,
																g_APinDescription[PIN_CTS0].ulPinType,
																g_APinDescription[PIN_CTS0].ulPin,
																g_APinDescription[PIN_CTS0].ulPinConfiguration);

								// Enable communication
								uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}

void DmaSerial::end(){
								// Disable communications
								uart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

								PIO_Configure(
																g_APinDescription[PIN_RTS0].pPort,
																PIO_OUTPUT_0,
																g_APinDescription[PIN_RTS0].ulPin,
																g_APinDescription[PIN_RTS0].ulPinConfiguration);
								PIO_Configure(
																g_APinDescription[PIN_CTS0].pPort,
																PIO_INPUT,
																g_APinDescription[PIN_CTS0].ulPin,
																g_APinDescription[PIN_CTS0].ulPinConfiguration);

}

uint8_t DmaSerial::available() {

								uint8_t available = 0;

								// Disable receive PDC
								uart->UART_PTCR = UART_PTCR_RXTDIS;

								// Wait for PDC disable to take effect
								while (uart->UART_PTSR & UART_PTSR_RXTEN);

								// Calculate available bytes
								available = DMA_SERIAL_RX_BUFFER_LENGTH - uart->UART_RCR - uart->UART_RNCR;

								// Turn on receiver
								uart->UART_PTCR = UART_PTCR_RXTEN;

								return available;
}

uint8_t DmaSerial::get(uint8_t delimiter, uint8_t* bytes, uint8_t length) {

								// Disable receive PDC
								uart->UART_PTCR = UART_PTCR_RXTDIS;

								// Wait for PDC disable to take effect
								while (uart->UART_PTSR & UART_PTSR_RXTEN);

								// Modulus needed if RNCR is zero and RPR counts to end of buffer
								rx_tail = (uart->UART_RPR - (uint32_t)rx_buffer) % DMA_SERIAL_RX_BUFFER_LENGTH;

								// Make sure RPR follows (actually only needed if RRP is counted to the end of buffer and RNCR is zero)
								uart->UART_RPR = (uint32_t)rx_buffer + rx_tail;

								// Update fill counter
								rx_count = DMA_SERIAL_RX_BUFFER_LENGTH - uart->UART_RCR - uart->UART_RNCR;

								// No bytes in buffer to retrieve
								if (rx_count == 0) { uart->UART_PTCR = UART_PTCR_RXTEN; return 0; }

								uint8_t i = 0;

								while (length--) {

																bytes[i++] = rx_buffer[rx_head];

																// If buffer is wrapped, increment RNCR, else just increment the RCR
																if (rx_tail > rx_head) { uart->UART_RNCR++; } else { uart->UART_RCR++; }

																// Increment head and account for wrap around
																rx_head = (rx_head + 1) % DMA_SERIAL_RX_BUFFER_LENGTH;

																// Decrement counter keeping track of amount data in buffer
																rx_count--;

																// Buffer is empty
																if (rx_count == 0) { break; }

																// Delimiter is recived
																if(delimiter != 0xFF) {
																								if (bytes[i-1] == delimiter) {
																																break;
																								}
																}
								}

								// Turn on receiver
								uart->UART_PTCR = UART_PTCR_RXTEN;

								return i;
}

uint8_t DmaSerial::getln(uint8_t* bytes, uint8_t length) {
								return get('\n',bytes,length);
}

uint8_t DmaSerial::put(const char* str) {

								return put((uint8_t*)str, strlen(str));
}

uint8_t DmaSerial::putln(const char* str) {

								return put(str) + put("\r\n");
}

uint8_t DmaSerial::put(uint8_t* bytes, uint8_t length) {

								// Disable transmit PDC
								uart->UART_PTCR = UART_PTCR_TXTDIS;

								// Wait for PDC disable to take effect
								while (uart->UART_PTSR & UART_PTSR_TXTEN);

								// Modulus needed if TNCR is zero and TPR counts to end of buffer
								tx_head = (uart->UART_TPR - (uint32_t)tx_buffer) % DMA_SERIAL_TX_BUFFER_LENGTH;

								// Make sure TPR follows (actually only needed if TRP is counted to the end of buffer and TNCR is zero)
								uart->UART_TPR = (uint32_t)tx_buffer + tx_head;

								// Update fill counter, used for detecting when buffer is full
								tx_count = uart->UART_TCR + uart->UART_TNCR;

								// Buffer is full
								if (tx_count >= DMA_SERIAL_TX_BUFFER_LENGTH) {
																uart->UART_PTCR = UART_PTCR_TXTEN;
																return 0;
								}

								uint8_t i = 0;

								while (length--) {

																// Append bytes to buffer
																tx_buffer[tx_tail] = bytes[i++];

																// If buffer is wrapped, increment TNCR, else just increment the TCR
																if (tx_tail < tx_head) { uart->UART_TNCR++; } else { uart->UART_TCR++; }

																// Increment tail and account for wrap around
																tx_tail = (tx_tail + 1) % DMA_SERIAL_TX_BUFFER_LENGTH;

																// Increment counter keeping track of amount data in buffer
																tx_count++;

																// Buffer is full
																if (tx_count >= DMA_SERIAL_TX_BUFFER_LENGTH) { break; }
								}

								// Turn on the transfer
								uart->UART_PTCR = UART_PTCR_TXTEN;

								return i;
}

int DmaSerial::print(char* buff, uint8_t length) {
								int bytes_remaining, bytes_index, bytes_send;
								uint32_t t0 = millis();

								bytes_remaining = length;
								bytes_index = 0;
								while((bytes_remaining > 0) && (abs(millis() - t0) < _timeout)) {
																if((uart->UART_SR & US_CSR_TXBUFE) > 0) {
																								bytes_send = put((uint8_t*)&buff[bytes_index], bytes_remaining);
																								if(bytes_send > 0) {
																																bytes_remaining -= bytes_send;
																																bytes_index+= bytes_send;
																								}
																}
								}
								while((uart->UART_SR & US_CSR_TXBUFE) == 0) {yield();}
								return (length - bytes_remaining);
}

int DmaSerial::readBytesUntil(char delimiter, char* buffer, uint8_t length) {
								int recv_len = 0;
								int mem_remaining = length;
								int bytes_recv;
								uint32_t t0 = millis();

								while((mem_remaining > 0) && (abs(millis() - t0) < _timeout)) {
																if((uart->UART_SR & US_CSR_RXBUFF) > 0) {
																								bytes_recv = get(delimiter,(uint8_t*)&buffer[recv_len],mem_remaining);
																								if(bytes_recv > 0) {
																																recv_len += bytes_recv;
																																mem_remaining -= bytes_recv;
																																if(buffer[recv_len-1] == delimiter) {
																																								break;
																																}
																								}
																}
								}
								return recv_len;
}

void DmaSerial::setTimeout(unsigned long timeout) {
								_timeout = timeout;
}
