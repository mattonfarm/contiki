/* Knob Source Code for CC2530 version microcontroller
 * Uses the mesh networking functunality of the RIME stack
 * Uses ContikiMAC for radio duty cycling
 */


//Included files
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
#include "lpm.h"
#include "dev/uart.h"
#include "dev/adc-sensor.h"
#include <stdint.h>
#include "cpu.h"
#include "net/netstack.h"
#include "random.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "dev/adc.h"
#include "dev/adc-sensor.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/sht11.h"

#define false 0
#define true 1

#define SEND_INTERVAL	(MeasurementPeriod * CLOCK_SECOND)			//This is the rate at which data packets are sent

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define MAX_PAYLOAD_LEN		150

#define DEBUG NONE
#include "net/ip/uip-debug.h"

//Global Variables
char ThisNodeAddress[12];											//String containing this nodes address (used for checking if a trickle message is for us)

static struct etimer periodic;										//Declare the timer used for sending measurements to the network

char usart_rx_buffer[500];											//Define a buffer for storing serial port strings
int usart_rx_buffer_index = 0;										//And an index for that buffer

unsigned char channel = 0x19;										//Set the RF channel to 0x19 by default			
unsigned char client[11] = {0,0,0,0,0,0,0,0,0,0,0};					//Declare a variable to store the client name (used as a reference only)
unsigned int MeasurementPeriod = 60;								//Default measurement period = 1 min

int16_t rv, dec;
float frac;
float A0;

//SHT75
float tc, hc;    
 unsigned int temp, humidity;

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

/*---------------------------- Contiki Processes ----------------------------*/
PROCESS(knob_collect_process, "knob Collect");
AUTOSTART_PROCESSES(&knob_collect_process);
/*---------------------------------------------------------------------------*/

// This extends the built in contiki microsecond delay routing to milliseconds
void delay_msec(int time) {
	while (time > 0) {
		clock_delay_usec(1000);
		time--;
		watchdog_periodic();
	}	
}

//This function takes readings from the ADC and sends them over the network to the
//sink (base node)
//Called when the event timer expires in the collect process main loop
/*---------------------------------------------------------------------------*/
void send_message() {
	char StringBuffer[MAX_PAYLOAD_LEN];				//Buffer for building the string of readings sent over the wireless network
	watchdog_periodic();				//Feed the dog
	static int seq_id;

	sprintf(StringBuffer, "Hello");				

	 seq_id++;
	//PRINTF("DATA send to %d seq %d'\n", server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
    uip_udp_packet_sendto(client_conn, StringBuffer, strlen(StringBuffer), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}

/*---------------------------------------------------------------------------*/
//This is the UART callback and is called every time the UART buffer has data in it
int uart_rx_callback(unsigned char c) {

	return 1;												//We always return 1 (success)
}

static void
tcpip_handler(void)
{

}

/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}

//This is the main Knob process.
//Here is where we set up I/O, timers and kick of the network processes
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(knob_collect_process, ev, data)
{
	PROCESS_BEGIN();

	PROCESS_PAUSE();

	set_global_address();
  
	PRINTF("UDP client process started\n");

	/* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
  UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
		

	etimer_set(&periodic, SEND_INTERVAL);											//Set up an event timer to send data back to base at a set interval
	
  while(1) {
	PROCESS_YIELD();								//Pause the process until an event is triggered
	if(ev == tcpip_event) {
      tcpip_handler();
    }
	if(etimer_expired(&periodic)) {					//The send message event timer expired variable

		send_message();
	}
  }
  PROCESS_END();
}
