/* Base node code for CC2538 version microcontroller
	else if (strncmp(usart_rx_buffer,"NW",1) == 0)
    {
		//printf("\rMessage for a node from MANGO\r");
        //printf(usart_rx_buffer);
   		packetbuf_copyfrom(usart_rx_buffer, 110);
		//printf("Sending:");
		printf(usart_rx_buffer);
		printf("\r");
        strncpy(NodeAddress, usart_rx_buffer + 2, 2);
		addrToSend1 = strtol(NodeAddress, NULL, 16);
   		strncpy(NodeAddress, usart_rx_buffer + 5, 2);
		addrToSend2 = strtol(NodeAddress, NULL, 16);		
		strncpy(NodeAddress, usart_rx_buffer + 8, 2);
		addrToSend3 = strtol(NodeAddress, NULL, 16);
   		strncpy(NodeAddress, usart_rx_buffer + 11, 2);
		addrToSend4 = strtol(NodeAddress, NULL, 16);	

		printf("SendTo %02x:%02x:%02x:%02x\r", addrToSend1, addrToSend2, addrToSend3, addrToSend4);
    uip_ip6addr_u8(&sendTo, 170, 170, 0, 0, 0, 0, 0, 0, 2, 18, 75, 0, addrToSend1, addrToSend2, addrToSend3, addrToSend4);

	
    PRINTF("Sending NW command\r");
    uip_ipaddr_copy(&server_conn->ripaddr, &sendTo);
	//uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
	//printf("%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d.%03d\r", server_conn->ripaddr.u8[0], server_conn->ripaddr.u8[1], server_conn->ripaddr.u8[2], server_conn->ripaddr.u8[3], server_conn->ripaddr.u8[4], server_conn->ripaddr.u8[5], server_conn->ripaddr.u8[6], server_conn->ripaddr.u8[7], server_conn->ripaddr.u8[8], server_conn->ripaddr.u8[9], server_conn->ripaddr.u8[10], server_conn->ripaddr.u8[11], server_conn->ripaddr.u8[12], server_conn->ripaddr.u8[13], server_conn->ripaddr.u8[14], server_conn->ripaddr.u8[15]);
    uip_udp_packet_send(server_conn, usart_rx_buffer, sizeof(usart_rx_buffer));
    uip_create_unspecified(&server_conn->ripaddr);
   		
	}