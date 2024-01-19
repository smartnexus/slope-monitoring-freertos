/* The prototypes of the following send and receive functions match that
expected by the transport interface's function pointers.  These simple
implementations show how to use the network context structure defined
above. */

#include "transport_interface.h"
#include "es_wifi.h"
#include "wifi.h"

void log_transport(char type,char *text,uint16_t len);
#define min(a,b) ((a) < (b) ? (a) : (b))

int32_t transport_recv( NetworkContext_t * pNetworkContext, void * pBuffer, size_t bytesToRecv) {
    int32_t socketStatus = 1;
    uint8_t ret;
    uint16_t recvlen;
	uint8_t recvdata[1024];

    ES_WIFI_Conn_t conn;

	conn.Number = pNetworkContext->socket;
	conn.RemotePort = pNetworkContext->remote_port;
	conn.LocalPort = 0;
	conn.Type = ES_WIFI_TCP_CONNECTION;
	conn.RemoteIP[0] = pNetworkContext->ipaddr[0];
	conn.RemoteIP[1] = pNetworkContext->ipaddr[1];
	conn.RemoteIP[2] = pNetworkContext->ipaddr[2];
	conn.RemoteIP[3] = pNetworkContext->ipaddr[3];

    if(!pNetworkContext->socket_open) {
    	ret=ES_WIFI_StartClientConnection(&EsWifiObj, &conn);

		if(ret!=ES_WIFI_STATUS_OK) {
			return 0;
		} else {
			pNetworkContext->socket_open=1;
		}
    }

	ret=ES_WIFI_ReceiveData(&EsWifiObj,pNetworkContext->socket, pBuffer, bytesToRecv, &recvlen, 1000);
	if(ret!=WIFI_STATUS_OK) {
		socketStatus=0;
		pNetworkContext->socket_open=0;
	} else {
		//log_transport('R',pBuffer,recvlen);
		recvdata[recvlen]=0;
		socketStatus=recvlen;
	}

    return socketStatus;
}

int32_t transport_send( NetworkContext_t * pNetworkContext, const void * pBuffer, size_t bytesToSend) {
    int32_t socketStatus=0;
    int16_t datasent;
    uint8_t ret;

    ES_WIFI_Conn_t conn;

	conn.Number = pNetworkContext->socket;
	conn.RemotePort = pNetworkContext->remote_port;
	conn.LocalPort = 0;
	conn.Type = ES_WIFI_TCP_CONNECTION;
	conn.RemoteIP[0] = pNetworkContext->ipaddr[0];
	conn.RemoteIP[1] = pNetworkContext->ipaddr[1];
	conn.RemoteIP[2] = pNetworkContext->ipaddr[2];
	conn.RemoteIP[3] = pNetworkContext->ipaddr[3];

    if(!pNetworkContext->socket_open) {
    	ret=ES_WIFI_StartClientConnection(&EsWifiObj, &conn);

		if(ret!=ES_WIFI_STATUS_OK) {
			return 0;
		} else {
			pNetworkContext->socket_open=1;
		}
    }

	ret=ES_WIFI_SendData(&EsWifiObj,pNetworkContext->socket,pBuffer,bytesToSend,&datasent,1000);
	//log_transport('S',pBuffer,bytesToSend);
	if(ret!=ES_WIFI_STATUS_OK) {
		pNetworkContext->socket_open=0;
		printf("Error in sending data: %d\n",ret);
		return 0;
	} else {
		socketStatus=datasent;
	}

    return socketStatus;
}

/* Populating the TransportInterface_t structure with the definitions above. */
void init_transport_from_socket( uint8_t tcpSocket, uint8_t socketOpen, NetworkContext_t * pNetworkContext, TransportInterface_t * pTransport) {
    pNetworkContext->socket = tcpSocket;
    pNetworkContext->socket_open=socketOpen;
    pTransport->recv = transport_recv;
    pTransport->send = transport_send;
    // We don't implement transport vector function
    pTransport->writev=NULL;
    pTransport->pNetworkContext = pNetworkContext;
}

void log_transport(char type,char *text,uint16_t len) {
	uint16_t i;
	printf("%c",type);
	for(i=0;i<min(len,511);++i) {
		printf(".%02X(%c)",(uint8_t)text[i],(uint8_t)text[i]);
	}
	printf(".(%d)\n",len);
}
