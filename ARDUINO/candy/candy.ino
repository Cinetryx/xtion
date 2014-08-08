#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

byte mac[] = { 0x80, 0xA2, 0xDA, 0x0F, 0x88, 0x32 };
IPAddress ip( 172, 16, 14, 200 );
unsigned int localPort = 4000;

char packetBuffer[ UDP_TX_PACKET_MAX_SIZE ];
EthernetUDP Udp;

void setup() {
    Serial.begin( 9600 );
    Ethernet.begin( mac, ip );
    Udp.begin( localPort );
}

int get_command() {
    if( Udp.parsePacket() ) {
        Udp.read( packetBuffer, UDP_TX_PACKET_MAX_SIZE );

        if( packetBuffer[0] == 'G' ) {
            return 1;
        }
        else {
            return 0;
        }
    }
}

void action() {
    Serial.println("GO");
}

void loop() {
    int return_num = get_command();
    if (return_num) {
        action();
    }
}
