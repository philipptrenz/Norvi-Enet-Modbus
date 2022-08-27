/*
 * W5500 "hardware" MAC address.
 */
uint8_t eth_MAC[] = { 0x02, 0xF0, 0x0D, 0xBE, 0xEF, 0x01 };

/*
 * Network configuration.
 */
IPAddress eth_IP(192, 168, 1, 252);     // Static device IP address.
IPAddress eth_MASK(255, 255, 255, 0);   // Subnet mask.
IPAddress eth_DNS(192, 168, 1, 1);      // DNS server IP address.
IPAddress eth_GW(192, 168, 1, 1);       // IP address of network gateway.