/*******************************************************
 DesafioEcoTx V. 0.01

 * Heltec Wireless Tracker V1.2
 * - Lee GPS con HT_TinyGPS++
 * - Muestra en TFT con HT_st7735
 * - Envía por LoRa (driver Radio.* de LoRaWan_APP.h)
 * - Frecuencia LoRa: 433 MHz
 * - GPS: 115200 baudios (pines RX=33, TX=34)
 * - Sin parpadeo en TFT
 *******************************************************/
#include "Arduino.h"			// Núcleo Arduino
#include "LoRaWan_APP.h"		// Stack/driver de radio (SX1262) de Heltec
#include "HT_TinyGPS++.h"		// GPS (versión Heltec)
#include "HT_st7735.h"			// Pantalla TFT (versión Heltec)

// ========================
// Configuración LoRa
// ========================
#define RF_FREQUENCY			433000000	// Frecuencia en Hz (433 MHz)
#define TX_OUTPUT_POWER			18			// Potencia TX (dBm)
#define LORA_BANDWIDTH			0			// 0:125kHz, 1:250kHz, 2:500kHz
#define LORA_SPREADING_FACTOR	12			// SF7..SF12
#define LORA_CODINGRATE			4			// 1:4/5, 2:4/6, 3:4/7, 4:4/8
#define LORA_PREAMBLE_LENGTH	24			// Preamble
#define LORA_SYMBOL_TIMEOUT		0			// Timeout símbolos (no usado)
#define LORA_FIX_LENGTH_PAYLOAD_ON	false	// Payload variable
#define LORA_IQ_INVERSION_ON		false	// IQ normal

// ========================
// Miscelánea
// ========================
#define RX_TIMEOUT_VALUE	1000	// Timeout RX (ms) (no crítico para solo TX)
#define BUFFER_SIZE			96		// Tamaño del buffer de TX

// ========================
// Pines / objetos de hardware
// ========================
#define VGNSS_CTRL			3				// Pin de alimentación del GNSS (ON/OFF)
TinyGPSPlus GPS;							// Objeto parser GPS
HT_st7735 tft;								// Objeto pantalla TFT
HardwareSerial &GPS_Serial = Serial1;		// Serial del GPS en UART1 (RX=33, TX=34)

// ========================
// Variables de aplicación
// ========================
char txpacket[BUFFER_SIZE];				// Buffer para el paquete LoRa
bool lora_idle = true;					// Indica si el radio está libre para transmitir
uint32_t packetCount = 0;				// Contador de paquetes enviados
unsigned long lastSendMs = 0;			// Marca de tiempo del último envío
const unsigned long sendIntervalMs = 10000;	// Intervalo de envío (1 segundo)
static RadioEvents_t RadioEvents;			// Estructura de callbacks del driver de radio
String lastLat = "";						// Última latitud mostrada en TFT
String lastLon = "";						// Última longitud mostrada en TFT
String lastPkt = "";						// Último contador de paquetes mostrado


// ========================
// Prototipos de callbacks de radio
// ========================
void OnTxDone(void);
void OnTxTimeout(void);

// =====================================================
// setup()
// =====================================================
void setup() {
	Serial.begin(115200);									// Consola para depurar
	delay(200);
	Serial.println("\n[INIT] Wireless Tracker - GPS + LoRa @433MHz");
	// ===== SECCIÓN GPS: Alimentación y UART =====
	pinMode(VGNSS_CTRL, OUTPUT);							// Pin como salida
	digitalWrite(VGNSS_CTRL, HIGH);							// Encender GNSS
	GPS_Serial.begin(115200, SERIAL_8N1, 33, 34);			// Inicializa GPS a 115200 baudios
	// ===== SECCIÓN TFT: Inicialización =====
	tft.st7735_init();										// Inicializa controlador ST7735
	tft.st7735_fill_screen(ST7735_BLACK);					// Pantalla en negro
	tft.st7735_write_str(0, 0, (String)"GPS+LoRa433MHz");	// Título
	tft.st7735_write_str(0, 20, (String)"Iniciando...");	// Mensaje
	tft.st7735_write_str(0, 40, (String)" DesafioEcoTx ");	// Version
	tft.st7735_write_str(0, 60, (String)"    V 0.01    ");	// Version
	
   delay(3000);
	tft.st7735_fill_screen(ST7735_BLACK);					// Pantalla en negro
	tft.st7735_write_str(0, 0, (String)"  DesECO2025  ");	// Cabecera fija
	tft.st7735_write_str(0, 20, (String)"LAT:");
	tft.st7735_write_str(0, 40, (String)"LON:");
	tft.st7735_write_str(0, 60, (String)"Paq:");
	// ===== SECCIÓN LoRa: Inicialización =====
	Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);		// Inicializa MCU (API Heltec)
	RadioEvents.TxDone = OnTxDone;				// Callback TX completada
	RadioEvents.TxTimeout = OnTxTimeout;		// Callback TX timeout
	Radio.Init(&RadioEvents);					// Inicializa driver radio
	Radio.SetChannel(RF_FREQUENCY);				// Setea canal (433 MHz)
	Radio.SetTxConfig(	MODEM_LORA,
						TX_OUTPUT_POWER,		// Potencia TX (dBm)
						0,						// Freq deviation (FSK) - no usado
						LORA_BANDWIDTH,			// Ancho de banda
						LORA_SPREADING_FACTOR,	// Spreading Factor
						LORA_CODINGRATE,			// Coding Rate
						LORA_PREAMBLE_LENGTH,		// Preamble
						LORA_FIX_LENGTH_PAYLOAD_ON, // Payload variable
						true,						// CRC ON
						0,							// FreqHop - no usado
						0,							// HopPeriod - no usado
						LORA_IQ_INVERSION_ON,		// IQ normal
						3000 );						// Tiempo de espera TX (ms)
	Serial.println("[INIT] Listo. Comenzando loop...");
}

// =====================================================
// loop()
// =====================================================
void loop() {
	// ===== SECCIÓN GPS: Leer datos =====
	while (GPS_Serial.available() > 0) {
		GPS.encode(GPS_Serial.read());				// Alimenta el parser NMEA
	}
	
	// ===== SECCIÓN TFT: Actualización sin parpadeo =====
	// Imprimo titulo y cuadro de datos que no se actualizan
	if (GPS.location.isValid()) {			// Si la ubicación es válida
		String latStr = String(GPS.location.lat(), 6);
		String lonStr = String(GPS.location.lng(), 6);
		if (latStr != lastLat) {
			tft.st7735_fill_rectangle(42, 20, 86, 10, ST7735_BLACK);
			tft.st7735_write_str(42, 20, latStr);
			lastLat = latStr;
		}
		if (lonStr != lastLon) {
			tft.st7735_fill_rectangle(42, 40, 86, 10, ST7735_BLACK);
			tft.st7735_write_str(42, 40, lonStr);
			lastLon = lonStr;
		}
	} else {
		String msg = "No GPS!";
		if (msg != lastLat) {
			tft.st7735_fill_rectangle(42, 20, 86, 10, ST7735_BLACK);
			tft.st7735_write_str(42, 20, msg);
			lastLat = msg; lastLon = ""; // Resetea lon para cuando haya fix
		}
	}
	
	String pktStr = String(packetCount);
	if (pktStr != lastPkt) {
		tft.st7735_fill_rectangle(42, 60, 86, 10, ST7735_BLACK);
		tft.st7735_write_str(42, 60, pktStr);
		lastPkt = pktStr;
	}
	
	// ===== SECCIÓN LoRa: Envío =====
	if ((millis() - lastSendMs >= sendIntervalMs) && lora_idle) {
		lastSendMs = millis();
		packetCount++;
		if (GPS.location.isValid()) {
			if (GPS.time.isValid()) {
				snprintf(txpacket, BUFFER_SIZE,
						"Pkt:%lu Time %02d:%02d:%02d LAT %.6f LON %.6f",
						(unsigned long)packetCount,
						GPS.time.hour(), GPS.time.minute(), GPS.time.second(),
						GPS.location.lat(), GPS.location.lng());
			} else {
				snprintf(txpacket, BUFFER_SIZE,
						"Pkt:%lu LAT %.6f LON %.6f",
						(unsigned long)packetCount,
						GPS.location.lat(), GPS.location.lng());
			}
		} else {
			snprintf(txpacket, BUFFER_SIZE,
					"Pkt:%lu GPS no fix",
					(unsigned long)packetCount);
		}
		Serial.printf("TX -> \"%s\" (len %d)\n", txpacket, strlen(txpacket));
		Radio.Send((uint8_t *)txpacket, strlen(txpacket)); lora_idle = false;
	}
	// Procesa interrupciones/estado del driver de radio
	Radio.IrqProcess();
}
// ===================================================== 
// Callbacks del driver de radio 
// ===================================================== 
void OnTxDone(void) { 
	Serial.println("TX done"); 
	lora_idle = true; 
} 

void OnTxTimeout(void) { 
	Radio.Sleep(); 
	Serial.println("TX Timeout"); 
	lora_idle = true; 
}