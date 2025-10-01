/*******************************************************
 DesafioEcoTx V. 0.03

 * Heltec Wireless Tracker V1.2
 * - Lee GPS con HT_TinyGPS++
 * - Muestra en TFT con HT_st7735
 * - Envía por LoRa (driver Radio.* de LoRaWan_APP.h)
 * - Frecuencia LoRa: 433 MHz
 * - GPS: 115200 baudios (pines RX=33, TX=34)
 * - Sin parpadeo en TFT
 * - Envío cada 10 segundos
 * - Espera ACK del receptor (DesafioEcoRx) tras cada envío
 * - Reenvía el mismo paquete si no se recibe ACK
 * - Muestra estado ACK en TFT
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
#define ACK_RX_WINDOW_MS    5000    // Tiempo a escuchar ACK tras TX (ms)
#define BUFFER_SIZE			128		// Tamaño del buffer de TX (ampliado)

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
volatile bool lora_idle = true;			// Indica si el radio está libre para transmitir (volatile porque cambia en callbacks)
volatile bool waiting_for_ack = false;    // Indicador de estar en ventana Rx esperando ACK
volatile bool ack_received = false;       // ACK recibido para el paquete
uint32_t lastAckedPacket = 0;            // Último paquete confirmado por ACK
uint32_t packetCount = 0;				// Contador de paquetes enviados
unsigned long lastSendMs = 0;			// Marca de tiempo del último envío
const unsigned long sendIntervalMs = 10000;	// Intervalo de envío (10000 ms = 10 s)
static RadioEvents_t RadioEvents;			// Estructura de callbacks del driver de radio
String lastLat = "";						// Última latitud mostrada en TFT
String lastLon = "";						// Última longitud mostrada en TFT
String lastPkt = "";						// Último contador de paquetes mostrado
String lastAckStr = "";                    // Último estado ACK mostrado en TFT

// Nuevo: tiempo de inicio de la ventana ACK para detectar expiración en loop()
unsigned long ackWindowStartMs = 0;


// ========================
// Prototipos de callbacks de radio
// ========================
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

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
    tft.st7735_write_str(0, 60, (String)"    V 0.03    ");	// Version
    
    delay(3000);
    tft.st7735_fill_screen(ST7735_BLACK);					// Pantalla en negro
    tft.st7735_write_str(0, 0, (String)"  DesECO2025  ");	// Cabecera fija
    tft.st7735_write_str(0, 40, (String)"ACK:");           // Nueva etiqueta para estado ACK
    tft.st7735_write_str(0, 60, (String)"Paq:");
    // ===== SECCIÓN LoRa: Inicialización =====
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);		// Inicializa MCU (API Heltec)
    RadioEvents.TxDone = OnTxDone;				// Callback TX completada
    RadioEvents.TxTimeout = OnTxTimeout;		// Callback TX timeout
    RadioEvents.RxDone = OnRxDone;              // Callback RX completada
    RadioEvents.RxTimeout = OnRxTimeout;        // Callback RX timeout
    RadioEvents.RxError = OnRxError;            // Callback RX error
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
		String latStr = String(GPS.location.lat(), 2);
		String lonStr = String(GPS.location.lng(), 2);
		if (latStr != lastLat) {
			tft.st7735_fill_rectangle(5, 20, 75, 20, ST7735_BLACK);
			tft.st7735_write_str(5, 20, latStr);
			lastLat = latStr;
		}
		if (lonStr != lastLon) {
			tft.st7735_fill_rectangle(80, 20, 80, 20, ST7735_BLACK);
			tft.st7735_write_str(80, 20, lonStr);
			lastLon = lonStr;
		}
	} else {
		String msg = "No GPS!";
		if (msg != lastLat) {
			tft.st7735_fill_rectangle(5, 20, 155, 20, ST7735_BLACK);
			tft.st7735_write_str(5, 20, msg);
			lastLat = msg; lastLon = ""; // Resetea lon para cuando haya fix
		}
	}
	
    // Nueva sección: mostrar estado ACK en TFT (solo desde loop, no en IRQ)
    String ackStr;
    if (packetCount == 0) {
        ackStr = "-";
    } else if (waiting_for_ack) {
        ackStr = "Waiting...";
    } else if (ack_received) {
        ackStr = "OK #" + String(lastAckedPacket);
    } else if (lastAckedPacket < packetCount) {
        ackStr = "No ACK";
    } else {
        ackStr = "-";
    }
    if (ackStr != lastAckStr) {
        tft.st7735_fill_rectangle(42, 40, 118, 20, ST7735_BLACK);
        tft.st7735_write_str(42, 40, ackStr);
        lastAckStr = ackStr;
    }
	
    // Mostrar contador de paquetes enviados
	String pktStr = String(packetCount);
	if (pktStr != lastPkt) {
		tft.st7735_fill_rectangle(42, 60, 118, 20, ST7735_BLACK);
		tft.st7735_write_str(42, 60, pktStr);
		lastPkt = pktStr;
	}
	
    // Procesa interrupciones/estado del driver de radio
    Radio.IrqProcess();

    // Si la ventana ACK quedó abierta pero no se recibió callback, forzar expiración
    if (waiting_for_ack && ackWindowStartMs != 0 && (millis() - ackWindowStartMs > (ACK_RX_WINDOW_MS + 200UL))) {
        Serial.println("[ACK] ventana expiró (forzado) - reintentando envío");
        waiting_for_ack = false;
        lora_idle = true;
        // Ajustar packetCount para reenviar el mismo número
        if (packetCount > 0) packetCount--;
        // Forzar reenvío inmediato
        lastSendMs = millis() - sendIntervalMs;
        ackWindowStartMs = 0;
    }

    // ===== SECCIÓN LoRa: Envío =====
    if ((millis() - lastSendMs >= sendIntervalMs) && lora_idle && !waiting_for_ack) {
        lastSendMs = millis();
        packetCount++;
        ack_received = false; waiting_for_ack = false;
        char latBuf[32] = {0}, lonBuf[32] = {0};
        if (GPS.location.isValid()) {
            // Formatear lat/lon con dtostrf para evitar %f en snprintf
            dtostrf(GPS.location.lat(), 0, 6, latBuf);
            dtostrf(GPS.location.lng(), 0, 6, lonBuf);
            if (GPS.time.isValid()) {
                snprintf(txpacket, BUFFER_SIZE,
                        "Pkt:%lu Time %02d:%02d:%02d LAT %s LON %s",
                        (unsigned long)packetCount,
                        GPS.time.hour(), GPS.time.minute(), GPS.time.second(),
                        latBuf, lonBuf);
            } else {
                snprintf(txpacket, BUFFER_SIZE,
                        "Pkt:%lu LAT %s LON %s",
                        (unsigned long)packetCount,
                        latBuf, lonBuf);
            }
        } else {
            snprintf(txpacket, BUFFER_SIZE,
                    "Pkt:%lu GPS no fix",
                    (unsigned long)packetCount);
        }
        Serial.printf("TX -> \"%s\" (len %d)\n", txpacket, strlen(txpacket));
        Radio.Send((uint8_t *)txpacket, strlen(txpacket)); 
        lora_idle = false;
        // OnTxDone iniciará la ventana Rx para ACK
    }
}
// ===================================================== 
// Callbacks del driver de radio 
// ===================================================== 
void OnTxDone(void) { 
    Serial.println("TX done"); 
    // Abrir ventana Rx para recibir ACK
    ack_received = false;
    waiting_for_ack = true;
    lora_idle = false;
    ackWindowStartMs = millis();               // marcar inicio de ventana ACK
    Radio.Rx(ACK_RX_WINDOW_MS); // Escuchar ACK durante ventana
}

void OnTxTimeout(void) { 
    Radio.Sleep(); 
    Serial.println("TX Timeout"); 
    lora_idle = true; 
    waiting_for_ack = false;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Asegurar terminador
    char buf[BUFFER_SIZE];
    uint16_t copyLen = (size < (BUFFER_SIZE-1)) ? size : (BUFFER_SIZE-1);
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';
    Serial.printf("RX -> \"%s\" (len %d) RSSI %d SNR %d\n", buf, size, rssi, snr);
    // Esperamos ACK en formato "ACK:<num>"
    if (strncmp(buf, "ACK:", 4) == 0) {
        unsigned long ackNum = strtoul(buf + 4, NULL, 10);
        Serial.printf("ACK for pkt %lu\n", ackNum);
        ack_received = true;
        lastAckedPacket = ackNum;
    }
    waiting_for_ack = false;
    lora_idle = true;
    ackWindowStartMs = 0; // limpiar marca de ventana al recibir algo
}

void OnRxTimeout(void) {
    Serial.println("RX Timeout (no ACK)");
    waiting_for_ack = false;
    lora_idle = true;
}

void OnRxError(void) {
    Serial.println("RX Error");
    waiting_for_ack = false;
    lora_idle = true;
}