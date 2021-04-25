/*
 * LoRa Wan Header
 */

#include <Arduino.h>

//#define CNF_ALLWAYS

#define LORA_RN_BAUDRATE            115200

#define LORA_MENS_BUFF_RX           10
#define LORA_MENS_BUFF_TX           20

#define CADENCIA_TX_MINIMA          1       // 1 Segundos entre mensajes

#define TIME_OUT_CHECK_NETWORK      600L    // 10 minutos x 60 segundos = 600

#define LORA_DELAY_RETRY            5       // 5 Segundos espera entre reintenos

#define LORA_TIME_OUT               30      // 30 Segundos espera TX mac_tx_ok

#define LORA_DATA_RECIVE            "mac_rx"
#define LORA_INVALID_PARAM          "invalid_param"
#define LORA_INVALID_DATA_LEN       "invalid_data_len"
#define LORA_NO_FREE_CH             "no_free_ch"
#define LORA_TX_OK                  "mac_tx_ok"
#define LORA_MAC_SET_ADR_ON         "mac set adr on"
#define LORA_MAC_SET_ADR_OFF        "mac set adr off"
#define LORA_MAC_SET_DR             "mac set dr 0"

#define null                        0
struct loraMensaje {
  unsigned int port;
  char largo_mensaje;
  char mensaje[100];
  char confirmacion;
};

enum CLASS{
  CLASS_A,
  CLASS_C,
};

enum MODEJOIN {
  NO_JOIN,
  OTAA,
  ABP,
};

class IoT_LoRaWan_RNRAK {
    private:
        Uart *loraSerial;
        Serial_ *logMonitor;
        
        enum CLASS devclassmode;
        enum MODEJOIN devjoin;
        int loraReset;
        int STTS_LORA;
        int pos_lora = 0;
        char command[256];
        char command_retry;
        char response[256],response_count;
        long time_to_check_network;
        long last_time_check;
        struct loraMensaje lora_rx[ LORA_MENS_BUFF_RX ];
        int fi_dare , fo_dare , leng_dare;
        struct loraMensaje lora_tx[ LORA_MENS_BUFF_TX ];
        int fi_datx , fo_datx , leng_datx;
        char hex[16]    = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };

        int lora_ADRmode = false;
        int lora_join_network = false;
        char deveui[17] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
        char devaddres[9] = { '0','1','2','3','4','5','6','7' };
        char appeui[17] = { '1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1' };
        char nwkkey[37] = { '1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1' };
        char appkey[37] = { '1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1' };

        void loraClearReadBuffer();
        int loraWaitResponse( int timeout );
        int loraSendCommand(char *command);
        void loraSendmensaje();
        void loraSendmensaje_OK();
        int tohex( char dato );
        char gethex( char *data );
        void getLastMensajeStatus( void );
        int GetRNdata( char *text , char *command );
        int setup( void );
        int runjoinOTAA( void );
        int runjoinABP( void );
        int rejoin( void );
        int rx_lora_process( void );

    public:
        char debuglevel = 1;
        int Check_Network_Enable = true;
        int last_RSSI;
        //int last_SNR;

        IoT_LoRaWan_RNRAK();

        enum CLASS setdevclass ( enum CLASS clase );

        int GetHardEUI( char *DEVEUI );
        int GetDevEUI(char *DevEUI );
        int SetDevEUI( char *DevEUI );

        void setADR( int mode );
        int getADR();

        virtual int begin ( Uart *hwSerial , int resetPIN , int sttsPIN  , Serial_ *monSerial  );
        virtual int begin ( Uart *hwSerial , int resetPIN , int sttsPIN  );
        virtual int begin ( Uart *hwSerial , int resetPIN );
        virtual int begin ( Uart *hwSerial );


        virtual int joinOTAA( void );
        virtual int joinOTAA( char *setAPPEUI , char *setAPPKEY ) ;

        int joinABP( char *setDEVADDRESS , char *setAPPEUI , char *setNWKKEY , char *setAPPKEY ) ;

        int loraSendData ( char port , char *data , char dataSize );
        int loraSendDataConf ( char port , char *data , char dataSize );
        int loraReady ( void );
        int loraTxBuffEmpy( void );
        int loraRecibeWait( void );
        struct loraMensaje loraReadData( void );
        void process( void );

        int SendCommand( char *comando , char *respuesta );

        void MonitorOFF();
        void MonitorON( Serial_ *monSerial );
};
