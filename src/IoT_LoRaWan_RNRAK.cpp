
/*
 * LoRa Wan RN_SAMR34
 */
/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Written by Cristian Maglie

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "IoT_LoRaWan_RNRAK.H"


// Public ------------------------------------------------------------------------------
IoT_LoRaWan_RNRAK::IoT_LoRaWan_RNRAK( )
{
  loraSerial = nullptr;
  logMonitor = nullptr;
  loraReset = 0;
  STTS_LORA = 0;
  devclassmode = CLASS_C;
  devjoin = NO_JOIN;
  debuglevel = 1;
}

enum CLASS IoT_LoRaWan_RNRAK::setdevclass ( enum CLASS clase )
{
  if( clase == CLASS_A ) {
    devclassmode = CLASS_A;    
  } else if( clase == CLASS_C ) {
    devclassmode = CLASS_C;
  }
  return( devclassmode );
}

int IoT_LoRaWan_RNRAK::begin ( Uart *hwSerial )
{
  loraSerial = hwSerial;
  loraReset = 0;
  STTS_LORA = 0;
  logMonitor = nullptr;
  return( setup() );
}

int IoT_LoRaWan_RNRAK::begin ( Uart *hwSerial , int resetPIN )
{
  loraSerial = hwSerial;
  loraReset = resetPIN;
  STTS_LORA = 0;
  logMonitor = nullptr;
  return( setup() );
}

int IoT_LoRaWan_RNRAK::begin ( Uart *hwSerial , int resetPIN , int sttsPIN )
{
  loraSerial = hwSerial;
  loraReset = resetPIN;
  STTS_LORA = sttsPIN;
  logMonitor = nullptr;
  return( setup() );
}

int IoT_LoRaWan_RNRAK::begin ( Uart *hwSerial , int resetPIN , int sttsPIN  , Serial_ *monSerial )
{
  int i;
  loraSerial = hwSerial;
  loraReset = resetPIN;
  STTS_LORA = sttsPIN;
  logMonitor = monSerial;
  return( setup() );
}

int  IoT_LoRaWan_RNRAK::SetDevEUI( char *DevEUI )
{
  strcpy( deveui , DevEUI );
  return(1);
}

int IoT_LoRaWan_RNRAK::GetDevEUI(char *DevEUI )
{
  strcpy(  DevEUI , deveui );
  return(1);
}

int IoT_LoRaWan_RNRAK::GetHardEUI(char *DEVEUI )
{
  char command[20] = "mac get deveui";
  if( loraSerial == null ) {
      return(-1);
  }
  loraSerial->end();
  loraSerial->begin(LORA_RN_BAUDRATE);
  loraClearReadBuffer();
  if(( logMonitor != nullptr )&&( debuglevel >= 2 )) { 
    logMonitor->print("[LoRa]TX:");logMonitor->println(command);
  }
  loraSerial->println(command);
  loraWaitResponse(1000);
  strcpy( DEVEUI , response );
  loraSerial->end();
  return(1);
}

void IoT_LoRaWan_RNRAK::setADR( int mode )
{
  char comnando[80];
  lora_ADRmode = mode;
  if( lora_ADRmode ) {
    loraSendCommand(LORA_MAC_SET_ADR_ON);
  } else {
    loraSendCommand(LORA_MAC_SET_ADR_OFF);
  }
}

int IoT_LoRaWan_RNRAK::getADR( )
{
  return( lora_ADRmode );
}


int IoT_LoRaWan_RNRAK::joinOTAA( char *setAPPEUI , char *setAPPKEY ) 
{
  strcpy( appeui , setAPPEUI );
  strcpy( appkey ,  setAPPKEY );
  devjoin = OTAA;
  return( runjoinOTAA() );
}

int IoT_LoRaWan_RNRAK::joinOTAA( void )
{
  devjoin = OTAA;
  return( runjoinOTAA() );
}

int IoT_LoRaWan_RNRAK::joinABP( char *setDEVADDRESS , char *setAPPEUI , char *setNWKKEY , char *setAPPKEY ) 
{
  strcpy( devaddres , setDEVADDRESS );
  strcpy( appeui ,  setAPPEUI );
  strcpy( nwkkey , setNWKKEY );
  strcpy( appkey ,  setAPPKEY );
  devjoin = ABP;
  return( runjoinABP() );
}

void IoT_LoRaWan_RNRAK::process( void )
{
    char c,i;
    char *ptr;
    if( loraSerial == nullptr ) {
        return;
    }

    switch( pos_lora ) {
        case 0:
            if( loraWaitResponse(0) ) {
                if( strstr( response , LORA_DATA_RECIVE ) != 0 ) {
                    rx_lora_process();
                    pos_lora = 100;
                    break;
                }
            }
            if( STTS_LORA != null ) {
                digitalWrite( STTS_LORA , LOW );
            }
            if(( leng_datx > 0 )&&( millis() >= last_time_check )) { // espera x segundos luego del ultimo envio de mensaje
                loraSendmensaje();
                pos_lora = 1;
            }
            break;
      
        case 1:
            if( STTS_LORA != null ) {
                digitalWrite( STTS_LORA , HIGH );
            }
            loraClearReadBuffer();
            if( logMonitor != nullptr ) {
                logMonitor->println( "[LoRa] Send Data ");
                if( debuglevel >= 2 ) {
                  logMonitor->print("[LoRa]TX:");logMonitor->println(command);
                }
            }
            loraSerial->println(command);
            last_time_check = millis() + LORA_TIME_OUT * 1000;
            pos_lora++;
            break;

        case 2:
            if(( loraWaitResponse(0) )||( millis() >= last_time_check )) {
                if( strstr( response , LORA_DATA_RECIVE ) != 0 ) {
                    loraSendmensaje_OK();
                    rx_lora_process();
                } else if( strstr( response , LORA_INVALID_PARAM ) != 0 ) {
                    pos_lora = 30; // re set DR
                } else if( strstr( response , LORA_INVALID_DATA_LEN ) != 0 ) {
                    pos_lora = 30; // re set DR
                } else if( strstr( response , LORA_NO_FREE_CH ) != 0 ) {
                    pos_lora = 5;
                } else if( strstr( response , LORA_TX_OK ) != 0 ) {
                    loraSendmensaje_OK();
                    last_time_check = millis() + LORA_TIME_OUT * 1000;
                    pos_lora = 100;
                } else if( strstr( response , "ok" ) != 0 ) {
                    last_time_check = millis() + LORA_TIME_OUT * 1000;
                    pos_lora = 3;
                } else if( strstr( response , "busy" ) != 0 ) {
                    pos_lora = 5;
                } else if( strstr( response , "not_joined" ) != 0 ) {
                    pos_lora = 8;
                } else if( strstr( response , "mac_err" ) != 0 ) {
                    pos_lora = 5;
                } else {
                    if(( logMonitor != nullptr )&&( debuglevel >= 2 )) {
                      logMonitor->println("[LoRa] RnParser Time Out [001]");
                    }
                    pos_lora = 5;           
                }
                loraClearReadBuffer();
            }
            break;

        case 3:
            if(( loraWaitResponse(0) )||( millis() >= last_time_check )) {
                if( strstr( response , LORA_DATA_RECIVE ) != 0 ) {
                    loraSendmensaje_OK();
                    rx_lora_process();
                } else if( strstr( response , LORA_INVALID_PARAM ) != 0 ) {
                    pos_lora = 30; // re set DR
                } else if( strstr( response , LORA_INVALID_DATA_LEN ) != 0 ) {
                    pos_lora = 30; // re set DR
                } else if( strstr( response , LORA_NO_FREE_CH ) != 0 ) {
                    pos_lora = 5;
                } else if( strstr( response , LORA_TX_OK ) != 0 ) {
                    if( logMonitor != nullptr ) {
                      logMonitor->println( "[LoRa] Send Mesage Ok ");
                    }
                    loraSendmensaje_OK();
                    // Agregar muestre el FC RSSI DR y freq.
                    last_time_check = millis() + CADENCIA_TX_MINIMA * 1000;
                    pos_lora = 100;
                } else if( strstr( response , "mac_err" ) != 0 ) {
                    pos_lora = 5;
                } else {
                    if(( logMonitor != nullptr )&&( debuglevel >= 2 )) {
                      logMonitor->println("[LoRa] RnParser Time Out [002]");
                    }
                    pos_lora = 5;           
                }
            }
            break;

        case 5:
            // Agregar muestre el FC RSSI DR y freq.
            if(( logMonitor != nullptr )&&( debuglevel >= 2 )) {
                logMonitor->println("[LoRa] RETRY waiting...");
            }
            loraClearReadBuffer();
            last_time_check = millis() + LORA_DELAY_RETRY * 1000;
            pos_lora++;
            break;

        case 6:
            loraWaitResponse(0);
            if( millis() >= last_time_check ) {
                if(( logMonitor != nullptr )&&( debuglevel >= 2 )){
                    logMonitor->println("[LoRa] RETRY Now ");
                }
                loraClearReadBuffer();
                command_retry--;
                if( command_retry ) {
                    pos_lora = 1;
                } else {
                    rejoin();
                    last_time_check = millis() + CADENCIA_TX_MINIMA * 1000;
                    pos_lora = 100;
                }
            }
            break;
        
        case 8:
            if( logMonitor != nullptr ) {
                logMonitor->println("[LoRa] RnParser ERROR [003] ");
            }
            rejoin();
            last_time_check = millis() + CADENCIA_TX_MINIMA * 1000;
            pos_lora = 0;
            break;
      
        case 30:
            loraSendCommand( LORA_MAC_SET_DR );
            pos_lora = 5;
            break;

        case 100: // End
            loraClearReadBuffer();
            getLastMensajeStatus();
            pos_lora = 0;
            break;

    }
}

int IoT_LoRaWan_RNRAK::loraSendData ( char port , char *data , char dataSize )
{
  int i;
  if( lora_join_network == false ){
    return(-2);
  }
  if( leng_datx < LORA_MENS_BUFF_TX ) {
    lora_tx[fi_datx].port = port;
    lora_tx[fi_datx].largo_mensaje = dataSize;
    lora_tx[fi_datx].confirmacion = false;
    for( i = 0 ; i < dataSize ; i++ ) {
      lora_tx[fi_datx].mensaje[i] = data[i];
    }
    fi_datx++;
    if( fi_datx >= LORA_MENS_BUFF_TX ) {
      fi_datx = 0;
    }
    leng_datx++;
    return( true );
  } else {
    return( false );
  }
}

int IoT_LoRaWan_RNRAK::loraSendDataConf ( char port , char *data , char dataSize )
{
  int i;
  if( lora_join_network == false ){
    return(-2);
  }
  if( leng_datx < LORA_MENS_BUFF_TX ) {
    lora_tx[fi_datx].port = port;
    lora_tx[fi_datx].largo_mensaje = dataSize;
    lora_tx[fi_datx].confirmacion = true;
    for( i = 0 ; i < dataSize ; i++ ) {
      lora_tx[fi_datx].mensaje[i] = data[i];
    }
    fi_datx++;
    if( fi_datx >= LORA_MENS_BUFF_TX ) {
      fi_datx = 0;
    }
    leng_datx++;
    return( true );
  } else {
    return( false );
  }
}

int IoT_LoRaWan_RNRAK::loraReady ( void )
{
  if( lora_join_network == false ) {
    return( -1 );
  } else if( pos_lora == 0 ) {
    return( true );
  } else {
    return( false );
  }
}

int IoT_LoRaWan_RNRAK::loraTxBuffEmpy ( void ) 
{
  if( leng_datx == 0) {
    return( true );
  } else {
    return( false );
  }
}

int IoT_LoRaWan_RNRAK::loraRecibeWait( void )
{
  int ret = false;
  if( leng_dare > 0 ) {
    ret = true;
  }
  return( ret );
}

struct loraMensaje IoT_LoRaWan_RNRAK::loraReadData( void )
{
  struct loraMensaje respuesta;
  int i;
  i = 0;
  respuesta.port = 0;
  respuesta.mensaje[0] = 0;
  respuesta.largo_mensaje = 0;
  
  if( leng_dare > 0 ) {
    respuesta = lora_rx[fo_dare];
    leng_dare--;
    fo_dare++;
    if( fo_dare >= LORA_MENS_BUFF_RX ) {
      fo_dare = 0;
    }
  }
  return( respuesta );
}

int IoT_LoRaWan_RNRAK::SendCommand( char *comando , char *respuesta )
{
  int ret = loraSendCommand( comando );
  if( ret == true ) {
    strcpy( respuesta , response );
  }
  return( ret );
}

// Private ---------------------------------------------------------------------------------------

void IoT_LoRaWan_RNRAK::loraClearReadBuffer()
{
  int i;
  for ( i = 0 ; i < 80 ; i++ ) {
    response[i] = '\0';
  }
  response_count = 0;
}

int IoT_LoRaWan_RNRAK::loraWaitResponse(int timeout)
{
  int ret = false;
  char c;
  size_t read = 0;
  if( timeout > 0 ) {
    loraSerial->setTimeout(timeout);
    read = loraSerial->readBytesUntil('\n', response, 100);
    response[read - 1] = '\0'; // set \r to \0
    if(( logMonitor != nullptr )&&( debuglevel >= 2 )) { 
        logMonitor->print("[LoRa]RX:");logMonitor->println(response);
    }
    ret = true;
  } else {
    if( loraSerial->available() ) {
      c = loraSerial->read();
      if( c == '\n' ) {
        if(( logMonitor != nullptr )&&( debuglevel >= 2 )) { 
            logMonitor->print("[LoRa]RX:");logMonitor->println(response);
        }
        response[response_count] = '\0';
        response_count = 0;
        ret = true;
      } else {
        response[response_count++] = c;
        response[response_count] = '\0';
      }
    }
  }
  return( ret );
}

int IoT_LoRaWan_RNRAK::loraSendCommand(char *command)
{
   loraClearReadBuffer();
  if(( logMonitor != nullptr )&&( debuglevel >= 2 )) { 
    logMonitor->print("[LoRa]TX:");logMonitor->println(command);
  }
  loraSerial->println(command);
  return( loraWaitResponse(1000) );
}




void IoT_LoRaWan_RNRAK::loraSendmensaje ( void )
{
  char *p,*data;
  int i;
  uint8_t c;

  if(( millis() >= time_to_check_network )&&( Check_Network_Enable )){
    if( logMonitor != nullptr ) {  
        logMonitor->println("[LoRa] TIME OUT CHECK NETWORK is NOW");
    }
    lora_tx[fo_datx].confirmacion = true;
  }

#ifdef CNF_ALLWAYS
  lora_tx[fo_datx].confirmacion = true;
#endif

  if( lora_tx[fo_datx].confirmacion == true ) {
    sprintf(command, "mac tx cnf %d ", lora_tx[fo_datx].port);
  } else {
    sprintf(command, "mac tx uncnf %d ", lora_tx[fo_datx].port);
  }
  
  p = command + strlen(command);
  
  data = lora_tx[fo_datx].mensaje;
  
  for (i = 0; i < lora_tx[fo_datx].largo_mensaje; i++) {
    c = *data++;
    *p++ = hex[c >> 4];
    *p++ = hex[c & 0x0f];
  }
  *p++ = 0;
  command_retry = 3;
  pos_lora = 1;
}

void IoT_LoRaWan_RNRAK::loraSendmensaje_OK ( void )
{
  if( leng_datx ) {
      if( lora_tx[fo_datx].confirmacion == true ) {
        time_to_check_network = millis() +  TIME_OUT_CHECK_NETWORK * (long)1000;
      }
      leng_datx--;
      fo_datx++;
      if( fo_datx >= LORA_MENS_BUFF_TX ) {
        fo_datx = 0;
      }
  }
}

int IoT_LoRaWan_RNRAK::tohex( char dato )
{
  int ret = 0;
  if(( dato >= '0' )&&( dato <= '9' )){
    ret = dato - '0';
  } else if(( dato >= 'A' )&&( dato <= 'F' )) {
    ret = dato - 'A' + 10;
  } else if(( dato >= 'a' )&&( dato <= 'f' )) {
    ret = dato - 'a' + 10;
  } else {
    ret = -1;
  }
  return(ret);
}

char IoT_LoRaWan_RNRAK::gethex( char *data )
{
  char ret = 0;
  while( *data == ' ' ) data++;
  ret = tohex(*data++);
  ret = ret << 4;
  ret |=  tohex(*data++);
  return(ret);
}

int IoT_LoRaWan_RNRAK::setup( void )
{
  if( loraSerial == nullptr ) {
      return(-1);
  }
  loraSerial->end();
  loraSerial->begin(LORA_RN_BAUDRATE);
  if( loraReset != null ) {
      pinMode(loraReset, OUTPUT);
      digitalWrite(loraReset, LOW);
      delay(1000);
      while( loraSerial->available()) {
        loraSerial->read();
      } 
      digitalWrite(loraReset, HIGH);
  }
  if( STTS_LORA != null ) {
      pinMode( STTS_LORA , OUTPUT );
      digitalWrite( STTS_LORA , LOW );
  }

  if( logMonitor != nullptr ) {
      logMonitor->println("[LoRa] ========= LoRaWan SETUP ============================= ");
  }
  do {
    loraWaitResponse(3000);
  } while( strstr( response , "USER BOARD" ) == 0 );
  loraSendCommand("sys factoryRESET"); 
  do {
    loraWaitResponse(3000);
  } while( strstr( response , "USER BOARD" ) == 0 );
  loraSendCommand("mac reset au915");
  return( true );
}

int IoT_LoRaWan_RNRAK::runjoinOTAA( void )
{
  int port;
  char comando[80];
  int intento_numero = 0;
  
  if( loraSerial == null ) {
      return(-1);
  }

  intento_numero = 0;

  lora_join_network = false;

  do {
    setup();
    sprintf( comando , "mac set deveui %16s" , deveui );
    loraSendCommand(comando);

    sprintf( comando , "mac set joineui %16s", appeui); loraSendCommand(comando);
    sprintf( comando , "mac set appkey %32s", appkey); loraSendCommand(comando);
  
    if(( logMonitor != nullptr )&&( debuglevel >= 2 )) {
          logMonitor->println("[LoRa] Config Canales ");
        if( intento_numero == 0 ) {
          logMonitor->println("[LoRa] Open port 0 to 8 ");
        } else {
          logMonitor->println("[LoRa] Open port 0 to 63");
        }
    }
    for ( port = 0 ; port < 72 ; port++ ) {
      if(( intento_numero == 0 )&&(( port >= 0 )&&( port <= 7 ))) {
        sprintf( comando, "mac set ch status %d on", port);
        loraSendCommand(comando);
        sprintf( comando, "mac set ch drrange %d 0 5", port);
        loraSendCommand(comando);
      } else if(( intento_numero != 0 )&&(( port >= 0 )&&( port <= 63 ))) {
        sprintf( comando, "mac set ch status %d on", port);
        loraSendCommand(comando);
        sprintf( comando, "mac set ch drrange %d 0 5", port);
        loraSendCommand(comando);
      } else {
        sprintf( comando, "mac set ch status %d off", port);
        loraSendCommand(comando);
      }
    }
  
    setADR( lora_ADRmode );

    loraSendCommand(LORA_MAC_SET_DR);

    if( devclassmode == CLASS_C ) {
      loraSendCommand("mac set edclass c");
    } else {
      loraSendCommand("mac set edclass a");
    }

    if( logMonitor != nullptr ) {
      logMonitor->println("[LoRa] DEVICE EUI ----------------------------------------- ");
      loraSendCommand("mac get deveui");
    }

    if( logMonitor != nullptr ) {
        logMonitor->println("[LoRa] ---------------------------------------------------- ");
    }

    if( STTS_LORA != null ) {
        digitalWrite( STTS_LORA , HIGH );
    }

    if( logMonitor != nullptr ) {
        logMonitor->println("[LoRa] JOIN IN LoRaWan NETWORK ---------------------------- ");
    }

    do {
        if( logMonitor != nullptr ) {
            logMonitor->print("[LoRa] Intento de Join nro: "); logMonitor->println(intento_numero);
        }
        intento_numero++;
        delay(150);
        loraSendCommand("mac join otaa");
        loraWaitResponse(10000);
        if( strstr( response , "accepted" ) == 0  ) {
            delay( random( 5000 , 36000 ) ); // Random entre 5seg. y 36seg.
        } else {
          lora_join_network = true;
        }
    } while(( lora_join_network != true  )&&( intento_numero < 3 )); // Reintenta Reseteando
  } while(( lora_join_network != true )&&( intento_numero < 5 )) ; // Descarga el Join Inicial y Espera al proximo intento

  time_to_check_network = 0;
  fi_dare = 0;
  fo_dare = 0;
  leng_dare = 0;

  fi_datx = 0;
  fo_datx = 0;
  leng_datx = 0;

  if( logMonitor != nullptr ) {
    if( lora_join_network == true ) {
      logMonitor->println("[LoRa] JOIN ACCEPTED ");
    } else {
      logMonitor->println("[LoRa] JOIN NOT ACCEPTED ");
    }
  }

  if( STTS_LORA != null ) {
    digitalWrite( STTS_LORA , LOW );
  }
  loraSendCommand( LORA_MAC_SET_DR ); 
  loraClearReadBuffer();
  if( logMonitor != nullptr ) {
    logMonitor->println("[LoRa] Ready ");
  }
  time_to_check_network = millis() +  TIME_OUT_CHECK_NETWORK *  (long)1000;
  pos_lora = 0;
}

int IoT_LoRaWan_RNRAK::runjoinABP( void )
{
  int port;
  char comando[80];
  int intento_numero = 0;
  
  if( loraSerial == null ) {
      return(-1);
  }

  intento_numero = 0;

  lora_join_network = false;

  do {
    setup();
    sprintf( comando , "mac set deveui %16s" , deveui );
    loraSendCommand(comando);

    do {
      sprintf( comando , "mac set joineui %16s", appeui); loraSendCommand(comando);
      sprintf( comando , "mac set devaddr %8s",devaddres); loraSendCommand(comando);
      sprintf( comando , "mac set nwkskey %32s",nwkkey); loraSendCommand(comando);
      sprintf( comando , "mac set appskey %32s",appkey); loraSendCommand(comando);
    
      if( devclassmode == CLASS_C ) {
        loraSendCommand("mac set edclass c");
      } else {
        loraSendCommand("mac set edclass a");
      }

      if( logMonitor != nullptr ) {
          logMonitor->print("[LoRa] Intento de Join nro: "); logMonitor->println(intento_numero);
      }
      intento_numero++;
      delay(150);
      loraSendCommand("mac join abp");
      loraWaitResponse(10000);
      if( strstr( response , "accepted" ) == 0  ) {
          delay( random( 5000 , 36000 ) ); // Random entre 5seg. y 36seg.
      } else {
        lora_join_network = true;
      }
    } while(( lora_join_network != true  )&&( intento_numero < 3 )); // Reintenta Reseteando
  } while(( lora_join_network != true )&&( intento_numero < 5 )) ; // Descarga el Join Inicial y Espera al proximo intento

  if(( logMonitor != nullptr )&&( debuglevel >= 2 )) {
    logMonitor->println("[LoRa] Config Canales ");
  }
  for ( port = 0 ; port < 72 ; port++ ) {
    if(( port >= 0 )&&( port <= 7 )) {
      sprintf( comando, "mac set ch status %d on", port);
      loraSendCommand(comando);
      sprintf( comando, "mac set ch drrange %d 0 5", port);
      loraSendCommand(comando);
    } else {
      sprintf( comando, "mac set ch status %d off", port);
      loraSendCommand(comando);
    }
  }
  
  setADR( lora_ADRmode );

  loraSendCommand(LORA_MAC_SET_DR);

  if( STTS_LORA != null ) {
      digitalWrite( STTS_LORA , HIGH );
  }

  if( logMonitor != nullptr ) {
      logMonitor->println("[LoRa] JOIN IN LoRaWan NETWORK ---------------------------- ");
  }

  time_to_check_network = 0;
  fi_dare = 0;
  fo_dare = 0;
  leng_dare = 0;

  fi_datx = 0;
  fo_datx = 0;
  leng_datx = 0;

  if( logMonitor != nullptr ) {
    if( lora_join_network == true ) {
      logMonitor->println("[LoRa] JOIN ACCEPTED ");
    } else {
      logMonitor->println("[LoRa] JOIN NOT ACCEPTED ");
    }
  }

  if( STTS_LORA != null ) {
    digitalWrite( STTS_LORA , LOW );
  }
  loraSendCommand( LORA_MAC_SET_DR ); 
  loraClearReadBuffer();
  if( logMonitor != nullptr ) {
    logMonitor->println("[LoRa] Ready ");
  }
  time_to_check_network = millis() +  TIME_OUT_CHECK_NETWORK *  (long)1000;
  pos_lora = 0;
}

int IoT_LoRaWan_RNRAK::rejoin( void )
{
  if( devjoin == OTAA ) {
    return ( runjoinOTAA() );
  } else if( devjoin == ABP ) {
    return ( runjoinABP() );
  } else {
    return(-2);
  }
}

int IoT_LoRaWan_RNRAK::rx_lora_process( void )
{
  int i;
  char *ptr;
  char comm_debug[80];

  if( leng_dare < LORA_MENS_BUFF_RX ) {
    ptr = response;
    if( logMonitor != nullptr ) {
      logMonitor->print("[LoRa] DATA RX : {");
    }
    i = 0;
    while(!(( *ptr >= '0' )&&( *ptr <= '9' ))) ptr++; // Busca el Port
    lora_rx[fi_dare].port = atoi(ptr);
    while( *ptr != ' ' ) ptr++; // Buscar el espacio entre el Port y el mensaje
    ptr++; // pasa el espacio por alto.

    while(( *ptr != '\0' )&&( *ptr != 0x0D )) {
      sprintf( comm_debug , "%c%c,",*ptr,*(ptr+1));
      if( logMonitor != nullptr ) {
        logMonitor->print(comm_debug);
      }
      lora_rx[fi_dare].mensaje[i++] = gethex( ptr );
      ptr++;
      ptr++;
    }
    lora_rx[fi_dare].largo_mensaje = i;
    sprintf( comm_debug , " } = %d ",i);
    if( logMonitor != nullptr ) {
      logMonitor->println(comm_debug);
    }
    leng_dare++;
    fi_dare++;
    if( fi_dare >= LORA_MENS_BUFF_RX ) {
      fi_dare = 0;
    }
  }
}

void IoT_LoRaWan_RNRAK::getLastMensajeStatus( void )
{
    if( logMonitor != nullptr ) {
        logMonitor->println("[LoRa] Informacion de Ultimos UpLinks y DownLinks -------------------------");
    }
        GetRNdata        (" UpLink   Frame Count" , "mac get upctr");
        GetRNdata        (" UpLink      DataRate" , "mac get dr");
        GetRNdata        (" UpLink     Power Set" , "mac get pwridx");
        GetRNdata        (" DownLink Frame Count" , "mac get dnctr");
    last_RSSI = GetRNdata (" DownLink        RSSI" , "mac get pktrssi");
    if( last_RSSI > 0 ) {
        last_RSSI = -256 + last_RSSI;
    }
    //last_SNR  = GetRNdata (" DownLink         SNR" , "radio get snr");
    if( logMonitor != nullptr ) {
        logMonitor->println("---------------------------------------------------------------------------");
    }
}

int IoT_LoRaWan_RNRAK::GetRNdata( char *text , char *command )
{
  int timeout = 1000;
  int read;
  char c;
  int ret = 0;
  loraClearReadBuffer();
  if( logMonitor != nullptr ) {
    logMonitor->print("[LoRa]");logMonitor->print(text);
  }
  loraSerial->println(command);
  loraSerial->setTimeout(timeout);
  read = loraSerial->readBytesUntil('\n', response, 100);
  response[read - 1] = '\0'; // set \r to \0
  ret = atoi( response );
  if( logMonitor != nullptr ) {
    logMonitor->print(" = ");logMonitor->println(ret);
  }
  return( ret );
}

void IoT_LoRaWan_RNRAK::MonitorOFF ()
{
  logMonitor = nullptr;
}

void IoT_LoRaWan_RNRAK::MonitorON ( Serial_ *monSerial )
{
  logMonitor = monSerial;
}
