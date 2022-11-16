#include "configuration.h"                              /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "hardware.h"                                   /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "system.h"                                     /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "receiverdatatypes.h"                          /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "signalmanagement.h"                           /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "navigation.h"                                 /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "satellitedb.h"                                /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "internalreceiverinterface.h"                  /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "externalreceiverinterface.h"                  /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "os.h"                                         /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "util.h"                                       /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "instrumentation.h"                            /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "../../layer2transport/rtcm/rtcmframe.h"
#include "rtcmdatatypes.h"

#include "rtcmmessages.h"
#include "encodertcm.h"
#include "commprotocolrtcm.h"
#include "nt1065.h"                                     /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#include "../../../navigation/kalman.h"                 /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "navigation.h"                                 /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

/* ******************************************************************** */

/* ******************************************************************** */
/*                         GENERATE CONTENT                             */
/* ******************************************************************** */

void commProtocolRtcmGenerateMessage1005DataFields(rtcmControl1005Type *rtcm)
{

    rtcm->stationId = 2003;            /* 12bits with Ref. Station ID. u-blox use 0x000 as id. */
    rtcm->messageLength = 19;          /* En bytes, sin encabezado ni pie. */
    rtcm->position.px = 2779333.55;    /* SENyT station parameters - antenna position (ECEF) in x coordinate [m]. */
    rtcm->position.py = -4437943.40;   /* SENyT station parameters - antenna position (ECEF) in y coordinate [m]. */
    rtcm->position.pz = -3629393.44;   /* SENyT station parameters - antenna position (ECEF) in z coordinate [m]. */
    
    return;
}

int8_t commProtocolRtcmGenerateMessage1074DataFields(rtcmControl1074Type *rtcm, globalRangingInfoSetType *observableSet)
{
    uint8_t i,unhealthySats = 0;
    rtcmInternalRangingDataStruct navGpsMeas;
    double gpsReceiverTime, freq, lambda;
    int32_t receiverGpsWeek;

    navGetGpsReceiverTime(&gpsReceiverTime,&receiverGpsWeek,observableSet);   /* Devuelve en el tiempo (resolucion de ps) desde el inicio de la semana GPS. Rutina de navegacion.*/

    rtcm->stationId = 2003;
    rtcm->syncFlag = false; /* 1 indicates that more MSMs follow for given physical time and reference station ID */
    
    /* JFL:
     * Se redondea el tiempo del receptor para obtener un tiempo sincronizado con los pps del tiempo GPS 
     * Esto tiene sentido siempre y cuando haya sincronizacion con los pps. A esta seccion se ingresa solo cuando ocurre esto. 
     * */

    rtcm->numObservables = (uint8_t) observableSet->itemCount;
    rtcm->numSatellites = 0;
    rtcmGpsCalculateSignalTransmissionTimes(observableSet,&navGpsMeas);
    for(i=0;i<rtcm->numObservables;i++){
        
        if(svdbIsSatelliteHealthy(observableSet->ranges[i].svid) == 0){

            /* Se agrega el identificador de svid a cada medicion. */
            rtcm->observablesSet[i-unhealthySats].svid = observableSet->ranges[i].svid;

            /* Se agregan los indicadores de codigo respectivos. */
            if((observableSet->ranges[i].signalId > signal_gps_legacy_first) && (observableSet->ranges[i].signalId < signal_gps_legacy_last)){
                rtcm->observablesSet[i-unhealthySats].codeIndicator = CODE_L1C;
            }
            else if((observableSet->ranges[i].signalId > signal_gps_l2c_first) && (observableSet->ranges[i].signalId < signal_gps_l2c_last)){
                rtcm->observablesSet[i-unhealthySats].codeIndicator = CODE_L2L;
            }
            /* Se setean los bits correspondientes al indicador LLI segun RINEX*/
            rtcm->observablesSet[i-unhealthySats].lossOfLockIndicator = (0<<LLI_HALF_AMB) + (0<<LLI_CYCLE_SLIP);

            rtcm->observablesSet[i-unhealthySats].psrngObsData = (gpsReceiverTime - navGpsMeas.transmitTime[i])*NLC - (gpsReceiverTime - ROUND(gpsReceiverTime))*NLC;              /* Se hacen correcciones por sesgo de reloj. */
            rtcm->observablesSet[i-unhealthySats].phrngObsData = observableSet->ranges[i].accumulatedDeltaCarrierQ20*P2_20 - (gpsReceiverTime - ROUND(gpsReceiverTime))*freq ;     /* Se hacen correcciones por sesgo de reloj. */

            freq = code2freq(SYS_GPS,rtcm->observablesSet[i].codeIndicator,0);
            if (freq ==0.0){    /* Se revisa que la funcion que obtiene la frecuencia de la señal no haya devuelto 0.0*/
                lambda = 0;
            }
            else{
                lambda = CLIGHT/freq;
            }
            
            rtcm->observablesSet[i-unhealthySats].carrierDoppler = navGpsMeas.carrierDeviation[i];
            /* Se carga la CN0 */
            rtcm->observablesSet[i-unhealthySats].logScaleCN0 = (uint8_t) utilCNoLinearToLogQ2((uint32_t) observableSet->ranges[i].linearScaleCNo);
        }
        else{
            unhealthySats++;
        }
    }
    rtcm->numObservables -= unhealthySats;  /* Si hay mediciones de satelites unhealthy, no se guardan.*/

    rtcm->numSignals = RTCM_NUMBER_OF_SIGNALS;  /* Se setea la cantidad de señales con las que se trabaja. */
    rtcm->obsTime.receiverTime = (uint32_t) (ROUND(gpsReceiverTime) * (1e3));    /* Tiempo transcurrido desde el inicio de la semana GPS (en ms). Redondeado a segundo GPS.*/
    return 0;
}

/* ******************************************************************** */
/*                               PACKAGE                                */
/* ******************************************************************** */

#ifdef RTCM_FF /* RTCM from file (se carga el payload del mensaje directamente como constante) para testeo de capa de transporte.  */
void commProtocolRtcmPackageConstantMessage(commProtocolRtcmProtocolData *protocolInternalData,commProtoRtcmMessageIdType type)
{
    commTransportRtcmContainerType *transportContainer;
    commTransportRtcmControlBlockType *transport;
    commProtoRtcmMessage *messagePtr;
    uint8_t buffer_1005[] = {0x3E,0xD7,0xD3,0x02,0x02,0x98,0x0E,0xDE,0xEF,0x34,0xB4,0xBD,0x62,0xAC,0x09,0x41,0x98,0x6F,0x33};
    uint16_t buffer_1005_len = 19;
    uint8_t buffer_1074[] = {0x43,0x20,0x00,0x77,0x87,0x1E,0xFC,0x00,0x00,0x22,0x1F,0x50,0x88,0x00,0x00,0x00,0x00,0x20,0x00,0x80,0x00,0x5F,0xDF,0xDC,0xA6,0x9A,0xA2,0x92,0x96,0xA6,0x92,0x94,0x8C,0x8C,0x96,0x4D,0x8C,0x33,0x7F,0xF0,0x10,0x74,0x86,0x90,0x07,0x3D,0x35,0x20,0xE6,0x1D,0x6E,0x4B,0xFB,0x68,0xC6,0x5C,0x87,0x62,0x2D,0x4E,0x4B,0x8B,0xBD,0xA8,0xD5,0x51,0x25,0x9C,0x8C,0x76,0x8A,0x02,0xF3,0xFA,0x72,0x39,0xE8,0x36,0x30,0x1A,0xFE,0x77,0x1E,0x25,0xA4,0x7D,0x09,0xCD,0xEE,0xF6,0xC0,0x42,0x04,0x60,0xED,0x16,0x05,0x00,0x46,0x12,0x95,0xEF,0xE0,0xFC,0xA0,0xB6,0xF4,0x01,0x73,0xEF,0xF2,0xBC,0xFF,0xB2,0xEA,0x80,0x3A,0x00,0xFF,0x7B,0x30,0x0E,0xFE,0x18,0x1D,0x61,0xC0,0x47,0x37,0x7C,0xE9,0x9D,0xDF,0xBF,0xFF,0x1F,0xDF,0xFF,0xFF,0xFF,0xFE,0x00,0x00,0x47,0x45,0x74,0x48,0xDB,0xD6,0xCD,0x05,0xD4,0xDB,0x3D,0xB5,0x4F,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA}; /* Sin header ni footer. Listo para que lo procese la capa de transporte. */
    uint16_t buffer_1074_len = 181;
    uint8_t msg_content[COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE];
    uint16_t buffer_len;
    
     
    commTransportRtcmContainerSourceIdType id;

    transport = protocolInternalData->transportLayerHandle;

    switch (type)
    {
        case commProtoRtcmMessageId1005:
            id = txContainerSmall;
            buffer_len = buffer_1005_len;
            memcpy(msg_content, buffer_1005,buffer_len);
            break;

        case commProtoRtcmMessageId1074:
        default:
            id = txContainerHuge;
            buffer_len = buffer_1074_len;
            memcpy(msg_content, buffer_1074,buffer_len);
            break;
    }
    /* Pido un bloque del tamaño del mensaje que se transmite */
    transportContainer = transport->transportAllocMessageContainer(
            transport->genericInternalTransportDataPtr,
            buffer_len*sizeof(uint8_t),
            id);

    if (transportContainer != NULL)
    {
        messagePtr = (commProtoRtcmMessage *)transportContainer->userData.messageBuffer;

        /* Completo el campo con el contenido del mensaje. */
        memcpy(messagePtr,msg_content,buffer_len);

        /* Calculo la cantidad de bytes en el mensaje. */
        transportContainer->userData.messageLength = buffer_len*sizeof(uint8_t);

        transportContainer->userData.srcAddress = -1;
        transportContainer->userData.dstAddress = -1;

        /* Encolo el mensaje para transmisión */
        transport->transportAddMessageContainerToTxQueue(
                transport->genericInternalTransportDataPtr,
                transportContainer);


    }
    
}
#endif  /* RTCM from file */

void commProtocolRtcmPackageMessage1005(commProtocolRtcmProtocolData *protocolInternalData)
{

    commTransportRtcmContainerType *transportContainer;
    commTransportRtcmControlBlockType *transport;
    commProtoRtcmMessage *messagePtr;  
    static rtcmControl1005Type rtcm;
    uint16_t i=0;

    transport = protocolInternalData->transportLayerHandle;

    commProtocolRtcmGenerateMessage1005DataFields(&rtcm);
    /* JFL:
     * Se setean los bits (dados por cuarto argumento) en el contenido del mensaje 
     * desde i hasta i+tercer_arg. 
     * Y luego se incrementa i en la cantidad de bits que se setearon.
     *  */
    setbitu(rtcm.messageContent,i,12,1005       ); i+=12; /* message no */
    setbitu(rtcm.messageContent,i,12,rtcm.stationId ); i+=12; /* ref station id */
    setbitu(rtcm.messageContent,i, 6,0          ); i+= 6; /* itrf realization year */
    setbitu(rtcm.messageContent,i, 1,1          ); i+= 1; /* gps indicator */
    setbitu(rtcm.messageContent,i, 1,0          ); i+= 1; /* glonass indicator */
    setbitu(rtcm.messageContent,i, 1,0          ); i+= 1; /* galileo indicator */
    setbitu(rtcm.messageContent,i, 1,0          ); i+= 1; /* ref station indicator */
    set38bits(rtcm.messageContent,i,rtcm.position.px/0.0001 ); i+=38; /* antenna ref point ecef-x */
    setbitu(rtcm.messageContent,i, 1,1          ); i+= 1; /* oscillator indicator */
    setbitu(rtcm.messageContent,i, 1,0          ); i+= 1; /* reserved */
    set38bits(rtcm.messageContent,i,rtcm.position.py/0.0001 ); i+=38; /* antenna ref point ecef-y */
    setbitu(rtcm.messageContent,i, 2,2          ); i+= 2; /* quarter cycle indicator */
    set38bits(rtcm.messageContent,i,rtcm.position.pz/0.0001 ); i+=38; /* antenna ref point ecef-z */

    /* Pido un bloque del tamaño del mensaje que se transmite */
    transportContainer = transport->transportAllocMessageContainer(
            transport->genericInternalTransportDataPtr,
            rtcm.messageLength*sizeof(uint8_t),
            txContainerSmall);

    if (transportContainer != NULL)
    {
        messagePtr = (commProtoRtcmMessage *)transportContainer->userData.messageBuffer;

        /* Completo el campo con el contenido del mensaje. */
        memcpy(messagePtr,&rtcm.messageContent,rtcm.messageLength);

        /* Calculo la cantidad de bytes en el mensaje. */
        transportContainer->userData.messageLength = rtcm.messageLength;

        transportContainer->userData.srcAddress = -1;
        transportContainer->userData.dstAddress = -1;

        /* Encolo el mensaje para transmisión */
        transport->transportAddMessageContainerToTxQueue(
                transport->genericInternalTransportDataPtr,
                transportContainer);

    }
    
}

void commProtocolRtcmPackageMessage1074(commProtocolRtcmProtocolData *protocolInternalData, globalRangingInfoSetType *observableSet, booleanData *flagFirstRtcm)
{
    commTransportRtcmContainerType *transportContainer;
    commTransportRtcmControlBlockType *transport;
    commProtoRtcmMessage *messagePtr;
    static rtcmControl1074Type rtcm;

    double rrng[64],psrng[64],phrng[64];    /* Aca se podria optimizar y bajar el largo del arreglo. */
    uint8_t cnr[64];  
    uint8_t half[64];
    uint16_t ncell;
    uint16_t i;
    uint16_t lock[CPARAM_TRACKING_CHANNELS*NUM_SIGNALS_Q];

    transport = protocolInternalData->transportLayerHandle;
    commProtocolRtcmGenerateMessage1074DataFields(&rtcm,observableSet);

    /* encode msm header */
    if (!(i = encode_msm_head(4,&rtcm,SYS_GPS,rtcm.syncFlag,&rtcm.numSatellites,&ncell,rrng,psrng,
                            phrng,lock,half,cnr,flagFirstRtcm))) {  /* Cuando la funcion devuelve 0 hay error. */
        return;
    }

    /* encode msm satellite data */
    i = encode_msm_int_rrng(&rtcm,i,rrng ,rtcm.numSatellites ); /* rough range integer ms */
    i = encode_msm_mod_rrng(&rtcm,i,rrng ,rtcm.numSatellites ); /* rough range modulo 1 ms */

    /* encode msm signal data */
    i = encode_msm_psrng   (&rtcm,i,psrng,ncell); /* fine pseudorange */
    i = encode_msm_phrng   (&rtcm,i,phrng,ncell); /* fine phase-range */
    i = encode_msm_lock    (&rtcm,i,lock ,ncell); /* lock-time indicator */
    i = encode_msm_half_amb(&rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i = encode_msm_cnr     (&rtcm,i,cnr  ,ncell); /* signal cnr */
    rtcm.numBits = i;
    rtcm.messageLength = ((rtcm.numBits+8-1)/8)*8/8;   /* Se redondea numBits al proximo multiplo de 8 y luego se obtiene cantidad de bytes. */

    /* Pido un bloque del tamaño del mensaje que se transmite */
    transportContainer = transport->transportAllocMessageContainer(
            transport->genericInternalTransportDataPtr,
            rtcm.messageLength*sizeof(uint8_t),
            txContainerHuge);

    if (transportContainer != NULL)
    {
        messagePtr = (commProtoRtcmMessage *)transportContainer->userData.messageBuffer;

        /* Completo el campo con el contenido del mensaje. */
        memcpy(messagePtr,rtcm.messageContent,rtcm.messageLength);

        /* Calculo la cantidad de bytes en el mensaje. */
        transportContainer->userData.messageLength = rtcm.messageLength;

        transportContainer->userData.srcAddress = -1;
        transportContainer->userData.dstAddress = -1;

        /* Encolo el mensaje para transmisión */
        transport->transportAddMessageContainerToTxQueue(
                transport->genericInternalTransportDataPtr,
                transportContainer);

    }
    return;
}

int32_t commProtocolRtcmProcessReceiverEvents(commProtocolRtcmProtocolData *protocolInternalData)
{
	receiverEventType newEvent;
    int32_t theresMoreWorkToDo;
    static globalRangingInfoSetType observableSet;
    static booleanData flagFirstRtcm[RTCM_NUMBER_OF_SIGNALS*32];    /* [0:1] Es L1 y L2 de GPS1 y asi... */
    uint8_t i;
    booleanData flagNavData;

    theresMoreWorkToDo = 0;

    if (extIntPendingEvents(protocolInternalData->eventClientData) == trueValue)
    {
        theresMoreWorkToDo = 1;

        /* Determino qué tipo procesamiento hay que realizar */
        extIntNonBlockingWait(protocolInternalData->eventClientData, &newEvent);

        flagNavData = false;

        switch (newEvent) {
            case eventRangingDataAvailable:
                /* Antes de solicitar un bloque chequeo que tengo información para enviar. */
                if (navGetGnssObservables(&observableSet) < 0)
                {
                    flagNavData = true;
                }

                if(observableSet.ppsSynchronized && !flagNavData){
                    commProtocolRtcmPackageMessage1005(protocolInternalData);
                    commProtocolRtcmPackageMessage1074(protocolInternalData,&observableSet,flagFirstRtcm);
                    commDebugConsole("Se envio RTCM");
                }
                else{
                    for (i=0;i<RTCM_NUMBER_OF_SIGNALS*32;i++)
                    {
                        flagFirstRtcm[i] = false;
                    }
                }
                break;
            default:
                break;

        }
    }

    return theresMoreWorkToDo;
}


osTaskReturnType commProtocolRtcmProcessingTask(osTaskArgumentType genericInternalDataPtr)
{
    commProtocolRtcmProtocolData *protocolInternalData = (commProtocolRtcmProtocolData *)genericInternalDataPtr;
    int32_t didProcessReceiverEvents;

    while (1)
    {
        /* Duermo la tarea esperando la activación de la misma por alguna tarea externa.
         * Esto espera la activación de la tarea realizada usando osSendTaskActivationSignal() ,
         * pero también funciona utilizando el sistema de eventos del receptor ya que junto con
         * el evento se envía una señal de activación. */
        osWaitForExternalTaskActivationSignal();

        /* Itero entre todas las actividades posibles hasta que ninguna de ellas realice trabajo útil.
         *
         * Cada una de las rutinas que ataca cada actividad particular procesa un único elemento por vez
         * (un contenedor, en mensaje, una iteración de la máquina de estados, etc.) y avisa si efectivamente
         * procesó algún elemento o no. El bucle en este nivel se encarga de turnar las actividades para que
         * ninguna postergue demasiado las demás. */
        do {
            /* Eventos RTCM del receptor */
            didProcessReceiverEvents = commProtocolRtcmProcessReceiverEvents(protocolInternalData);

            /* No se utiliza pues no recibimos mensajes RTCM por el momento */
            //didProcessInputContainers = commProtocolRtcmProcessInputContainers(protocolInternalData);

            /* Esto es de tracking? */
            //didProcessTrackingLogSamples = commProtocolRtcmProcessTrackingLogSamples(protocolInternalData);

        } while ((didProcessReceiverEvents != 0) /*|| (didProcessInputContainers != 0) || (didProcessTrackingLogSamples != 0)*/);
    }
}

commProtocolControlBlockType *commProtocolRtcmCreateControlBlock(commTransportRtcmControlBlockType *transport)
{
    commProtocolControlBlockType *newProtocolControlBlockPtr;
    commProtocolRtcmProtocolData *newProtocolInternalDataPtr;

    /* Creo el bloque de control de la nueva instancia del protocolo */
    newProtocolControlBlockPtr = osMemAlloc(sizeof(commTransportRtcmControlBlockType));
    systemAssert(newProtocolControlBlockPtr != NULL, "Not enough memory!");

    /* Creo el bloque de información del protocolo */
    newProtocolInternalDataPtr = osMemAlloc(sizeof(commProtocolRtcmProtocolData));
    systemAssert(newProtocolInternalDataPtr != NULL, "Not enough memory!");

    /* Inicializo las estructuras en cero */
    utilZeroMem(newProtocolControlBlockPtr, sizeof(commTransportRtcmControlBlockType));
    utilZeroMem(newProtocolInternalDataPtr, sizeof(commProtocolRtcmProtocolData));

    /* ****************************************** */

    osTaskCreateWithArgument("PRTK",
            PRIO_COMM_LAYER3_PROC_TASK,
            OS_MINIMUM_STACK_SIZE,
            OS_PREEMPT|OS_TIMESLICE,
            OS_FLOATING_POINT,
            commProtocolRtcmProcessingTask,
            &newProtocolInternalDataPtr->protocolProcessingTaskId,
            (osTaskArgumentType)newProtocolInternalDataPtr);

    /* Me guardo el puntero al tranporte para uso posterior */
    newProtocolInternalDataPtr->transportLayerHandle = transport;

    /* Registro la tarea de procesamiento en el layer2 para recibir notificaciones cuando hayan nuevos
     * contenedores para procesar en la cola de recepción. */
    transport->transportRegisterProtocolProcessingTask(
            transport->genericInternalTransportDataPtr,
            newProtocolInternalDataPtr->protocolProcessingTaskId);

    /* ****************************************** */

    /* Registro la tarea de procesamiento en el manejador de eventos de la interfaz
     * central del receptor. */
    newProtocolInternalDataPtr->eventClientData = extIntRegisterClient(newProtocolInternalDataPtr->protocolProcessingTaskId);

    /* ****************************************** */

    newProtocolControlBlockPtr->genericInternalProtocolDataPtr = (void *)newProtocolInternalDataPtr;

    newProtocolControlBlockPtr->protocolDescription = "MGGR RTCM Protocol Layer";

    return newProtocolControlBlockPtr;
}