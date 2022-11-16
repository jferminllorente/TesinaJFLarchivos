#include "configuration/configuration.h"        /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "hardware/hardware.h"                  /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#include "ifoperatingsystem/os.h"               /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "commtransportrtcm.h"
#include "system.h"                             /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "util.h"                               /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#include "commdevicelayer.h"                    /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#include "rtcmframe.h"

/* ******************************************************************** */
/* ******************************************************************** */

typedef struct {

    commDeviceControlBlockType *deviceLayerHandle; /**< Dispositivo subyacente a través del cual se hacen entradas y salidas */

    /* Espacio de memoria asignados a contenedores Tx de tamaño regular */
    uint8_t *smallTxContainerPartitionsStorageSpace;   /**< Espacio de almacenamiento de bloques de control de Tx. */
    uint8_t *smallTxPayloadPartitionsStorageSpace; /**< Espacio de almacenamiento de almacenamiento de los mensajes de Tx. */

    /* Espacio de memoria asignados a contenedores Tx de gran tamaño */
    uint8_t *hugeTxContainerPartitionsStorageSpace;   /**< Espacio de almacenamiento de bloques de control de Tx. */
    uint8_t *hugeTxPayloadPartitionsStorageSpace; /**< Espacio de almacenamiento de almacenamiento de los mensajes de Tx. */

    /* Espacio de memoria asignados a contenedores Tx de tamaño completo */
    uint8_t *fullTxContainerPartitionsStorageSpace;   /**< Espacio de almacenamiento de bloques de control de Tx. */
    uint8_t *fullTxPayloadPartitionsStorageSpace; /**< Espacio de almacenamiento de almacenamiento de los mensajes de Tx. */

    /* Espacio de memoria asignados a Rx */
    uint8_t *rxContainerPartitionsStorageSpace;	/**< Espacio de almacenamiento de bloques de control de Rx. */
    uint8_t *rxPayloadPartitionsStorageSpace;	/**< Espacio de almacenamiento de almacenamiento de los mensajes de Rx. */

    /* Identificadores de partición de Tx */
    osPartitionIdType smallTxMessageContainerPartitionId; /**< Identificador de la partición de bloques de control de Tx. */
    osPartitionIdType smallTxMessagePayloadPartitionId;   /**< Identificador de la partición de almacenamiento de los mensajes de Tx. */

    /* Identificadores de partición de Tx de bloques grandes*/
    osPartitionIdType hugeTxMessageContainerPartitionId; /**< Identificador de la partición de bloques de control de Tx. */
    osPartitionIdType hugeTxMessagePayloadPartitionId;   /**< Identificador de la partición de almacenamiento de los mensajes de Tx. */

        /* Identificadores de partición de Tx de bloques completo*/
    osPartitionIdType fullTxMessageContainerPartitionId; /**< Identificador de la partición de bloques de control de Tx. */
    osPartitionIdType fullTxMessagePayloadPartitionId;   /**< Identificador de la partición de almacenamiento de los mensajes de Tx. */

    /* Identificadores de partición de Rx */
    osPartitionIdType rxMessageContainerPartitionId; /**< Identificador de la partición de bloques de control de Rx. */
    osPartitionIdType rxMessagePayloadPartitionId;   /**< Identificador de la partición de almacenamiento de los mensajes de Rx. */

    /* *************************************** */

    /* Mutex de acceso a los miembros que pueden ser modificados
     * concurrentemente */
    osMutexIdType accessMutex;

    /* Estadísticas de los bloques de memoria de Tx de tamaño regular */
    uint32_t smallTxContainersInUse; /**<  Cantidad de bloques de control de Tx que están en uso. */
    uint32_t smallTxContainersInUseMax;	/**<  Cantidad máxima de bloques de control de Tx que estuvieron en uso desde que se inicializó el protocolo. */
    uint32_t smallTxContainersCriticalThreshold; /**< Cantidad mínima de contenedores disponibles para que se puedan asignar (alloc) contenedores para paquetes no críticos. */

    /* Estadísticas de los bloques de memoria de Tx de tamaño grande */
    uint32_t hugeTxContainersInUse;	 /**<  Cantidad de bloques de control de Tx que están en uso. */
    uint32_t hugeTxContainersInUseMax;	/**<  Cantidad máxima de bloques de control de Tx que estuvieron en uso desde que se inicializó el protocolo. */

    /* Estadísticas de los bloques de memoria de Tx de tamaño completo */
    uint32_t fullTxContainersInUse;	 /**<  Cantidad de bloques de control de Tx que están en uso. */
    uint32_t fullTxContainersInUseMax;	/**<  Cantidad máxima de bloques de control de Tx que estuvieron en uso desde que se inicializó el protocolo. */

    /* Estadísticas de los bloques de memoria de Rx */
    uint32_t rxContainersInUse;		/**<  Cantidad de bloques de control de Rx que están en uso. */
    uint32_t rxContainersInUseMax;	/**<  Cantidad máxima de bloques de control de Rx que estuvieron en uso desde que se inicializó el protocolo. */

    /* Parámetros de configuración del protocolo */
    uint32_t smallTxTotalContainers; /**< Largo máximo de la cola de Tx. */
    uint32_t hugeTxTotalContainers;	 /**< Largo máximo de la cola de Tx. */
    uint32_t fullTxTotalContainers;  /**< Largo máximo de la cola de Tx. */
    uint32_t rxTotalContainers;      /**< Largo máximo de la cola de Rx. */

    /* *************************************** */

    /* Colas de transmisión y recepción */
    osMessageQueueIdType txQueueId; /**< Cola de transmisión. */
    osMessageQueueIdType rxQueueId; /**< Cola de recepción. */

    /* Tarea de recepción */
    osTaskIdType txTaskId;	/**< Identificador de la tarea de transmisión. */
    osTaskIdType rxTaskId;	/**< Identificador de la tarea de recepción. */

    /* Identificador de la tarea de procesamiento delprotocolo. */
    osTaskIdType protocolProcessingTaskId; /**< Identificador de la tarea de procesamiento de protocolo, que está en layer3protocol. */
    booleanData  protocolProcessingTaskRegistered; /**< Verdadero si hay una tarea registrada, falso si no hay ninguna. */

} commTransportRtcmProtocolData;


/**
 * \brief Calculo del los bits de paridad segun CRC24Q.
 *
 * @param buff Buffer de datos del mensaje al cual se le calcula la paridad.
 * @param len Largo de los datos a los cuales se le calcula la paridad. Incluye contenido del mensaje y 3 bytes de encabezado.
 * @return 3 bytes de paridad crc24q.
 */
uint32_t rtk_crc24q(const uint8_t *buff, int len)
{
    uint32_t crc=0;
    int i;
    
    for (i=0;i<len;i++){
        crc = ((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    }
    return crc;
}

void commTransportRtcmReleaseMessageContainer(void *genericInternalTransportDataPtr, commTransportRtcmContainerType *container)
{
    osPartitionIdType containerPartitionId = 0;  /* Inicializado para evitar un warning. */
    osPartitionIdType payloadPartitionId = 0; /* Inicializado para evitar un warning. */

    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;

    if (container != NULL)
    {
        osMutexGetIntoCriticalSection(transportInternalData->accessMutex);

        /* Determino a qué pool tengo que liberar la partición. */
        switch (container->systemData.sourceId) {

            case txContainerSmall:
                containerPartitionId = transportInternalData->smallTxMessageContainerPartitionId;
                payloadPartitionId   = transportInternalData->smallTxMessagePayloadPartitionId;

                /* Actualizo los contadores de contenedores de TX pequeños */
                transportInternalData->smallTxContainersInUse--;

                break;

            case txContainerHuge:
                containerPartitionId = transportInternalData->hugeTxMessageContainerPartitionId;
                payloadPartitionId   = transportInternalData->hugeTxMessagePayloadPartitionId;

                /* Actualizo los contadores de contenedores de TX grandes */
                transportInternalData->hugeTxContainersInUse--;

                break;

            case txContainerFull :
                containerPartitionId = transportInternalData->fullTxMessageContainerPartitionId;
                payloadPartitionId   = transportInternalData->fullTxMessagePayloadPartitionId;

                /* Actualizo los contadores de contenedores de TX completos */
                transportInternalData->fullTxContainersInUse--;
                break;
        }

        /* Libero la memoria de paquete */
        if (container->systemData.containerBuffer != NULL)
        {
            osMemoryPartitionReleaseBuffer(payloadPartitionId, (void*)container->systemData.containerBuffer);
        }

        /* Libero la memoria del contenedor */
        osMemoryPartitionReleaseBuffer(containerPartitionId, (void*)container);

        osMutexGetOutOfCriticalSection(transportInternalData->accessMutex);
    }
}

commTransportRtcmContainerType *commTransportRtcmAllocMessageContainer(void *genericInternalTransportDataPtr, uint32_t buffersize, commTransportRtcmContainerSourceIdType sourceId)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;
    commTransportRtcmContainerType *newContainer;
    void *containerBuffer;
    uint16_t packetSize;
    uint16_t containerBufferSize = 0; /* Inicializado para evitar un warning. */

    osPartitionIdType containerPartitionId = 0; /* Inicializado para evitar un warning. */
    osPartitionIdType payloadPartitionId = 0; /* Inicializado para evitar un warning. */

    /* Calculo el tamaño del buffer de memoria necesario */
    packetSize = COMM_TRANSPORT_RTCM_HEADER_LENGTH + buffersize + COMM_TRANSPORT_RTCM_FOOTER_LENGTH;

    /* Determino el pool de origen de las particiones. */
    switch(sourceId) {

        case txContainerSmall :
            containerPartitionId = transportInternalData->smallTxMessageContainerPartitionId;
            payloadPartitionId   = transportInternalData->smallTxMessagePayloadPartitionId;

            containerBufferSize = COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE;
            break;

        case txContainerHuge :
            containerPartitionId = transportInternalData->hugeTxMessageContainerPartitionId;
            payloadPartitionId   = transportInternalData->hugeTxMessagePayloadPartitionId;

            containerBufferSize = COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE;
            break;

        case txContainerFull :
            containerPartitionId = transportInternalData->fullTxMessageContainerPartitionId;
            payloadPartitionId   = transportInternalData->fullTxMessagePayloadPartitionId;

            containerBufferSize = COMM_TRANSPORT_RTCM_FULL_CONTAINER_SIZE;
            break;

        default :
            systemInconditionalFatalError("Undefined container type!");
            break;
    }

    if (packetSize > containerBufferSize)
    {
        return NULL;
    }

    if (buffersize == 0)
    {
        return NULL;
    }

    osMutexGetIntoCriticalSection(transportInternalData->accessMutex);
    
    /* JFL:
     * No es necesario revisar si hay contenedores disponibles para aplicaciones no criticas
     * pues todos los mensajes RTCM son no criticos, no existe tal diferenciacion. */

    newContainer = (commTransportRtcmContainerType *)osMemoryPartitionGetBuffer(containerPartitionId);

    if (newContainer != NULL)
    {
        /* Inicializo el buffer */
        utilZeroMem(newContainer, sizeof(commTransportRtcmContainerType));

        /* Reservo la memoria para el payload */
        containerBuffer = (uint8_t *)osMemoryPartitionGetBuffer(payloadPartitionId);

        if (containerBuffer == NULL)
        {
            /* no se pudo reservar el segundo bloque de memoria, así que libero el container que reservé. */
            osMemoryPartitionReleaseBuffer(containerPartitionId, newContainer);

            osMutexGetOutOfCriticalSection(transportInternalData->accessMutex);
            return NULL;
        }

        /* Hago la contabilidad de contenedores restantes del bloque de contenedores de tamaño normal */
        switch(sourceId) {
            case txContainerSmall :
                /* Actualizo los contadores de contenedores de TX pequeños */
                transportInternalData->smallTxContainersInUse++;

                if (transportInternalData->smallTxContainersInUseMax < transportInternalData->smallTxContainersInUse)
                {
                    transportInternalData->smallTxContainersInUseMax = transportInternalData->smallTxContainersInUse;
                }
                break;
            case txContainerHuge :
                /* Actualizo los contadores de contenedores de TX grandes */
                transportInternalData->hugeTxContainersInUse++;

                if (transportInternalData->hugeTxContainersInUseMax < transportInternalData->hugeTxContainersInUse)
                {
                    transportInternalData->hugeTxContainersInUseMax = transportInternalData->hugeTxContainersInUse;
                }
                break;
            case txContainerFull :
                /* Actualizo los contadores de contenedores de TX completos */
                transportInternalData->fullTxContainersInUse++;

                if (transportInternalData->fullTxContainersInUseMax < transportInternalData->fullTxContainersInUse)
                {
                    transportInternalData->fullTxContainersInUseMax = transportInternalData->fullTxContainersInUse;
                }
                break;
        }

        systemAssert(transportInternalData->smallTxContainersInUse <= transportInternalData->smallTxTotalContainers, "Too many small tx containers assigned!");
        systemAssert(transportInternalData->hugeTxContainersInUse <= transportInternalData->hugeTxTotalContainers, "Too many huge tx containers assigned!");
        systemAssert(transportInternalData->fullTxContainersInUse <= transportInternalData->fullTxTotalContainers, "Too many full tx containers assigned!");

        /* Inicializo los campos de sistema */
        newContainer->systemData.containerBuffer = containerBuffer;
        newContainer->systemData.containerBufferSize = containerBufferSize;
        newContainer->systemData.messageBufferSize = containerBufferSize - COMM_TRANSPORT_RTCM_HEADER_LENGTH - COMM_TRANSPORT_RTCM_FOOTER_LENGTH;
        newContainer->systemData.sourceId = sourceId;

        /* Inicializo los campos de usuario */
        newContainer->userData.messageLength = 0;
        newContainer->userData.dstAddress = -1;
        newContainer->userData.srcAddress = -1;

        /* El buffer de usuario está ubicado dentro del buffer reservado, dejando espacio para colocar luego el encabezado de los mensajes */
        newContainer->userData.messageBuffer = newContainer->systemData.containerBuffer + COMM_TRANSPORT_RTCM_HEADER_LENGTH;

        osMutexGetOutOfCriticalSection(transportInternalData->accessMutex);
        return newContainer;
    }

    osMutexGetOutOfCriticalSection(transportInternalData->accessMutex);
    return NULL;
}

void commTransportRtcmAddMessageContainerToTxQueue(void *genericInternalTransportDataPtr, commTransportRtcmContainerType *containerPtr)
{
    commTransportRtcmProtocolData *protoData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;
    osReturnValueType retValue;

    if (containerPtr == NULL)
    {
        return;
    }

    retValue = osMessageQueueSend(protoData->txQueueId, (void*)&containerPtr, sizeof(commTransportRtcmContainerType *));

    if (retValue != OS_SUCCESS)
    {
        /* si por algún motivo falla el envío, elimino el frame para liberar la memoria */
        commTransportRtcmReleaseMessageContainer(genericInternalTransportDataPtr, containerPtr);
    }
}

/**
 * \brief Devuelve verdadero si hay contenedores pendientes de ser procesados en la cola de recepción.
 *
 * Esta rutina no bloquea al llamador.
 *
 * @param protocolData Estructura con los datos correspondientes a este protocolo.
 * @return Verdadero si hay contenedores pendientes. Falso si no hay contenedores pendientes.
 */
booleanData commTransportRtcmRxQueueNotEmpty(void *genericInternalTransportDataPtr)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;

    return osMessageQueuePoolReady(transportInternalData->rxQueueId);
}

/**
 * \brief Devuelve un mensaje pendiente en la cola de recepción. No bloqueante.
 *
 * Si hay mensajes pendientes devuelve 0.
 *
 * Si no hay mensajes pendientes, devuelve -1.
 *
 * @param protocolData Estructura con los datos correspondientes a este protocolo.
 * @param packetContainer Puntero
 * @return
 */
int32_t commTransportRtcmGetContainerFromRxQueueNonBlocking(void *genericInternalTransportDataPtr, commTransportRtcmContainerType **container)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;
    uint32_t containerSize;
    int32_t returnCode;

    returnCode = 0;

    if (osMessageQueueNonBlockingReceive(transportInternalData->rxQueueId, (void*)container, &containerSize) != OS_SUCCESS)
    {
        returnCode = -1;
    }

    return returnCode;
}


void commTransportRtcmRegisterProtocolProcessingTask(void *genericInternalTransportDataPtr, osTaskIdType protocolProcessingTaskId)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;

    transportInternalData->protocolProcessingTaskId = protocolProcessingTaskId;
    transportInternalData->protocolProcessingTaskRegistered = trueValue;

}

void commTransportRtcmInternalFrameAndSend(commTransportRtcmProtocolData *genericInternalTransportDataPtr, commTransportRtcmContainerType *packetBuffer)
{
    uint8_t *wordPtr;
    uint32_t auxVar;
    uint32_t header;
    uint32_t parity;
    commDeviceControlBlockType *device;

    device = genericInternalTransportDataPtr->deviceLayerHandle;

    if (packetBuffer == NULL)
    {
        return; /* oops */
    }

    /* a few sanity checks first ... */
    if ((packetBuffer->userData.messageLength <= packetBuffer->systemData.messageBufferSize)	/* self explaining */
            && ( (packetBuffer->userData.messageLength > 0) || (packetBuffer->systemData.containerBuffer == NULL))) /* data solo es null si no hay carga */
    {
        /* Armo el header y el footer */
        wordPtr = packetBuffer->systemData.containerBuffer;

        /* Agrego el prefijo REVISAR MEMORIA PORQUE PREFIJO ES MAS PEQUEñO*/
        header = (COMM_TRANSPORT_RTCM_FRAME_PREFIX<<16) + (packetBuffer->userData.messageLength & 0x0003FF);
        header = header << 8;
        auxVar = utilLocal2BigEndianU32(header);
        utilMemCopy((void*)wordPtr, (void*)(&auxVar), COMM_TRANSPORT_RTCM_HEADER_LENGTH*sizeof(uint8_t));
        wordPtr += COMM_TRANSPORT_RTCM_HEADER_LENGTH; /**<  A esta altura ya esta cargado preambulo, ceros y largo del msj  */

        /* Salto por encima del payload */
        wordPtr = ((wordPtr) + packetBuffer->userData.messageLength);

        /* Calculo el checksum del cuerpo y lo agrego para armar el footer */
        parity = rtk_crc24q(
                (packetBuffer->systemData.containerBuffer), /* Aca es equivalente poner packetBuffer->systemData.containerBuffer */
                (int)(packetBuffer->userData.messageLength + COMM_TRANSPORT_RTCM_HEADER_LENGTH));

        parity = parity << 8;
        auxVar = utilLocal2BigEndianU32(parity);
        utilMemCopy((void*)wordPtr, (void*)&auxVar, COMM_TRANSPORT_RTCM_FOOTER_LENGTH*sizeof(uint8_t));
        wordPtr += COMM_TRANSPORT_RTCM_FOOTER_LENGTH;
        
        /* Done!!! Envío el mensaje por el device. */
        systemAssert(
                device->deviceStreamingWrite != NULL,
                "Inadequate device for this protocol!");

        device->deviceStreamingWrite(
                device->genericInternalDeviceDataPtr,
                packetBuffer->systemData.containerBuffer,
                packetBuffer->userData.messageLength + COMM_TRANSPORT_RTCM_HEADER_LENGTH + COMM_TRANSPORT_RTCM_FOOTER_LENGTH);
    }

    /* Libero el container */
    commTransportRtcmReleaseMessageContainer(genericInternalTransportDataPtr, packetBuffer);
}

osTaskReturnType commTransportRtcmTransmitterTask(osTaskArgumentType genericInternalTransportDataPtr)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;
    commTransportRtcmContainerType *container;
    osReturnValueType returnValue;
    uint32_t containerSize;

    while (1)
    {
        /* duermo esperando un mensaje de la cola */
        returnValue = osMessageQueueBlockingReceive(transportInternalData->txQueueId, (void*)&container, &containerSize);

        systemAssert(returnValue == OS_SUCCESS, "Transmitter Tx queue failure!");

        commTransportRtcmInternalFrameAndSend(transportInternalData, container);
    }
}


uint32_t commTransportRtcmRotateRightAndInsertByte(uint32_t originalWord , uint8_t newByte)
{
    return (((((uint32_t)originalWord) & 0x00ffffff) << 8) + (((uint32_t)newByte) & 0x000000ff));
}
#ifdef RTCM_RECEIVE /**< Por si en algun momento se agrega recepcion de RTCM. */
commTransportRtcmContainerType *commTransportRtcmSyncAndReceivePacket(commTransportRtcmProtocolData *transportInternalData)
{
    commDeviceControlBlockType *deviceLayerHandle;
    commTransportRtcmContainerType *rxContainerPtr;

    enum {st1RecvHeader, st2RecvLength, st3RecvMsgBody, st4RecvChecksum, st5RecvTail} machineState;
    int32_t machineSubstate;

    booleanData	newFrameReady;

    uint32_t networkOrderWord, hostOrderWord;
    uint32_t calculatedChecksum = 0;
    uint8_t	 receivedByte;
    int32_t maxProtocolMsgSize;

    /* Obtengo la información del dispositivo de hardware */
    deviceLayerHandle = transportInternalData->deviceLayerHandle;

    systemAssert(deviceLayerHandle != NULL, "No device to read from!");
    systemAssert(deviceLayerHandle->deviceStreamingRead != NULL, "Required reading mode not supported by device!");

    /* Calculo el tamaño máximo que puede tener que almacenar un contenendor */
    maxProtocolMsgSize = COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE - COMM_TRANSPORT_RTCM_HEADER_LENGTH - COMM_TRANSPORT_RTCM_FOOTER_LENGTH;

    /* Reservo un bloque del tamaño máximo */
    rxContainerPtr = commTransportRtcmAllocMessageContainer((void *)transportInternalData, maxProtocolMsgSize, rxContainer);

    if (rxContainerPtr == NULL)
    {
        return NULL;
    }

    /* Inicializo la máquina de estado que se encarga del sincronismo de paquete */
    networkOrderWord = 0;
    machineSubstate = 0;

    machineState = st1RecvHeader;
    newFrameReady = falseValue;

    while (newFrameReady == falseValue)
    {
        /* Notar que la capa de hardware no devuelve ningún tipo de código de retorno para indicar el éxito o error de la lectura! */
        deviceLayerHandle->deviceStreamingRead(deviceLayerHandle->genericInternalDeviceDataPtr, &receivedByte, 1);

        /* en receivedByte está el caracter */
        switch (machineState)
        {
            case st1RecvHeader:

                networkOrderWord = commTransportRtcmRotateRightAndInsertByte(networkOrderWord, receivedByte);
                hostOrderWord    = cpuBigEndianToLocalU32(networkOrderWord);

                if (hostOrderWord == COMM_TRANSPORT_RTCM_FRAME_PREFIX)
                {
                    machineState = st2RecvLength;
                    machineSubstate = 0;
                }
                break;

            case st2RecvLength:

                /* Recibo los cuatro bytes del campo longitud */
                networkOrderWord = commTransportRtcmRotateRightAndInsertByte(networkOrderWord, receivedByte);
                hostOrderWord    = cpuBigEndianToLocalU32(networkOrderWord);
                machineSubstate++;

                if (machineSubstate >= 4)
                {
                    /* Valido que el campo longitud tenga sentido */
                    if (hostOrderWord <=  COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE)
                    {
                        /* Guardo el valor de la longitud del mensaje en el campo correspondiente del frame */
                        rxContainerPtr->userData.messageLength	= hostOrderWord;

                        /* Paso al estado siguiente de la máquina */
                        machineState = st3RecvMsgBody;
                        machineSubstate = 0;

                        /* Inicializo en cero el checksum */
                        calculatedChecksum = 0;

                    } else {
                        /* Falló la verificación del campo de longitud, por lo que reinicio la máquina para busque nuevamente el header */
                        machineState = st1RecvHeader;
                        networkOrderWord = 0;
                    }
                }
                break;

            case st3RecvMsgBody:
                /* Recibo el contenido del cuerpo del mensaje */
                rxContainerPtr->userData.messageBuffer[machineSubstate] = receivedByte;

                /* Simultáneamente calculo el checksum del cuerpo del mensaje */
                calculatedChecksum += (uint32_t)receivedByte;
                calculatedChecksum &= 0x7fffffff;

                machineSubstate++;
                if (machineSubstate >= rxContainerPtr->userData.messageLength)
                {
                    /* Se recibió el último byte del cuerpo del mensaje, paso a recibir los campos del final */
                    machineState = st4RecvChecksum;
                    machineSubstate = 0;
                }
                break;

            case st4RecvChecksum:
                /* Recibo los cuatro bytes del checksum transmitido */
                networkOrderWord = commTransportRtcmRotateRightAndInsertByte(networkOrderWord, receivedByte);
                hostOrderWord    = cpuBigEndianToLocalU32(networkOrderWord);
                machineSubstate++;

                if (machineSubstate >= 4)
                {
                    /* Contrasto el checksum recibido contra el calculado localmente */
                    if (hostOrderWord == calculatedChecksum)
                    {
                        /* Checksum correcto, paso al siguiente estado de la máquina */
                        machineState = st5RecvTail;
                        machineSubstate = 0;
                    } else {
                        /* Falla de verificación de checksum, reinicio la máquina de estados */
                        machineState = st1RecvHeader;
                        networkOrderWord = 0;
                    }
                }
                break;

            case st5RecvTail:
                /* Recibo el identificador de fin de mensaje */
                networkOrderWord = commTransportRtcmRotateRightAndInsertByte(networkOrderWord, receivedByte);
                hostOrderWord    = cpuBigEndianToLocalU32(networkOrderWord);
                machineSubstate++;

                if (machineSubstate >= 4)
                {
                    /* Verifico el campo terminador de mensaje */   /* Esto hay que verlo... NO HAY POSTFIX */
                    if (hostOrderWord == COMM_TRANSPORT_STANDARD_FRAME_POSTFIX)
                    {
                        /* Recepción exitosa. Reinicio la máquina de estados y devuelvo el frame recién recibido */
                        machineState = st1RecvHeader;
                        machineSubstate = 0;

                        /* Indico que hay un mensaje nuevo recibido completo! */
                        newFrameReady = trueValue;

                    } else {
                        /* No se recibió el terminador de mensaje correcto, reinicio la máquina de estados */
                        machineState = st1RecvHeader;
                        networkOrderWord = 0;
                    }
                }
                break;
            default:
                /* Estado desconocido, reinicio */
                machineState = st1RecvHeader;
                networkOrderWord = 0;
                machineSubstate = 0;
                break;
        }
    }

    if (newFrameReady == falseValue)
    {
        /* si esta condición es cierta significa que salí del bucle por alguna razónç
         * diferente de que se haya recibido exitosamente un mensaje. En este caso
         * libero los recursos que haya reservado. */

        commTransportRtcmReleaseMessageContainer((void *)transportInternalData, rxContainerPtr);

        rxContainerPtr = NULL;
    }

    return rxContainerPtr;
}


osTaskReturnType commTransportRtcmReceiverTask(osTaskArgumentType genericInternalTransportDataPtr)
{
    commTransportRtcmProtocolData *transportInternalData = (commTransportRtcmProtocolData *)genericInternalTransportDataPtr;
    commTransportRtcmContainerType *packetContainerPtr;
    osReturnValueType retValue;

    while (1)
    {
        /* Recibo un contenedor de paquete completo. Esta rutina es bloqueante. */
        packetContainerPtr = commTransportRtcmSyncAndReceivePacket(transportInternalData);

        /* Una condición posible bajo la cual la función anterior puede devulver NULL es si
         * se agotan los buffers de recepción. En ese caso simplemente espero un tiempo prudente para evitar entrar en
         * un loop cerrado, y dar tiempo a que se liberen bloques. */
        if (packetContainerPtr != NULL)
        {
            /* En colo el contenedor del paquete en la cola de recepción */
            retValue = osMessageQueueSend(transportInternalData->rxQueueId, (void*)&packetContainerPtr, sizeof(commTransportRtcmContainerType *));

            /* si por algún motivo falla el envío, elimino el frame para liberar la memoria */
            if (retValue != OS_SUCCESS)
            {
                commTransportRtcmReleaseMessageContainer((void*)transportInternalData, packetContainerPtr);
            }

            /* Si hay una tarea de procesamiento registrada por la capa de protocolo, la activo */
            if(transportInternalData->protocolProcessingTaskRegistered == trueValue)
            {
                osSendTaskActivationSignal(transportInternalData->protocolProcessingTaskId);
            }

        } else {
            osTaskWakeAfter(20);
        }
    }
}
#endif

commTransportRtcmControlBlockType *commTransportRtcmCreateControlBlock(commDeviceControlBlockType *device, uint32_t smallTxContainersCount, uint32_t hugeTxContainersCount, uint32_t fullTxContainersCount)
{
    commTransportRtcmControlBlockType *newTransportControlBlockPtr;
    commTransportRtcmProtocolData *newTransportInternalDataPtr;
    /* No se esta revisando disponibilidad.
    systemAssert(smallTxCountCriticalThreshold < smallTxContainersCount, "The number of blocks for critical packages exceeds the overall number of tx packages.");
    */

    /* Creo el bloque de control de la nueva instancia del protocolo */
    newTransportControlBlockPtr = osMemAlloc(sizeof(commTransportRtcmControlBlockType));
    systemAssert(newTransportControlBlockPtr != NULL, "Not enough memory!");

    /* Creo el bloque de información del protocolo */
    newTransportInternalDataPtr = osMemAlloc(sizeof(commTransportRtcmProtocolData));
    systemAssert(newTransportInternalDataPtr != NULL, "Not enough memory!");

    /* Inicializo las estructuras en cero */
    utilZeroMem(newTransportControlBlockPtr, sizeof(commTransportRtcmControlBlockType));
    utilZeroMem(newTransportInternalDataPtr, sizeof(commTransportRtcmProtocolData));

    /* ********************************************** */

    /* Inicializo los datos del dispositivo que requieren los handlers */
    newTransportInternalDataPtr->deviceLayerHandle = device;

    /*
     * Creo los buffers de memoria de transmisión y recepción, y las particiones correspondientes.
     * */

    /* Tx de tamaño pequeño */
    newTransportInternalDataPtr->smallTxContainerPartitionsStorageSpace = osMemAlloc(smallTxContainersCount * sizeof(commTransportRtcmContainerType));
    systemAssert(newTransportInternalDataPtr->smallTxContainerPartitionsStorageSpace != NULL, "Not enough memory!");

    newTransportInternalDataPtr->smallTxPayloadPartitionsStorageSpace = osMemAlloc(smallTxContainersCount * COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE);
    systemAssert(newTransportInternalDataPtr->smallTxPayloadPartitionsStorageSpace != NULL, "Not enough memory!");

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->smallTxContainerPartitionsStorageSpace,
            smallTxContainersCount * sizeof(commTransportRtcmContainerType),
            sizeof(commTransportRtcmContainerType),
            &(newTransportInternalDataPtr->smallTxMessageContainerPartitionId));

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->smallTxPayloadPartitionsStorageSpace,
            smallTxContainersCount * COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE,
            COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE,
            &(newTransportInternalDataPtr->smallTxMessagePayloadPartitionId));

    /* Tx de gran tamaño */
    newTransportInternalDataPtr->hugeTxContainerPartitionsStorageSpace = osMemAlloc(hugeTxContainersCount * sizeof(commTransportRtcmContainerType));
    systemAssert(newTransportInternalDataPtr->hugeTxContainerPartitionsStorageSpace != NULL, "Not enough memory!");

    newTransportInternalDataPtr->hugeTxPayloadPartitionsStorageSpace = osMemAlloc(hugeTxContainersCount * COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE);
    systemAssert(newTransportInternalDataPtr->hugeTxPayloadPartitionsStorageSpace != NULL, "Not enough memory!");

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->hugeTxContainerPartitionsStorageSpace,
            hugeTxContainersCount * sizeof(commTransportRtcmContainerType),
            sizeof(commTransportRtcmContainerType),
            &(newTransportInternalDataPtr->hugeTxMessageContainerPartitionId));

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->hugeTxPayloadPartitionsStorageSpace,
            hugeTxContainersCount * COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE,
            COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE,
            &(newTransportInternalDataPtr->hugeTxMessagePayloadPartitionId));

    /* Tx de tamaño completo */
    newTransportInternalDataPtr->fullTxContainerPartitionsStorageSpace = osMemAlloc(fullTxContainersCount * sizeof(commTransportRtcmContainerType));
    systemAssert(newTransportInternalDataPtr->fullTxContainerPartitionsStorageSpace != NULL, "Not enough memory!");

    newTransportInternalDataPtr->fullTxPayloadPartitionsStorageSpace = osMemAlloc(fullTxContainersCount * COMM_TRANSPORT_RTCM_FULL_CONTAINER_SIZE);
    systemAssert(newTransportInternalDataPtr->fullTxPayloadPartitionsStorageSpace != NULL, "Not enough memory!");

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->fullTxContainerPartitionsStorageSpace,
            fullTxContainersCount * sizeof(commTransportRtcmContainerType),
            sizeof(commTransportRtcmContainerType),
            &(newTransportInternalDataPtr->fullTxMessageContainerPartitionId));

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->fullTxPayloadPartitionsStorageSpace,
            fullTxContainersCount * COMM_TRANSPORT_RTCM_FULL_CONTAINER_SIZE,
            COMM_TRANSPORT_RTCM_FULL_CONTAINER_SIZE,
            &(newTransportInternalDataPtr->fullTxMessagePayloadPartitionId));

#ifdef RTCM_RECEIVE
    newTransportInternalDataPtr->rxContainerPartitionsStorageSpace = osMemAlloc(rxContainersCount * sizeof(commTransportRtcmContainerType));
    systemAssert(newTransportInternalDataPtr->rxContainerPartitionsStorageSpace != NULL, "Not enough memory!");

    newTransportInternalDataPtr->rxPayloadPartitionsStorageSpace = osMemAlloc(rxContainersCount * COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE);
    systemAssert(newTransportInternalDataPtr->rxPayloadPartitionsStorageSpace != NULL, "Not enough memory!");

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->rxContainerPartitionsStorageSpace,
            rxContainersCount * sizeof(commTransportRtcmContainerType),
            sizeof(commTransportRtcmContainerType),
            &(newTransportInternalDataPtr->rxMessageContainerPartitionId));

    osMemoryPartitionCreate(
            newTransportInternalDataPtr->rxPayloadPartitionsStorageSpace,
            rxContainersCount * COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE,
            COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE,
            &(newTransportInternalDataPtr->rxMessagePayloadPartitionId));
#endif

    /* Creo las colas de transmisión y recepción */
    osMessageQueueCreate(
            "QRTK",
            smallTxContainersCount + hugeTxContainersCount, /* En el peor caso esta cola tiene que acomodar la totalidad de ambos tipos de contenedor */
            sizeof(commTransportRtcmContainerType *),
            &newTransportInternalDataPtr->txQueueId);

#ifdef RTCM_RECEIVE
    osMessageQueueCreate(
            "RQUE",
            rxContainersCount,
            sizeof(commTransportRtcmContainerType *),
            &newTransportInternalDataPtr->rxQueueId);
#endif

    /* ********************************************** */

    /* Creo el mutex de acceso a la estructura de datos para los datos que no
     * tienen protección contra el acceso múltiple. */
    newTransportInternalDataPtr->accessMutex = osMutexCreate(simpleMutexFree);

    /* Estadísticas de los bloques de memoria de Tx regulares */
    newTransportInternalDataPtr->smallTxContainersInUse = 0;
    newTransportInternalDataPtr->smallTxContainersInUseMax = 0;

    /* Estadísticas de los bloques de memoria de Tx grandes */
    newTransportInternalDataPtr->hugeTxContainersInUse = 0;
    newTransportInternalDataPtr->hugeTxContainersInUseMax = 0;

#ifdef RTCM_RECEIVE
    /* Estadísticas de los bloques de memoria de Rx */
    newTransportInternalDataPtr->rxContainersInUse = 0;
    newTransportInternalDataPtr->rxContainersInUseMax = 0;
#endif

    /* Parámetros de configuración del protocolo */
    newTransportInternalDataPtr->smallTxTotalContainers = smallTxContainersCount;
    newTransportInternalDataPtr->hugeTxTotalContainers = hugeTxContainersCount;
#ifdef RTCM_RECEIVE
    newTransportInternalDataPtr->rxTotalContainers = rxContainersCount;
#endif
/*  No es necesario reservar contenedores para mensajes criticos ya que esta interfaz solo trabaja con RTCM (no criticos)
    Cantidad de contenedores reservados en la cola de transmisión para paquetes críticos 
    newTransportInternalDataPtr->smallTxContainersCriticalThreshold = smallTxCountCriticalThreshold;
*/
    /* ********************************************** */

    /* Estos campos son inicializados posteriormente cuando se arme el stack layer1/layer2/layer3 entero. */
    newTransportInternalDataPtr->protocolProcessingTaskId = 0;
    newTransportInternalDataPtr->protocolProcessingTaskRegistered = falseValue;

    /* ********************************************** */

    /* Inicializo la estructura del bloque de control */
    newTransportControlBlockPtr->transportDescription = "MGGR RTCM Transport Layer";

    newTransportControlBlockPtr->genericInternalTransportDataPtr = (void *)newTransportInternalDataPtr;

    newTransportControlBlockPtr->transportAllocMessageContainer = commTransportRtcmAllocMessageContainer;
    newTransportControlBlockPtr->transportReleaseMessageContainer = commTransportRtcmReleaseMessageContainer;
    newTransportControlBlockPtr->transportRegisterProtocolProcessingTask = commTransportRtcmRegisterProtocolProcessingTask;
    newTransportControlBlockPtr->transportAddMessageContainerToTxQueue = commTransportRtcmAddMessageContainerToTxQueue;
    newTransportControlBlockPtr->transportRxQueueNotEmpty = commTransportRtcmRxQueueNotEmpty;
    newTransportControlBlockPtr->transportGetContainerFromRxQueueNonBlocking = commTransportRtcmGetContainerFromRxQueueNonBlocking;

    /* ********************************************** */

    /* Luego de que están todos los datos inicializados creo las tareas de transmisión y recepción */
    osTaskCreateWithArgument("TRTK",
            PRIO_COMM_LAYER2_TX_TASK,
            OS_MINIMUM_STACK_SIZE,
            OS_PREEMPT|OS_TIMESLICE,
            OS_NO_FLOATING_POINT,
            commTransportRtcmTransmitterTask,
            &newTransportInternalDataPtr->txTaskId,
            (osTaskArgumentType)newTransportInternalDataPtr);
#ifdef RTCM_RECEIVE
    osTaskCreateWithArgument("RRTK",
            PRIO_COMM_LAYER2_RX_TASK,
            OS_MINIMUM_STACK_SIZE,
            OS_PREEMPT|OS_TIMESLICE,
            OS_NO_FLOATING_POINT,
            commTransportRtcmReceiverTask,
            &newTransportInternalDataPtr->rxTaskId,
            (osTaskArgumentType)newTransportInternalDataPtr);
#endif

    return newTransportControlBlockPtr;
}