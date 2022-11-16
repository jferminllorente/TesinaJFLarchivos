
#ifndef COMMPROTOCOL_H_
#define COMMPROTOCOL_H_

typedef enum {txContainerSmall, txContainerHuge, txContainerFull} commTransportRtcmContainerSourceIdType;

typedef struct  {

    uint8_t *messageBuffer; /**< Espacio de memoria para colocar el mensaje. */
    uint16_t messageLength; /**< Cantidad de memoria ocupada por el mensaje. Puede ser menor que la cantidad reservada. */
    /* JFL:
     * Se representa con 16 bits pues el maximo largo de mensaje es 1200 bytes de buffer + 6 bytes de header y footer.
     */
    
    int32_t dstAddress; /**< Dirección del destinatario. Si vale -1 es un broadcast o punto a punto. */
    int32_t srcAddress; /**< Dirección del origen. Si vale -1 es para conexión punto a punto. */

} commTransportRtcmContainerUserDataType;

typedef struct  {

    commTransportRtcmContainerSourceIdType sourceId; /**< Identificador de la fuente del buffer de mensaje, RX o TX. */

    uint8_t *containerBuffer; /**< Espacio de memoria para colocar el mensaje con el encabezado y el pie. */

    uint16_t messageBufferSize; /**< Cantidad máxima de datos que puede colocar el usuario en el buffer. 1200 bytes*/
    uint16_t containerBufferSize; /**< Cantidad máxima de datos que pueden colocarse en el buffer. 1206 bytes*/

} commTransportRtcmContainerSystemDataType;

typedef struct  {

    commTransportRtcmContainerUserDataType userData; /**< Datos útiles que son de utilidad al emisor del mensaje. */

    commTransportRtcmContainerSystemDataType systemData;  /**< Datos de utilización interna que el usuario no debe modificar. */

} commTransportRtcmContainerType;

typedef struct {

    const char *transportDescription;

    void *genericInternalTransportDataPtr;

    /* ***************** */
    commTransportRtcmContainerType *(*transportAllocMessageContainer)(
            void *genericInternalTransportDataPtr,
            uint32_t buffersize,
            commTransportRtcmContainerSourceIdType sourceId);

    void (*transportReleaseMessageContainer)(
            void *genericInternalTransportDataPtr,
            commTransportRtcmContainerType *containerBuffer);

    void (*transportRegisterProtocolProcessingTask)(
            void *genericInternalTransportDataPtr,
            osTaskIdType clientTaskId);

    void (*transportAddMessageContainerToTxQueue)(
            void *genericInternalTransportDataPtr,
            commTransportRtcmContainerType *containerBuffer);

    booleanData (*transportRxQueueNotEmpty)(
            void *genericInternalTransportDataPtr);

    int32_t (*transportGetContainerFromRxQueueNonBlocking)(
            void *genericInternalTransportDataPtr,
            commTransportRtcmContainerType **containerBuffer);

} commTransportRtcmControlBlockType;

#endif // COMMPROTOCOL_H_