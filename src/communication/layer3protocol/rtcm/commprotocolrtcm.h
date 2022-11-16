#ifndef COMMPROTOCOLRTCM_H_
#define COMMPROTOCOLRTCM_H_

#include "../commprotocollayer.h"                           /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "../../layer2transport/commtransportlayer.h"       /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "eventengine.h"                                    /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "navgps.h"                                         /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "rtcmdatatypes.h"

#ifdef GPS_DUAL_BAND_MODE
#define RTCM_NUMBER_OF_SIGNALS (2)
#else
#define RTCM_NUMBER_OF_SIGNAL (1)
#endif

#define LLI_HALF_AMB    1
#define LLI_CYCLE_SLIP  0

typedef struct {

    commTransportRtcmControlBlockType *transportLayerHandle; /**< Puntero a la estructura con los datos de la capa de transporte. */

    osTaskIdType protocolProcessingTaskId; /**< Identificador de la tarea de procesamiento del protocolo. */

    eventClientDataType *eventClientData; /**< Handle del sistema de la cola de eventos de esta tarea. */

} commProtocolRtcmProtocolData;

void commProtocolRtcmGenerateMessage1005DataFields(rtcmControl1005Type *rtcm);
int8_t commProtocolRtcmGenerateMessage1074DataFields(rtcmControl1074Type *rtcm, globalRangingInfoSetType *observableSet);
#ifdef RTCM_FF
void commProtocolRtcmPackageConstantMessage(commProtocolRtcmProtocolData *protocolInternalData,commProtoRtcmMessageIdType type);
#endif
void commProtocolRtcmPackageMessage1005(commProtocolRtcmProtocolData *protocolInternalData);
void commProtocolRtcmPackageMessage1074(commProtocolRtcmProtocolData *protocolInternalData, globalRangingInfoSetType *observableSet, booleanData *flagFirstRtcm);
int32_t commProtocolRtcmProcessReceiverEvents(commProtocolRtcmProtocolData *protocolInternalData);
osTaskReturnType commProtocolRtcmProcessingTask(osTaskArgumentType genericInternalDataPtr);
commProtocolControlBlockType *commProtocolRtcmCreateControlBlock(commTransportRtcmControlBlockType *transport);

#endif /* COMMPROTOCOLRTCM_H_ */