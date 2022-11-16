#ifndef RTCMMESSAGES_H_
#define RTCMMESSAGES_H_

/** \brief Definición de los identificadores de mensajes válidos en el protocolo rtcm.
 */
 typedef enum {
    commProtoRtcmMessageIdNull                     = 0x00 /**< Identificador nulo. No corresponde a ningún mensaje válido. */,
    commProtoRtcmMessageId1005                     = 0x01 /**< Identificador correspondiente al mensaje RTCM 1005. */,
    commProtoRtcmMessageId1074                     = 0x02 /**< Identificador correspondiente al mensaje RTCM 1074 (MSM4 de GPS). */
} commProtoRtcmMessageIdType;



/* ******************************************************************
 * DEFINICIÓN DE MENSAJES DEL PROTOCOLO
 *
 * */

/** \brief Estructura estandar de los mensajes RTCM. */
typedef struct {

    uint8_t header_1;                                /**< Campo para el header del mensaje. Preambulo fijo 0xD3. */

    uint16_t header_2;                                /**< Segundo y tercer byte del header. [6_bits_en_0  10bits_de_msglen] */
    
} header_rtcm;

typedef struct {
    
    uint8_t footer_1;                                 /**< Campo para el footer del mensaje. 1er byte de paridad con CRC24q. */

    uint16_t footer_2;                                 /**< Campo para el footer del mensaje. 2do y 3er byte de paridad con CRC24q. */

} footer_rtcm;

typedef struct {

    uint8_t header_1;                                /**< Campo para el header del mensaje. Preambulo fijo 0xD3. */

    uint16_t header_2;                                /**< Segundo y tercer byte del header. [6_bits_en_0  10bits_de_msglen] */
    
    uint32_t content[4];                               /**< Contenido del mensaje. El valor 4 es generico, luego se reserva y utiliza mas.*/

    uint8_t footer_1;                                 /**< Campo para el footer del mensaje. 1er byte de paridad con CRC24q. */

    uint16_t footer_2;                                 /**< Campo para el footer del mensaje. 2do y 3er byte de paridad con CRC24q. */

} __attribute__((packed)) commProtoRtcmMessage;

#endif /* RTCMMESSAGES_H_ */
