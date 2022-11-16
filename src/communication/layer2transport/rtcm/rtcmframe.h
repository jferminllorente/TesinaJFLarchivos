#ifndef RTCMFRAME_H_
#define RTCMFRAME_H_

#define COMM_TRANSPORT_RTCM_FRAME_PREFIX  (0xD3)    /**< RTCMv3.3 preamble */

#define COMM_TRANSPORT_RTCM_HEADER_LENGTH (3)       /**< preamble (8bits) + zeros (6bits) + msglength (10bits) */
#define COMM_TRANSPORT_RTCM_FOOTER_LENGTH (3)       /**< crc parity = 24 bits */

#define COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE (30)
#define COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE (500)
#define COMM_TRANSPORT_RTCM_FULL_CONTAINER_SIZE (1206)
/* JFL: 
 * MSG 1005 completo tiene 25 bytes
 * MSG 1074 completo tiene 456 bytes
 * rtklib tiene buffer de tamaño 1200 bytes + 6 header y footer. 
 * Contenedor FULL alcanza para todos los mensajes de rtklib. 
 * */

#endif /* RTCMFRAME_H_ */