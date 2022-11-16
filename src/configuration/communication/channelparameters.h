
#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/* ---------------------------- Protocolo RTCM ------------------------------ */
/**  \brief Nombre del dispositivo de comunicación serie utilizado en la interfaz RTCM.
 *
 * Dispositivos válidos son: /dev/console, /dev/console_b, /dev/console_c, etc.
 */
#define COMM_INTERFACE_RTCM_DEVICE_NAME "/dev/console_b"

/**  \brief Tasa de comunicación serie utilizado en la interfaz RTCM.
 *
 * Valores válidos son los baudrates estandar: 1200, 2400, 4800, 9600, 19200, 38400, 115200, etc.
 */
#define COMM_INTERFACE_RTCM_DEVICE_BITRATE (230400) /* Para primeras pruebas (maxima admitida) */

/**  \brief Cantidad de contenedores de transmisión de tamaño regular en la interfaz RTCM.
 */
#define COMM_INTERFACE_RTCM_SMALL_TX_CONTAINER_COUNT (8)

/**  \brief Cantidad de contenedores de transmisión de tamaño grande en la interfaz RTCM.
 */
#define COMM_INTERFACE_RTCM_HUGE_TX_CONTAINER_COUNT (16)

/**  \brief Cantidad de contenedores de transmisión de tamaño completo en la interfaz RTCM.
 */
#define COMM_INTERFACE_RTCM_FULL_TX_CONTAINER_COUNT (2)

/**  \brief Largo de la cola de recepción de la interfaz RTCM.
 */
#define COMM_INTERFACE_RTCM_RX_CONTAINER_COUNT (0)

/**  \brief Cantidad mínimia de contenedores disponibles para transportar paquetes críticos. Si la cantidad de
 * contenedores disponibles cae por debajo de esta cantidad las rutinas de alloc dejan de asignar contenedores para
 * aplicaciones ordinarias.
 */
#define COMM_INTERFACE_RTCM_TX_CRITICAL_THRESHOLD 1

#endif /* COMMUNICATION_H_ */
