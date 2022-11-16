#ifndef RTCMDATATYPES_H_
#define RTCMDATATYPES_H_

/** \file */

#include "datatypes.h"          /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "configuration.h"      /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "receiverdatatypes.h"  /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "navinternal.h"        /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "rtcmframe.h"
#include "satellitedb.h"        /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include <sys/types.h>

#define GPS_ONLY

#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */
#define SNR_UNIT    0.001               /* SNR unit (dBHz) */
#define P2_10       0.0009765625          /* 2^-10 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */
#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

#define FREQ1       1.57542E9           /* L1/E1/B1C  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2         frequency (Hz) */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless,B1codeless (GPS,BDS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A,B1A    (GAL,BDS) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1S (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2bI  (GAL,BDS) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2bQ  (GAL,BDS) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
#define CODE_L6A    30                  /* obs code: E6A,B3A    (GAL,BDS) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C,L6D+E (GAL,QZS) */
#define CODE_L6S    35                  /* obs code: L6S        (QZS) */
#define CODE_L6L    36                  /* obs code: L6L        (QZS) */
#define CODE_L8I    37                  /* obs code: E5abI      (GAL) */
#define CODE_L8Q    38                  /* obs code: E5abQ      (GAL) */
#define CODE_L8X    39                  /* obs code: E5abI+Q,B2abD+P (GAL,BDS) */
#define CODE_L2I    40                  /* obs code: B1_2I      (BDS) */
#define CODE_L2Q    41                  /* obs code: B1_2Q      (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) (obsolute) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) (obsolute) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define CODE_L1D    56                  /* obs code: B1D        (BDS) */
#define CODE_L5D    57                  /* obs code: L5D(L5S),B2aD (QZS,BDS) */
#define CODE_L5P    58                  /* obs code: L5P(L5S),B2aP (QZS,BDS) */
#define CODE_L5Z    59                  /* obs code: L5D+P(L5S) (QZS) */
#define CODE_L6E    60                  /* obs code: L6E        (QZS) */
#define CODE_L7D    61                  /* obs code: B2bD       (BDS) */
#define CODE_L7P    62                  /* obs code: B2bP       (BDS) */
#define CODE_L7Z    63                  /* obs code: B2bD+P     (BDS) */
#define CODE_L8D    64                  /* obs code: B2abD      (BDS) */
#define CODE_L8P    65                  /* obs code: B2abP      (BDS) */
#define CODE_L4A    66                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
#define MAXCODE     68                  /* max number of obs code */

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1
#ifndef GPS_ONLY
    #define MAXSAT      (NSATGPS+NSATGLO)
                                        /* max satellite number (1 to MAXSAT) */
#else 
    #define MAXSAT      (NSATGPS)
#endif  /* GPS_ONLY */

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif  /*  ENAGLO  */

#define NFREQ       2                   /* number of carrier frequencies */
#define NEXOBS      0                   /* number of extended obs codes */
#define NUM_SIGNALS_Q 2

#define DF401_RANGE 1171.0              /* Valor maximo admitido por este DF (referir a DF401 Range en c10403.3 - DGNSS Seccion 3.4) */

typedef struct {        /* RTCM control struct type - MSG 1005 - adapted for QSERIES Rx */
    uint16_t stationId;          /* station id */
    globalECEFPositionSolutionType position;
    uint16_t numBytes;          /* number of bytes in message buffer */ 
    uint16_t numBits;           /* number of bits in word buffer */ 
    uint16_t messageLength;            /* message length (bytes) */
    uint8_t messageContent[COMM_TRANSPORT_RTCM_SMALL_CONTAINER_SIZE];    /* message buffer*/
} rtcmControl1005Type;

typedef struct {
    uint32_t receiverTime;  /* tow */
    int32_t receiverGpsWeek;
} gpsTimeType;

typedef struct {
    uint8_t observationNum;
    uint8_t codeIndicator;
    globalSpaceVehicleIdType svid;
    double psrngObsData;        /**< Observacion de pseudorango [m]   */
    double phrngObsData;        /**< Accumulated Delta Carrier del lazo de portadora, en ciclos de portadora Q20. */
    double carrierDoppler;      /**< Corrimiento doppler de la portadora [Hz] */
    double carrierPhaseMeas;    /**< Cantidad de Nros. enteros de longitudes de onda en la medicion fina*/
    uint8_t lossOfLockIndicator;/**< 3bit indicator. Section 6.7.1 RINEX 3.05 */
    uint8_t logScaleCN0;        /**< Cociente CN0 en escala logaritmica. */
    uint16_t lastLockTime;      /**< Tiempo en el que logro engancharse.  */

} observableType;

typedef struct {                /* RTCM control struct type - MSG 1074 - adapted for QSERIES Rx */
    uint16_t stationId;         /* station id */
    uint16_t numBytes;          /* number of bytes in message buffer */ 
    uint16_t numBits;           /* number of bits in word buffer */ 
    uint16_t messageLength;     /* message length (bytes) */
    uint8_t messageContent[COMM_TRANSPORT_RTCM_HUGE_CONTAINER_SIZE];    /* message buffer*/
    uint8_t numSatellites;
    uint8_t numSignals;
    uint8_t numObservables;
    gpsTimeType obsTime;
    observableType observablesSet[RECEIVER_RANGING_DATA_TABLES_SIZE];
    booleanData validityFlag;
    booleanData syncFlag;
} rtcmControl1074Type;

typedef struct {

    uint32_t serviceId;                                 /**< Número de servicio de generación de mediciones al que corresponde el contenido de la estructura. */
    uint32_t timestamp;                                 /**< Marca de tiempo de generación de las mediciones presentes en el resto de la estructura. */

    booleanData dataValidL1[NAV_MAX_MEASURES];          /**< Indicador de validez de las entradas correspondientes en las entradas correspondientes de los demás campos. */
    booleanData dataValidL2[NAV_MAX_MEASURES];          /**< Indicador de validez de las mediciones L2. */
    booleanData dataValidIF[NAV_MAX_MEASURES];          /**< Indicador de validez de las mediciones IF. */

    int32_t     idxMeasL1forL2C[NAV_MAX_MEASURES];      /**< Índice de la medición L1 que corresponde al mismo SVID que una dada medición L2C */

    globalSpaceVehicleIdType svid[NAV_MAX_MEASURES];    /**< Identificador del satélite del que proviene la medición */

    globalSignalIdType signalId[NAV_MAX_MEASURES];      /**< Identificador de la señal a la que corresponde la medición */

    uint32_t observableId[NAV_MAX_MEASURES];            /**< Identificador único de mediciones provenientes de un mismo enganche, y por ende relacionadas entre sí. */

    double transmitTime[NAV_MAX_MEASURES];          /**< Tiempo de transmisión. */
    double carrierDeviation[NAV_MAX_MEASURES];      /**< Desviación de frecuencia portadora de la señal del satélite respecto del valor nominal. */

    double svidPos[NAV_MAX_MEASURES][3];                /**< Coordenadas ECEF de la posición del satélite transmisor en el tiempo de transmisión. */
    double svidVel[NAV_MAX_MEASURES][3];                /**< Coordenadas ECEF de la velocidad del satélite transmisor en el tiempo de transmisión. */

    double svidElev[NAV_MAX_MEASURES];
    double svidCosAz[NAV_MAX_MEASURES];
    double svidSinAz[NAV_MAX_MEASURES];

    double pseudoRanges[NAV_MAX_MEASURES];              /**< Pseudorango, en metros, con todas las correcciones aplicadas. */
    double deltaRanges[NAV_MAX_MEASURES];               /**< Deltarango, en metros/seg, con todas las correcciones aplicadas. */

    double pseudoRangesIF[NAV_MAX_MEASURES];            /**< Pseudorango Iono-free, en metros, con todas las correcciones aplicadas. */

	double linearScaleCNoInv[NAV_MAX_MEASURES];


} rtcmInternalRangingDataStruct;

#endif /* RTCMDATATYPES_H_ */