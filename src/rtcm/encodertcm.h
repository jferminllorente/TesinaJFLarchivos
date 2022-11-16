#ifndef ENCODER_RTCM_H_
#define ENCODER_RTCM_H_

#include "rtcmdatatypes.h"
#include "gnssconstants.h"              /**< Archivo propio del proyecto Receptor GNSS SENyT. */
#include "math.h"
#include "internalreceiverinterface.h"  /**< Archivo propio del proyecto Receptor GNSS SENyT. */

#define ROUND(x)    ((int)floor((x)+0.5))
#define ROUND_U(x)  ((uint32_t)floor((x)+0.5))

/* MSM signal ID table -------------------------------------------------------*/
static char *msm_sig_gps[] = {
    /* GPS: ref [17] table 3.5-91 */
    ""  ,"1C","1P","1W",""  ,""  ,""  ,"2C","2P","2W",""  ,""  , /*  1-12 */
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};

static char *obscodes[]={       /* observation code strings */
    
    ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
    "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
    "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
    "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
    "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
    "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", /* 50-59 */
    "6E","7D","7P","7Z","8D", "8P","4A","4B","4X",""    /* 60-69 */
};
typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} gtime_t;

static const uint16_t gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */

char *code2obs(uint8_t code);
uint8_t satsys(uint8_t sat, uint8_t *prn);
uint8_t to_satid(uint8_t sys, uint8_t sat);
uint8_t to_sigid(uint8_t sys, uint8_t code);
void gen_msm_index(rtcmControl1074Type *rtcm, uint8_t sys, uint8_t *nsat, uint8_t *nsig,
                          uint16_t *ncell, uint8_t *sat_ind, uint8_t *sig_ind,
                          uint16_t *cell_ind);
uint16_t encode_msm_head(uint16_t type, rtcmControl1074Type *rtcm, uint8_t sys, booleanData sync, uint8_t *nsat,
                           uint16_t *ncell, double *rrng,
                           double *psrng, double *phrng,
                           uint16_t *lock, uint8_t *half,
                           uint8_t *cnr, booleanData *flagFirstRtcm);
void gen_msm_sat(rtcmControl1074Type *rtcm, uint8_t sys, const uint8_t *sat_ind,
                        double *rrng);
void gen_msm_sig(rtcmControl1074Type *rtcm, uint8_t sys, uint8_t nsig, uint16_t ncell,
                        const uint8_t *sat_ind, const uint8_t *sig_ind,
                        const uint16_t *cell_ind, const double *rrng,
                        double *psrng, double *phrng,
                        uint16_t *lock, uint8_t *half, uint8_t *cnr, booleanData *flagFirstRtcm);
uint16_t encode_msm_int_rrng(rtcmControl1074Type *rtcm, uint16_t i, const double *rrng,
                               uint8_t nsat);
uint16_t encode_msm_mod_rrng(rtcmControl1074Type *rtcm, uint16_t i, const double *rrng,
                               uint8_t nsat);
uint16_t encode_msm_psrng(rtcmControl1074Type *rtcm, uint16_t i, const double *psrng, uint16_t ncell);
uint16_t encode_msm_phrng(rtcmControl1074Type *rtcm, uint16_t i, const double *phrng, uint16_t ncell);
uint16_t encode_msm_lock(rtcmControl1074Type *rtcm, uint16_t i, uint16_t *lock, uint16_t ncell);
uint16_t encode_msm_half_amb(rtcmControl1074Type *rtcm, uint16_t i, const uint8_t *half,
                               uint16_t ncell);
uint16_t encode_msm_cnr(rtcmControl1074Type *rtcm, uint16_t i, uint8_t *cnr, uint16_t ncell);

void setbitu(uint8_t *buff, int pos, int len, uint32_t data);
void set38bits(uint8_t *buff, int pos, double value);
void setbits(uint8_t *buff, int pos, int len, int32_t  data);
gtime_t epoch2time(const uint16_t *ep);
double time2gpst(gtime_t t, int *week);
errorType rtcmGpsCalculateSignalTransmissionTimes(globalRangingInfoSetType *pMeasurementSet, rtcmInternalRangingDataStruct *pGpsMeas);
uint8_t lockTime2LockTimeIndicator(uint32_t trackingT);
int code2freq_GPS(uint8_t code, double *freq);
double code2freq(int sys, uint8_t code, int fcn);

#endif /* ENCODER_RTCM_H_ */