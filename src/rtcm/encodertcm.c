#include "encodertcm.h"
#include "commprotocolrtcm.h"

/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : uint8_t code     I   obs code (CODE_???)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
char *code2obs(uint8_t code)
{
    if (code <= CODE_NONE || MAXCODE < code){
        return "";
    }
    return obscodes[code];
}

/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
uint8_t satsys(uint8_t sat, uint8_t *prn)
{
    uint8_t sys = SYS_NONE;
    if (sat <= 0 || MAXSAT < sat){
        sat=0;
    }
    else if (sat<=NSATGPS) {
        sys = SYS_GPS;
        sat += MINPRNGPS - 1;
    }
#ifndef GPS_ONLY
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_CMP; sat+=MINPRNCMP-1; 
    }
    else if ((sat-=NSATCMP)<=NSATIRN) {
        sys=SYS_IRN; sat+=MINPRNIRN-1; 
    }
    else if ((sat-=NSATIRN)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1; 
    }
#endif
    else sat = 0;
    if (prn){
        *prn=sat;  /* Si prn != NULL entra en el condicional y carga la variable sat en *prn */
    }
    return sys;
}

/* satellite no to MSM satellite ID ------------------------------------------*/
uint8_t to_satid(uint8_t sys, uint8_t sat)
{
    uint8_t prn;
    
    if (satsys(sat,&prn) != sys){
        return 0;
    }
#ifndef GPS_ONLY
    if      (sys==SYS_QZS) prn-=MINPRNQZS-1;
    else if (sys==SYS_SBS) prn-=MINPRNSBS-1;
#endif

    return prn;
}

/* observation code to MSM signal ID -----------------------------------------*/
uint8_t to_sigid(uint8_t sys, uint8_t code)
{
    static char **msm_sig;
    char *sig;
    uint8_t i;
    
    /* signal conversion for undefined signal by rtcm */
    if (sys==SYS_GPS) {
        if      (code==CODE_L1Y){
            code=CODE_L1P;
        }
        else if (code==CODE_L1M){
            code=CODE_L1P;
        }
        else if (code==CODE_L1N){
            code=CODE_L1P;
        }
        else if (code==CODE_L2D){
            code=CODE_L2P;
        }
        else if (code==CODE_L2Y){
            code=CODE_L2P;
        }
        else if (code==CODE_L2M){
            code=CODE_L2P;
        }
        else if (code==CODE_L2N){
            code=CODE_L2P;
        }
    }
    if (!*(sig = code2obs(code))){
        return 0;
    }
    
    switch (sys) {
        case SYS_GPS:
            msm_sig = msm_sig_gps;
            break;
#ifndef GPS_ONLY
        case SYS_GLO:
            msm_sig = msm_sig_glo;
            break;
        case SYS_GAL:
            msm_sig = msm_sig_gal;
            break;
        case SYS_QZS:
            msm_sig = msm_sig_qzs;
            break;
        case SYS_SBS:
            msm_sig = msm_sig_sbs;
            break;
        case SYS_CMP:
            msm_sig = msm_sig_cmp;
            break;
        case SYS_IRN:
            msm_sig = msm_sig_irn;
            break;
#endif
        default:
            return 0;
            break;
    }
    for (i=0;i<32;i++) {
        if (!strcmp(sig,msm_sig[i])){
            return i+1;
        }
    }
    return 0;
}

void gen_msm_index(rtcmControl1074Type *rtcm, uint8_t sys, uint8_t *nsat, uint8_t *nsig,
                          uint16_t *ncell, uint8_t *sat_ind, uint8_t *sig_ind,
                          uint16_t *cell_ind)
{
    uint8_t i;
    uint8_t sat,sig;
    uint8_t cell;
    
    *nsat=*nsig=*ncell=0;

    /* generate satellite and signal index */
    /* satellite index (Table 3.5-90 GPS Satellite ID mapping) */
    /* signal index (Table 3.5-91 GPS Signal ID mapping) */
    for (i=0;i<rtcm->numObservables;i++) {
        if (!(sat=to_satid(sys,(rtcm->observablesSet[i].svid - svid_gps_first)))){
            continue;
        }
            
        if (!(sig=to_sigid(sys,rtcm->observablesSet[i].codeIndicator))){
            continue;
        }
        sat_ind[sat-1] = sig_ind[sig-1] = 1;
    }
    for (i=0;i<64;i++) {
        if (sat_ind[i]){
            sat_ind[i] =++(*nsat);
        }
    }
    for (i=0;i<32;i++) {
        if (sig_ind[i]){
            sig_ind[i] =++(*nsig);
        }
    }
    /* generate cell index */
    for (i=0;i<rtcm->numObservables;i++) {
        if (!(sat=to_satid(sys,(rtcm->observablesSet[i].svid - svid_gps_first)))) continue;
        
        if (!(sig = to_sigid(sys,rtcm->observablesSet[i].codeIndicator))){
            continue;
        }
        
        cell = sig_ind[sig-1]-1+(sat_ind[sat-1]-1)*(*nsig);
        cell_ind[cell] = 1;
    }
    for (i=0;i<*nsat*(*nsig);i++) {
        if (cell_ind[i]&&*ncell<64){
            cell_ind[i] =++(*ncell);
        }
    }
}
/* generate MSM satellite data fields ----------------------------------------*/
void gen_msm_sat(rtcmControl1074Type *rtcm, uint8_t sys, const uint8_t *sat_ind,
                        double *rrng)
{

    uint8_t i,k,sat,sig;
    
    for (i=0;i<64;i++){
        rrng[i]=0.0;
    }
    
    for (i=0;i<rtcm->numObservables;i++) {
        
        if (!(sat=to_satid(sys,(rtcm->observablesSet[i].svid - svid_gps_first)))){
            continue;
        }
        
        if (!(sig = to_sigid(sys,rtcm->observablesSet[i].codeIndicator))){
            continue;
        }
        k = sat_ind[sat-1]-1;
        
        /* rough range (ms) and rough phase-range-rate (m/s) */
        if ( (rrng[k] == 0.0) && (rtcm->observablesSet[i].psrngObsData != 0.0) ) {  /* Si ya se cargo el rrng del satelite k-esimo, no se vuelve a cargar porque es comun a cada satelite. */
            rrng[k] = ROUND( rtcm->observablesSet[i].psrngObsData/RANGE_MS/P2_10)*RANGE_MS*P2_10;
        }
    }
}
/* generate MSM signal data fields -------------------------------------------*/
void gen_msm_sig(rtcmControl1074Type *rtcm, uint8_t sys, uint8_t nsig, uint16_t ncell,
                        const uint8_t *sat_ind, const uint8_t *sig_ind,
                        const uint16_t *cell_ind, const double *rrng,
                        double *psrng, double *phrng,
                        uint16_t *lock, uint8_t *half, uint8_t *cnr, booleanData *flagFirstRtcm)
{
    double lambda, psrng_s, phrng_s;
    uint16_t i, lt;
    uint8_t sat, sig, cell, k, LLI, flag_ind;
    
    for (i=0;i<ncell;i++) {
        if (psrng){
            psrng[i] = 0.0;
        }
        if (phrng){
            phrng[i] = 0.0;
        }
    }
    for (i=0;i<rtcm->numObservables;i++) {
        
        if (!(sat=to_satid(sys,rtcm->observablesSet[i].svid))){
            continue;
        }
        

        if (!(sig=to_sigid(sys,rtcm->observablesSet[i].codeIndicator))){
            continue;
        }
        
        k = sat_ind[sat-1]-1;
        if ((cell=cell_ind[sig_ind[sig-1]-1+k*nsig]) >= 64){
            continue;
        }
        
        lambda = code2freq(sys,rtcm->observablesSet[i].codeIndicator,0);    /* Es frecuencia todavia. */
        if (lambda != 0.0){
            lambda = CLIGHT/lambda;                               /* Se lleva a longitud de onda. */
        }

        if (rtcm->observablesSet[i].psrngObsData == 0.0){
            psrng_s = 0;
        }
        else{
            psrng_s = rtcm->observablesSet[i].psrngObsData-rrng[k];
        }

        if ( (rtcm->observablesSet[i].phrngObsData == 0.0) || (lambda <= 0.0) ){
            phrng_s = 0;
        }
        else{
            phrng_s = rtcm->observablesSet[i].phrngObsData*lambda-rrng[k];   /* Ciclos multiplicado por lambda da en metros. */
        }

        /* subtract phase - psudorange integer cycle offset */
        if( (rtcm->observablesSet[i].codeIndicator == CODE_L1C) || (RTCM_NUMBER_OF_SIGNALS == 1) ){
            flag_ind =  RTCM_NUMBER_OF_SIGNALS*k;
        }
        else{
            flag_ind = RTCM_NUMBER_OF_SIGNALS*k + 1;
        }
        LLI = rtcm->observablesSet[i].lossOfLockIndicator;
        if ( (LLI&1) || !(*(flagFirstRtcm + flag_ind)) || (fabs(phrng_s-rtcm->observablesSet[i].carrierPhaseMeas) > DF401_RANGE) ) { /* Entra si ocurrio un cycle slip (en nuestro caso no se indica), si el valor de phrng se excede su maximo o si es la primera vez que se envia RTCM. */
            rtcm->observablesSet[i].carrierPhaseMeas = ROUND(phrng_s/lambda)*lambda;
            LLI|= 1;
            rtcm->observablesSet[i].lastLockTime = rtcm->obsTime.receiverTime/1000;
            *(flagFirstRtcm + flag_ind) = true;  /* Esto se pone en alto con la primer senial pero lo tengo que hacer para todas las seniales. */
        }

        phrng_s-= rtcm->observablesSet[i].carrierPhaseMeas;
        lt = rtcm->obsTime.receiverTime/1000 - rtcm->observablesSet[i].lastLockTime;

        if (psrng && (psrng_s != 0.0)){ /**< Se carga pseudorango*/
            psrng[cell-1] = psrng_s;
        }

        if (phrng && (phrng_s != 0.0)){ /**< Se carga fase de portadora*/
            phrng[cell-1] = phrng_s;
        }
        
        if (lock){
            lock[cell-1] = lt;
        }

        if (half){  /**< Se carga indicador de ambiguedades de medio ciclo*/
            if (rtcm->observablesSet[i].lossOfLockIndicator & 2){
                half[cell-1] = 1;
            }
            else{
                half[cell-1] = 0;
            }
        }

        if (cnr ){ /**< Se carga CN0. Se mantiene el nombre cnr pues es el que se usa en rtklib.*/
            cnr [cell-1] = (rtcm->observablesSet[i].logScaleCN0);
        }

    }
}
uint16_t encode_msm_head(uint16_t type, rtcmControl1074Type *rtcm, uint8_t sys, booleanData sync, uint8_t *nsat,
                           uint16_t *ncell, double *rrng,
                           double *psrng, double *phrng,
                           uint16_t *lock, uint8_t *half,
                           uint8_t *cnr, booleanData *flagFirstRtcm)
{
    uint8_t sat_ind[64] = {0}, sig_ind[32] = {0};
    uint16_t i = 0, j;
    uint16_t cell_ind[CPARAM_TRACKING_CHANNELS*NUM_SIGNALS_Q] = {0};
    uint8_t nsig = 0;

    /* generate msm satellite, signal and cell index */
    gen_msm_index(rtcm,SYS_GPS,nsat,&nsig,ncell,sat_ind,sig_ind,cell_ind);

    switch (type){
    case 4:
        type+= 1070;    /**< Se pasa de nro. MSM a Tipo de MSG*/
        break;
    default:
        break;
    }
    /* encode msm header (ref [15] table 3.5-78) */
    setbitu(rtcm->messageContent,i,12,type       ); i+=12; /* message number */
    setbitu(rtcm->messageContent,i,12,rtcm->stationId); i+=12; /* reference station id */
    setbitu(rtcm->messageContent,i,30,rtcm->obsTime.receiverTime      ); i+=30; /* epoch time */
    setbitu(rtcm->messageContent,i, 1,sync       ); i+= 1; /* multiple message bit - 1 indicates that more MSMs follow for given physical time and reference station ID */
    setbitu(rtcm->messageContent,i, 3,0          ); i+= 3; /* issue of data station - This field is reserved to be used to link MSM with future sitedescription (receiver, antenna description, etc.) messages. A value of “0” indicates that this field is not utilized.*/
    setbitu(rtcm->messageContent,i, 7,0          ); i+= 7; /* reserved */
    setbitu(rtcm->messageContent,i, 2,1          ); i+= 2; /* clock streering indicator - Se estan haciendo correcciones de reloj*/
    setbitu(rtcm->messageContent,i, 2,0          ); i+= 2; /* external clock indicator */
    setbitu(rtcm->messageContent,i, 1,0          ); i+= 1; /* smoothing indicator */
    setbitu(rtcm->messageContent,i, 3,0          ); i+= 3; /* smoothing interval */

    /* satellite mask */
    for (j=0;j<64;j++) {
        setbitu(rtcm->messageContent,i,1,sat_ind[j]?1:0);
        i+=1;
    }
    /* signal mask */
    for (j=0;j<32;j++) {
        setbitu(rtcm->messageContent,i,1,sig_ind[j]?1:0);
        i+=1;
    }
    /* cell mask */
    for (j=0;j<*nsat*nsig&&j<64;j++) {
        setbitu(rtcm->messageContent,i,1,cell_ind[j]?1:0);
        i+=1;
    }

    /* generate msm satellite data fields */
    gen_msm_sat(rtcm,SYS_GPS,sat_ind,rrng);
    
    /* generate msm signal data fields */
    gen_msm_sig(rtcm,SYS_GPS,nsig,*ncell,sat_ind,sig_ind,cell_ind,rrng,
                psrng,phrng,lock,half,cnr,flagFirstRtcm);

    return i;
}
/* encode rough range integer ms ---------------------------------------------*/
uint16_t encode_msm_int_rrng(rtcmControl1074Type *rtcm, uint16_t i, const double *rrng,
                               uint8_t nsat)
{
    uint32_t int_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if (rrng[j] == 0.0) {
            int_ms = 255;
        }
        else if ( (rrng[j] < 0.0) || (rrng[j] > RANGE_MS*255.0) ) {
            int_ms = 255;
        }
        else {
            int_ms = ROUND_U(rrng[j]/RANGE_MS/P2_10) >> 10;
        }
        setbitu(rtcm->messageContent,i,8,int_ms);
        i+=8;
    }
    return i;
}
/* encode rough range modulo 1 ms --------------------------------------------*/
uint16_t encode_msm_mod_rrng(rtcmControl1074Type *rtcm, uint16_t i, const double *rrng,
                               uint8_t nsat)
{
    uint32_t mod_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if ( (rrng[j] <= 0.0) || (rrng[j] > RANGE_MS*255.0) ) {
            mod_ms = 0;
        }
        else {
            mod_ms = ROUND_U(rrng[j]/RANGE_MS/P2_10) & 0x3FFu;
        }
        setbitu(rtcm->messageContent,i,10,mod_ms);
        i+=10;
    }
    return i;
}
/* encode fine pseudorange ---------------------------------------------------*/
uint16_t encode_msm_psrng(rtcmControl1074Type *rtcm, uint16_t i, const double *psrng, uint16_t ncell)
{
    int psrng_val;
    uint16_t j;
    
    for (j=0;j<ncell;j++) {
        if (psrng[j] == 0.0) {
            psrng_val=- 16384;
        }
        else if (fabs(psrng[j]) > 292.7) {
            psrng_val=- 16384;
        }
        else {
            psrng_val = ROUND(psrng[j]/RANGE_MS/P2_24);
        }
        setbits(rtcm->messageContent,i,15,psrng_val);
        i+=15;
    }
    return i;
}
/* encode fine phase-range ---------------------------------------------------*/
uint16_t encode_msm_phrng(rtcmControl1074Type *rtcm, uint16_t i, const double *phrng, uint16_t ncell)
{
    int j,phrng_val;
    
    for (j=0;j<ncell;j++) {
        if (phrng[j] == 0.0) {
            phrng_val =- 2097152;
        }
        else if (fabs(phrng[j]) > 1171.0) {
            phrng_val=- 2097152;
        }
        else {
            phrng_val = ROUND(phrng[j]/RANGE_MS/P2_29);
        }
        setbits(rtcm->messageContent,i,22,phrng_val);
        i+=22;
    }
    return i;
}
/* encode lock-time indicator ------------------------------------------------*/
uint16_t encode_msm_lock(rtcmControl1074Type *rtcm, uint16_t i, uint16_t *lock, uint16_t ncell)
{
    int j;
    uint8_t lock_val;

    for (j=0;j<ncell;j++) {
        
        lock_val = lockTime2LockTimeIndicator(lock[j]*1000);   /* La funcion necesita el parametro en ms. lock[j] esta medido en segundos. */

        setbitu(rtcm->messageContent,i,4,(uint32_t) lock_val);
        i+=4;

    }
    return i;
}
/* encode half-cycle-ambiguity indicator -------------------------------------*/
uint16_t encode_msm_half_amb(rtcmControl1074Type *rtcm, uint16_t i, const uint8_t *half,
                               uint16_t ncell)
{
    int j;
    
    for (j=0;j<ncell;j++) {
        setbitu(rtcm->messageContent,i,1,half[j]);
        i+=1;
    }
    return i;
}
/* encode signal CNR ---------------------------------------------------------*/
uint16_t encode_msm_cnr(rtcmControl1074Type *rtcm, uint16_t i, uint8_t *cnr, uint16_t ncell)
{
    int j,cnr_val;
    
    for (j=0;j<ncell;j++) {

        cnr_val = cnr[j]>>2;    /* Descarto los dos bits menos significativos que son los que me dan resolucion 0.25dBHz. */
        setbitu(rtcm->messageContent,i,6,cnr_val);
        i+=6;
    }
    return i;
}
/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : uint8_t *buff IO byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
*          [u]int32_t data  I   unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
void setbitu(uint8_t *buff, int pos, int len, uint32_t data)
{
    uint32_t mask=1u<<(len-1);
    int i;
    if ( (len <= 0) || (32<len) ){
        return;
    }
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data & mask){
            buff[i/8]|= 1u<<(7-i%8);
        }
        else{
            buff[i/8]&=~(1u<<(7-i%8));
        }
    }
    return;
}

/* set signed 38 bit field ---------------------------------------------------*/
void set38bits(uint8_t *buff, int pos, double value)
{
    int word_h = (int)floor(value/64.0);
    uint32_t word_l = (uint32_t)(value-word_h*64.0);
    setbits(buff,pos  ,32,word_h);
    setbitu(buff,pos+32,6,word_l);
}
void setbits(uint8_t *buff, int pos, int len, int32_t data)
{
    if (data<0){
        data|= 1 << (len-1);
    }
    else{
        data&=~(1<<(len-1)); /* set sign bit */
    }
    setbitu(buff,pos,len,(uint32_t)data);
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const uint16_t *ep)
{
    const uint16_t doy[] = {1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time = {0};
    uint16_t days,sec,year = (uint16_t)ep[0], mon = (uint16_t)ep[1], day = (uint16_t)ep[2];
    
    if ( (year<1970) || (2099<year) || (mon<1) ||( 12<mon) ){
        return time;
    }
    
    /* leap year if year%4==0 in 1901-2099 */
    days = (year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec = (uint16_t)floor(ep[5]);
    time.time = (time_t)days*86400 + (int)ep[3]*3600 + (int)ep[4]*60 + sec;
    time.sec = ep[5] - sec;
    return time;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double time2gpst(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(gpst0);
    time_t sec = t.time - t0.time;
    int w = (int)(sec/(86400*7));
    
    if (week){
        *week = w;
    }
    return (double)(sec-(double)w*86400*7) + t.sec;
}
errorType rtcmGpsCalculateSignalTransmissionTimes(globalRangingInfoSetType *pMeasurementSet, rtcmInternalRangingDataStruct *pGpsMeas)
{
    int32_t                             i;
    globalSpaceVehicleIdType            svid;
    double                              transmitTime;
    double                              carrierDoppler;

    /* Se copia la indicación de referencia temporal. */
    pGpsMeas->serviceId = pMeasurementSet->serviceId;
    pGpsMeas->timestamp = pMeasurementSet->timestamp;

    for (i = 0; (i < RECEIVER_RANGING_DATA_TABLES_SIZE); i++)
    {
        /* Si tengo mediciones de GPS L1 válidas para este canal */
        if((i < pMeasurementSet->itemCount) && ((receiverSignalIdIsGpsLegacy(pMeasurementSet->ranges[i].signalId)) == trueValue))
        {
            svid = pMeasurementSet->ranges[i].svid;

            /* Me aseguro de que el satélite esté healthy, sino no tiene sentido hacer nada. */
            if(svdbIsSatelliteHealthy(svid) == 0)
            {
                /* (10* hBSO + msSHB)/1000.0 + corrSCP / 2^32 /(1.023 * 10^6)  */
                transmitTime = ((double)(10*pMeasurementSet->ranges[i].halfBitsSinceOrigin + pMeasurementSet->ranges[i].msSinceHalfBit))/1000.0 +
                        ((double)(pMeasurementSet->ranges[i].corrScaleCodePhase))/(((double)(0x100000000LL))*1023000.0);

                /* ncoIncrement * FREQ_NCO_UPDATE_RATE / 2^32 */
                carrierDoppler = ((double)(pMeasurementSet->ranges[i].ncoIncrementForDeltaCarrierFrequency)) * ((double)(FREQ_NCO_UPDATE_RATE)) /
                       ((double)(0x100000000LL));

                /* Se carga la estructura de salida */
                pGpsMeas->transmitTime[i] = transmitTime;
                pGpsMeas->carrierDeviation[i] = carrierDoppler;

                pGpsMeas->dataValidL1[i]     = trueValue;
                pGpsMeas->svid[i]            = svid;
                pGpsMeas->signalId[i]        = pMeasurementSet->ranges[i].signalId;

                /* Copio también la información del observableId, que me permite verificar que efectivamente dos mediciones son consecutivas
                    * (corresponden al mismo evento de adquisición/tracking) */
                pGpsMeas->observableId[i]    = pMeasurementSet->ranges[i].virtualChannelId;

            }
            else
            {
                // commDebugConsole("RTCM GPSL1 SV UNHEALTHY, svid %d", svid);
            }
        }

#ifdef GPS_DUAL_BAND_MODE

        /* Si se utiliza la versión que incorpora mediciones L2C se calcula también el tiempo de transmisión */
        /* en base a las mismas                                                                              */

        /* Si tengo mediciones de GPS L2C válidas para este canal */
        if((i < pMeasurementSet->itemCount) && ((receiverSignalIdIsGpsL2C(pMeasurementSet->ranges[i].signalId)) == trueValue))
        {

            svid = pMeasurementSet->ranges[i].svid;

            /* Me aseguro de que el satélite esté healthy, sino no tiene sentido hacer nada. */
            if(svdbIsSatelliteHealthy(svid) == 0)
            {
                /* (10* hBSO + msSHB)/1000.0 + corrSCP / 2^32 /(1.023 * 10^6)  */
                transmitTime = ((double)(10*pMeasurementSet->ranges[i].halfBitsSinceOrigin + pMeasurementSet->ranges[i].msSinceHalfBit))/1000.0 +
                        ((1.5) * (double)(pMeasurementSet->ranges[i].corrScaleCodePhase))/(((double)(0x100000000LL))*767250.0);

                /* ncoIncrement * FREQ_NCO_UPDATE_RATE / 2^32 */
                carrierDoppler = ((double)(pMeasurementSet->ranges[i].ncoIncrementForDeltaCarrierFrequency)) * ((double)(FREQ_NCO_UPDATE_RATE)) /
                        ((double)(0x100000000LL));

                /* Se carga la estructura de salida */
                
                pGpsMeas->transmitTime[i]    = transmitTime;
                pGpsMeas->carrierDeviation[i]    = carrierDoppler;

                pGpsMeas->dataValidL2[i]     = trueValue;
                pGpsMeas->svid[i]            = svid;
                pGpsMeas->signalId[i]        = pMeasurementSet->ranges[i].signalId;

                /* Copio también la información del observableId, que me permite verificar que efectivamente dos mediciones son consecutivas
                    * (corresponden al mismo evento de adquisición/tracking) */
                pGpsMeas->observableId[i]    = pMeasurementSet->ranges[i].virtualChannelId;
            }
            else
            {
                // commDebugConsole("RTCM GPSL2 SV UNHEALTHY, svid %d", svid);
            }
        }
#endif

    }

    return SUCCESS;
}

uint8_t lockTime2LockTimeIndicator(uint32_t trackingT)
{
    if( 0      <= trackingT && trackingT < 32       ){
        return (0 );
    }
    if( 32     <= trackingT && trackingT < 64       ){
        return (1 );
    }
    if( 64     <= trackingT && trackingT < 128      ){
        return (2 );
    }
    if( 128    <= trackingT && trackingT < 256      ){
        return (3 );
    }
    if( 256    <= trackingT && trackingT < 512      ){
        return (4 );
    }
    if( 512    <= trackingT && trackingT < 1024     ){
        return (5 );
    }
    if( 1024   <= trackingT && trackingT < 2048     ){
        return (6 );
    }
    if( 2048   <= trackingT && trackingT < 4096     ){
        return (7 );
    }
    if( 4096   <= trackingT && trackingT < 8192     ){
        return (8 );
    }
    if( 8192   <= trackingT && trackingT < 16384    ){
        return (9 );
    }
    if( 16384  <= trackingT && trackingT < 32768    ){
        return (10);
    }
    if( 32768  <= trackingT && trackingT < 65536    ){
        return (11);
    }
    if( 65536  <= trackingT && trackingT < 131072   ){
        return (12);
    }
    if( 131072 <= trackingT && trackingT < 262144   ){
        return (13);
    }
    if( 262144 <= trackingT && trackingT < 524288   ){
        return (14);
    }
    if( 524288 <= trackingT                         ){
        return (15);
    }
    return 16; //No va a llegar nunca hasta aca.

}
/* GPS obs code to frequency -------------------------------------------------*/
int code2freq_GPS(uint8_t code, double *freq)
{
    char *obs = code2obs(code);
    
    switch (obs[0]) {
        case '1':
            *freq = FREQ1;
            return 0; /* L1 */
            break;
        case '2':
            *freq = FREQ2;
            return 1; /* L2 */
            break;
    }
    return -1;
}
/* system and obs code to frequency --------------------------------------------
* convert system and obs code to carrier frequency
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
*          int    fcn       I   frequency channel number for GLONASS
* return : carrier frequency (Hz) (0.0: error)
*-----------------------------------------------------------------------------*/
double code2freq(int sys, uint8_t code, int fcn)
{
    double freq = 0.0;
    
    switch (sys) {
        case SYS_GPS:
            (void)code2freq_GPS(code,&freq);
            break;
#ifndef GPS_ONLY
        case SYS_GLO:
            (void)code2freq_GLO(code,fcn,&freq);
            break;
        case SYS_GAL:
            (void)code2freq_GAL(code,&freq);
            break;
        case SYS_QZS:
            (void)code2freq_QZS(code,&freq);
            break;
        case SYS_SBS:
            (void)code2freq_SBS(code,&freq);
            break;
        case SYS_CMP:
            (void)code2freq_BDS(code,&freq);
            break;
        case SYS_IRN:
            (void)code2freq_IRN(code,&freq);
            break;
#endif
    }
    return freq;
}