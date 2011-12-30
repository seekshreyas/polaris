from math import *

'''
Alternate algorithms (VGO, DCM, EKF Euler angles versus quaternions) could provide a gut check / fall-back option as a fail-safe...

This didn't get very far.

DCM - Bill Premerlani & Paul Bizard / DIY Drones
Inputs:
    gyros (rad/s)
    accelerometers (m/s^2)
    GPS vx vy horizontal speed (m/s)
    GPS course over ground (rad)
    GPS speeed Ve (m/s)
Outputs
    y0[0 1 2] euler angels (rad)
    y1[0 1 2]
    y2[0 1 2]
    y3[0 1 2]
'''

class DCM:

    def __init__(self):
        self.psi = 0.0
        self.theta = 0.023865
        self.phi = 0.0
        psi = self.psi
        theta = self.theta
        phi = self.phi
        self.R[0,0] = cos(theta) * cos(psi)
        self.R[0,1] = cos(theta) * sin(psi)
        self.R[0,2] = -sin(theta)
        self.R[1,0] = (sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi))
        self.R[1,1] = (sin(phi) *sin(theta) * sin(psi)) + (cos(phi) * cos(psi))
        self.R[1,2] = sin(phi) * cos(theta)
        self.R[2,0] = (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi))
        self.R[2,1] = (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))
        self.R[2,2] = cos(phi) * cos(theta)
        self.rup = matrix("1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0")
        self.integral = matrix("0.0 0.0 0.0")
        self.Rfilter0 = R[0,0]
        self.Rfilter3 = R[1,0]
        self.yaw_flag = 0.0
        self.GPSspeed = matrix("26.66 26.66")
        self.omega = matrix("0.0 0.0 0.0")

    def model_outputs(self):
        self.psi = atan2(self.R[1,0],self.R[0,0])
        self.theta = -asin(self.R[2,0])
        self.phi = atan2(self.R[2,1],self.R[2,2])
        return {"psi": self.psi,
                "theta": self.theta,
                "phi": self.phi,}

    def model_update(self, dt):
        omega_correction = matrix("%f %f %f" % omega_correction_p_acc, omega_correction_p_gps, omega_correction_integral)
        self.omega += omega_correction
        self.theta_hat = omega * dt
        self.rup[0,0]
        

# /*============================================================================*/
# /* SFUNCTION NUMERICAL CONTROLLER                                             */
# /*============================================================================*/
# /* Authors: Bill PREMERLANI & Paul BIZARD / DIY Drones                        */
# /* Date   : February 2009                                                     */
# /*============================================================================*/
# /* INPUTS                                                                     */
# /* ------                                                                     */
# /* u0[0 1 2] Gyrometers p q r             rad/s                               */
# /* u1[0 1 2] Accelerometers x y z         m/s^2                               */
# /* u2[0 1]   GPS vx vy horizontal speed   m/s                                 */
# /* u3[0]     GPS Khi course over ground   rad                                 */
# /* u4[0]     GPS speed Ve                 m/s                                 */
# /*============================================================================*/
# /* OUTPUTS                                                                    */
# /* -------                                                                    */
# /* y0[0 1 2] Euler angles           rad                                       */
# /* y1[0 1 2]                                                                  */
# /* y2[0 1 2]                                                                  */
# /* y3[0 1 2]                                                                  */
# /*============================================================================*/
# /* DCM matrix estimation                                                      */
# /* R marix = transformation from body frame to earth frame                    */
# /*                                                                            */
# /* 1st column is x body vector expressed in earth frame                       */
# /* 2nd...        y                                                            */
# /* 3rd...        z                                                            */
# /*                                                                            */
# /* 1st row is x earth vector expressed in body frame                          */
# /* 2nd row    y                                                               */
# /* 3rd row    z                                                               */
# /*                                                                            */
# /* x front                                                                    */
# /* y right wing                                                               */
# /* z down                                                                     */
# /*============================================================================*/

# #define S_FUNCTION_NAME       R
# #define S_FUNCTION_LEVEL      2
# #define NUM_INPUTS            5
# #define INPUT_0_WIDTH         3
# #define INPUT_1_WIDTH         3
# #define INPUT_2_WIDTH         2
# #define INPUT_3_WIDTH         1
# #define INPUT_4_WIDTH         1
# #define INPUT_0_FEEDTHROUGH   1
# #define INPUT_1_FEEDTHROUGH   1
# #define INPUT_2_FEEDTHROUGH   1
# #define INPUT_3_FEEDTHROUGH   1
# #define INPUT_4_FEEDTHROUGH   1
# #define NUM_OUTPUTS           6
# #define OUTPUT_0_WIDTH        3
# #define OUTPUT_1_WIDTH        3
# #define OUTPUT_2_WIDTH        3
# #define OUTPUT_3_WIDTH        3
# #define OUTPUT_4_WIDTH        3
# #define OUTPUT_5_WIDTH        3
# #define NPARAMS               0
# #define SAMPLE_TIME_0         0.025
# #define SAMPLE_TIME_1         1.  
# #define NUM_DISC_STATES       0
# #define DISC_STATES_IC        [0]
# #define NUM_CONT_STATES       0
# #define CONT_STATES_IC        [0]
# #define SFUNWIZ_GENERATE_TLC  1
# #define SOURCEFILES           ""
# #define PANELINDEX            5
# #define SFUNWIZ_REVISION      1.0
# 
# #define DT25          0.025     // 25ms sample time
# #define GPSFILT       0.012422  // x body filter coefficient = 1-exp(-DT25/2s)
# 
# #define KPPitchRoll   0.012   // Pitch-Roll correction proportional gain
# #define KIPitchRoll   0.00002 // Pitch-Roll correction integral gain
# #define KPYaw         0.3 // Yaw correction proportional gain
# #define KIYaw         0.0001  // Yaw correction integral gain
# 
# #define R2D           57.295779 // Radians to degrees
# #define D2R           0.0174532 // Degrees to radians
# 
# #define PARAM_DEF0(S) ssGetSFcnParam(S, 0)
# 
# #define MDL_CHECK_PARAMETERS
# #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
# /*----------------------------------------------------------------------------*/
# static void mdlCheckParameters(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
#    unsigned short int  i;
#    bool                validParam = false;
# 
# 
#    /* All parameters must be scalar */
#    for (i=0;i<ssGetSFcnParamsCount(S);i++)
#    {
#       const mxArray *pVal = ssGetSFcnParam(S,i);
# 
#       if (!mxIsNumeric(pVal) || !mxIsDouble(pVal) ||  mxIsLogical(pVal)
#        ||  mxIsComplex(pVal) ||  mxIsSparse(pVal) 
#        || !mxIsFinite(mxGetPr(pVal)[0]))
#       {
#          validParam = true;
#          break;
#       }
#    }
# 
#    if (validParam) 
#    {
#       ssSetErrorStatus(S,"All parameters must be a scalar or vectors");     
#       return;
#    }
# 
#    /* All parameters are not tunable */
#    for (i=0;i<ssGetSFcnParamsCount(S);i++)
#    {
#       ssSetSFcnParamTunable(S, i, 0);
#    }
# }
# #endif
# 
# /*----------------------------------------------------------------------------*/
# static void mdlInitializeSizes(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
#    ssSetNumSFcnParams(S, NPARAMS);           /* Number of expected parameters */
# #if defined(MATLAB_MEX_FILE)
#    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
#    {
#       mdlCheckParameters(S);
#       if (ssGetErrorStatus(S) != NULL) return;
#    }
#    /* Parameter mismatch will be reported by Simulink */
#    else  return;
# #endif
#     
#    ssSetNumContStates(S, NUM_CONT_STATES);
#    ssSetNumDiscStates(S, NUM_DISC_STATES);
# 
#    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
# 
#    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
#    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
#    ssSetInputPortRequiredContiguous(S, 0, 1);
# 
#    ssSetInputPortWidth(S, 1, INPUT_1_WIDTH);
#    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
#    ssSetInputPortRequiredContiguous(S, 1, 1);
# 
#    ssSetInputPortWidth(S, 2, INPUT_2_WIDTH);
#    ssSetInputPortDirectFeedThrough(S, 2, INPUT_2_FEEDTHROUGH);
#    ssSetInputPortRequiredContiguous(S, 2, 1);
# 
#    ssSetInputPortWidth(S, 3, INPUT_3_WIDTH);
#    ssSetInputPortDirectFeedThrough(S, 3, INPUT_3_FEEDTHROUGH);
#    ssSetInputPortRequiredContiguous(S, 3, 1);
# 
#    ssSetInputPortWidth(S, 4, INPUT_4_WIDTH);
#    ssSetInputPortDirectFeedThrough(S, 4, INPUT_4_FEEDTHROUGH);
#    ssSetInputPortRequiredContiguous(S, 4, 1);
# 
#    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
#    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
#    ssSetOutputPortWidth(S, 1, OUTPUT_1_WIDTH);
#    ssSetOutputPortWidth(S, 2, OUTPUT_2_WIDTH);
#    ssSetOutputPortWidth(S, 3, OUTPUT_3_WIDTH);
#    ssSetOutputPortWidth(S, 4, OUTPUT_4_WIDTH);
#    ssSetOutputPortWidth(S, 5, OUTPUT_5_WIDTH);
# 
#    ssSetNumSampleTimes(S, 2);
#    ssSetNumRWork(S, 0);
#    ssSetNumIWork(S, 0);
#    ssSetNumPWork(S, 17);
#    ssSetNumModes(S, 0);
#    ssSetNumNonsampledZCs(S, 0);
# 
#    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
#    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE
#                  | SS_OPTION_DISCRETE_VALUED_OUTPUT);
# }
# 
# /*----------------------------------------------------------------------------*/
# static void mdlInitializeSampleTimes(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
#    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
#    ssSetOffsetTime(S, 0, 0.0);
#    ssSetSampleTime(S, 1, SAMPLE_TIME_1);
#    ssSetOffsetTime(S, 1, 0.0);
# }
# 
# #define MDL_START
# #if defined(MDL_START)
# /*----------------------------------------------------------------------------*/
# static void mdlStart(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    /* ============ */
#    float  *rmat;
#    float  *rup;
#    float  *errorRP;
#    float  *omegacorrPAcc;
#    float  *omegacorrInt;
#    float  *Integral;
#    float  *rmat0filt;
#    float  *rmat3filt;
#    float  *dirovergndHRmat;
#    float  *dirovergndHGPS;
#    float  *errorYawground;
#    float  *errorYawplane;
#    float  *omegacorrPGPS;
#    float  *sogGPS;
#    float  *speedGPS;
#    float  *accelGPS;
#    int    *flags;
# 
# 
#    /* Allocate memory to arrays */
#    /* ========================= */
#    rmat            = (float *) calloc(9, sizeof(float));
#    rup             = (float *) calloc(9, sizeof(float));
#    errorRP         = (float *) calloc(3, sizeof(float));
#    omegacorrPAcc   = (float *) calloc(3, sizeof(float));
#    omegacorrInt    = (float *) calloc(3, sizeof(float));
#    Integral        = (float *) calloc(3, sizeof(float));
#    rmat0filt       = (float *) calloc(1, sizeof(float));
#    rmat3filt       = (float *) calloc(1, sizeof(float));
#    dirovergndHRmat = (float *) calloc(3, sizeof(float));
#    dirovergndHGPS  = (float *) calloc(3, sizeof(float));
#    errorYawground  = (float *) calloc(3, sizeof(float));
#    errorYawplane   = (float *) calloc(3, sizeof(float));
#    omegacorrPGPS   = (float *) calloc(3, sizeof(float));
#    sogGPS          = (float *) calloc(1, sizeof(float));
#    speedGPS        = (float *) calloc(2, sizeof(float));
#    accelGPS        = (float *) calloc(1, sizeof(float));
#    flags           = (int *)   calloc(1, sizeof(int));
# 
# 
#    /* Attach pointers as user data */
#    /* ============================ */
#    ssGetPWork(S)[0]  = rmat;
#    ssGetPWork(S)[1]  = rup;
#    ssGetPWork(S)[2]  = errorRP;
#    ssGetPWork(S)[3]  = omegacorrPAcc;
#    ssGetPWork(S)[4]  = omegacorrInt;
#    ssGetPWork(S)[5]  = Integral;
#    ssGetPWork(S)[6]  = rmat0filt;
#    ssGetPWork(S)[7]  = rmat3filt;
#    ssGetPWork(S)[8]  = dirovergndHRmat;
#    ssGetPWork(S)[9]  = dirovergndHGPS;
#    ssGetPWork(S)[10] = errorYawground;
#    ssGetPWork(S)[11] = errorYawplane;
#    ssGetPWork(S)[12] = omegacorrPGPS;
#    ssGetPWork(S)[13] = sogGPS;
#    ssGetPWork(S)[14] = speedGPS;
#    ssGetPWork(S)[15] = accelGPS;
#    ssGetPWork(S)[16] = flags;
# }
# #endif
# 
# #define MDL_INITIALIZE_CONDITIONS
# #if defined(MDL_INITIALIZE_CONDITIONS)
# /*----------------------------------------------------------------------------*/
# static void mdlInitializeConditions(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    /* ============ */
#    float  *rmat      = ssGetPWork(S)[0];
#    float  *rup       = ssGetPWork(S)[1];
#    float  *Integral  = ssGetPWork(S)[5];
#    float  *rmat0filt = ssGetPWork(S)[6];
#    float  *rmat3filt = ssGetPWork(S)[7];
#    float  *speedGPS  = ssGetPWork(S)[14];
#    int    *flags     = ssGetPWork(S)[16];
# 
#    float  Yaw, Pitch, Roll;
#    int    i;
# 
# 
#    /* Initial Euler angles */
#    /* ==================== */
#    Yaw   = 0. * D2R;
#    Pitch = 0.023865;
#    Roll  = 0. * D2R;
# 
# 
#    /* Initialize DCM */
#    /* ============== */
#    rmat[0] =  cos(Yaw)*cos(Pitch);
#    rmat[1] = -sin(Yaw)*cos(Roll) + cos(Yaw)*sin(Pitch)*sin(Roll);
#    rmat[2] =  sin(Yaw)*sin(Roll) + cos(Yaw)*sin(Pitch)*cos(Roll);
#    rmat[3] =  sin(Yaw)*cos(Pitch);
#    rmat[4] =  cos(Yaw)*cos(Roll) + sin(Yaw)*sin(Pitch)*sin(Roll);
#    rmat[5] = -cos(Yaw)*sin(Roll) + sin(Yaw)*sin(Pitch)*cos(Roll);
#    rmat[6] = -sin(Pitch);
#    rmat[7] =  cos(Pitch)*sin(Roll);
#    rmat[8] =  cos(Pitch)*cos(Roll);
# 
# 
#    /* Update matrix rup = identity */
#    /* ============================ */
#    for (i=0;i<9;i++) rup[i] = 0.;
#    rup[0] = 1.; rup[4] = 1.; rup[8] = 1.;
# 
# 
#    /* Integrator state */
#    /* ================ */
#    for (i=0;i<3;i++) Integral[i] = 0.;
# 
# 
#    /* X body over ground filter */
#    /* ========================= */
#    rmat0filt[0] = rmat[0];
#    rmat3filt[0] = rmat[3];
# 
# 
#    /* Yaw flag */
#    /* ======== */
#    flags[0] = 0;
# 
# 
#    /* GPS speed */
#    /* ========= */
#    speedGPS[0] = 26.66;
#    speedGPS[1] = 26.66;
# }
# #endif
# 
# #define MDL_OUTPUTS
# #if defined(MDL_OUTPUTS)
# /*----------------------------------------------------------------------------*/
# static void mdlOutputs(SimStruct *S, int_T tid)
# /*----------------------------------------------------------------------------*/
# {
#    real_T *y0 = ssGetOutputPortRealSignal(S,0);
#    real_T *y1 = ssGetOutputPortRealSignal(S,1);
#    real_T *y2 = ssGetOutputPortRealSignal(S,2);
#    real_T *y3 = ssGetOutputPortRealSignal(S,3);
#    real_T *y4 = ssGetOutputPortRealSignal(S,4);
#    real_T *y5 = ssGetOutputPortRealSignal(S,5);
# 
#    float  *rmat          = ssGetPWork(S)[0];
#    float  *omegacorrPAcc = ssGetPWork(S)[3];
#    float  *omegacorrInt  = ssGetPWork(S)[4];
#    float  *omegacorrPGPS = ssGetPWork(S)[12];
#    float  *speedGPS      = ssGetPWork(S)[14];
#    float  *accelGPS      = ssGetPWork(S)[15];
# 
#    float  Yaw, Pitch, Roll;
# 
# 
#    /* Euler angles from DCM */
#    /* ===================== */
#    /* atan2(rmat31,rmat11) */
#    Yaw = atan2(rmat[3],rmat[0]);
# 
#    /* -asin(rmat31) */
#    Pitch = -asin(rmat[6]);   
# 
#    /* atan2(rmat32,rmat33) */
#    Roll = atan2(rmat[7],rmat[8]);
# 
# 
#    /* Outputs */
#    /* ======= */
#    /* Euler angles */
#    y0[0] = Roll; 
#    y0[1] = Pitch; 
#    y0[2] = Yaw; 
# 
#    /* Roll-Pitch correction proportional term */
#    y1[0] = omegacorrPAcc[0]; 
#    y1[1] = omegacorrPAcc[1];
#    y1[2] = omegacorrPAcc[2];
# 
#    /* Yaw correction proportional term */
#    y2[0] = omegacorrPGPS[0]; 
#    y2[1] = omegacorrPGPS[1]; 
#    y2[2] = omegacorrPGPS[2]; 
# 
#    /* Roll-Pitch + Yaw integral term */
#    y3[0] = omegacorrInt[0]; 
#    y3[1] = omegacorrInt[1]; 
#    y3[2] = omegacorrInt[2]; 
# 
#    /* Debug 1 */
#    y4[0] = 0.; 
#    y4[1] = 0.;
#    y4[2] = 0.;
# 
#    /* Debug 2 */
#    y5[0] = 0.; 
#    y5[1] = 0.;
#    y5[2] = 0.;
# }
# #endif
# 
# #define MDL_UPDATE
# #if defined(MDL_UPDATE)
# /*----------------------------------------------------------------------------*/
# static void mdlUpdate(SimStruct *S, int_T tid)
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    /* ============ */
#    real_T *u0 = (const real_T *) ssGetInputPortSignal(S,0);
#    real_T *u1 = (const real_T *) ssGetInputPortSignal(S,1);
#    real_T *u2 = (const real_T *) ssGetInputPortSignal(S,2);
#    real_T *u3 = (const real_T *) ssGetInputPortSignal(S,3);
#    real_T *u4 = (const real_T *) ssGetInputPortSignal(S,4);
# 
#    float  *rmat            = ssGetPWork(S)[0];
#    float  *rup             = ssGetPWork(S)[1];
#    float  *errorRP         = ssGetPWork(S)[2];
#    float  *omegacorrPAcc   = ssGetPWork(S)[3];
#    float  *omegacorrInt    = ssGetPWork(S)[4];
#    float  *Integral        = ssGetPWork(S)[5];
#    float  *rmat0filt       = ssGetPWork(S)[6];
#    float  *rmat3filt       = ssGetPWork(S)[7];
#    float  *dirovergndHRmat = ssGetPWork(S)[8];
#    float  *dirovergndHGPS  = ssGetPWork(S)[9];
#    float  *errorYawground  = ssGetPWork(S)[10];
#    float  *errorYawplane   = ssGetPWork(S)[11];
#    float  *omegacorrPGPS   = ssGetPWork(S)[12];
#    float  *sogGPS          = ssGetPWork(S)[13];
#    float  *speedGPS        = ssGetPWork(S)[14];
#    float  *accelGPS        = ssGetPWork(S)[15];
#    int    *flags           = ssGetPWork(S)[16];
# 
#    float  omegatotal[3], theta[3];
#    float  V_buff[3];
#    float  r_buff[9];
#    float  f_buff;
# 
#    int    i;
# 
#    FILE*  deb_file_ptr;
# 
# 
# /*----------------------------------------------------------------------------*/
# //   /* Open debug file */
# //   deb_file_ptr = fopen ("deb_file.txt", "a");
# /*----------------------------------------------------------------------------*/
# 
# /*============================================================================*/
# /*============================================================================*/
# /*                                25ms routine                                */
# /*============================================================================*/
# /*============================================================================*/
# 
#    /* If 25ms sample time */
#    if (ssIsSampleHit(S,0,tid))
#    {
#       /* R matrix update */
#       /* =============== */
#       /* =============== */
#       /* Read body angular velocity (p q r gyrometers) */
#       for (i=0;i<3;i++) omegatotal[i] = u0[i];
# 
#       /* Add pitch-roll and yaw corrections */
#       VectorAdd (omegatotal, omegacorrPAcc, omegatotal);
#       VectorAdd (omegatotal, omegacorrPGPS, omegatotal);
#       VectorAdd (omegatotal, omegacorrInt,  omegatotal);
# 
#       /* Integrate angular velocity over the 25ms time step */
#       for (i=0;i<3;i++) theta[i] = omegatotal[i] * DT25;
# 
#       /* Assemble equivalent small rotation matrix (update matrix) */
#       rup[1] = -theta[2];
#       rup[2] =  theta[1];
#       rup[3] =  theta[2];
#       rup[5] = -theta[0];
#       rup[6] = -theta[1];
#       rup[7] =  theta[0];
# 
#       /* Rotate body frame */
#       MatrixMultiply (rmat, rup);
# 
# 
#       /* Roll-Pitch correction */
#       /* ===================== */
#       /* ===================== */
#       /* Read body acceleration (x y z accelerometers) */
#       for (i=0;i<3;i++) V_buff[i] = u1[i];
# 
#       /* Correct body acceleration for centrifugal effect */
#       V_buff[0] += accelGPS[0];      
#       V_buff[1] += (u0[2]+omegacorrInt[2])*speedGPS[0];      
#       V_buff[2] -= (u0[1]+omegacorrInt[1])*speedGPS[0];      
# 
#       /* Compute the roll-pitch error vector: cross product of measured */
#       /* earth Z vector with estimated earth vector expressed in body   */
#       /* frame (3rd row of rmat)                                        */
#       VectorCross(V_buff, &rmat[6], errorRP);
# 
#       /* Compute pitch-roll correction proportional term */
#       for (i=0;i<3;i++) omegacorrPAcc[i] = errorRP[i] * KPPitchRoll;
# 
#       /* Add pitch-roll error to integrator */
#       for (i=0;i<3;i++) Integral[i] += errorRP[i] * KIPitchRoll;
# 
# 
#       /* Yaw correction part 2 */
#       /* ===================== */
#       /* ===================== */
#       /* Filter with a 1st order the x body vector over ground */ 
#       rmat0filt[0] += GPSFILT * (rmat[0] - rmat0filt[0]);
#       rmat3filt[0] += GPSFILT * (rmat[3] - rmat3filt[0]);
# 
# 
#       /* If direction over ground has been updated */
#       if (flags[0] == 1)
#       {
#          /* Compute the yaw error vector expressed in earth frame */
#          VectorCross(dirovergndHRmat, dirovergndHGPS, errorYawground);
# 
#          /* Express the yaw error vector in the body frame */
#          TransposeMatrix(rmat, r_buff);
#          MatrixVector(r_buff, errorYawground, errorYawplane);
# 
#          /* Compute yaw correction proportional term */
#          for (i=0;i<3;i++) omegacorrPGPS[i] = errorYawplane[i] * KPYaw;
# 
#          /* Reset yaw correction flag */
#          flags[0] = 0;
#       }
# 
#       /* Add yaw error to integrator */
#       for (i=0;i<3;i++) Integral[i] += errorYawplane[i] * KIYaw;
# 
#       /* Compute integral term */
#       for (i=0;i<3;i++) omegacorrInt[i] = Integral[i];
#    }
# 
# /*============================================================================*/
# /*============================================================================*/
# /*                                1s routine                                  */
# /*============================================================================*/
# /*============================================================================*/
# 
#    /* If 1s sample time */
#    if (ssIsSampleHit(S,1,tid))
#    {
#       /* Orthogonalization */
#       /* ================= */
#       /* ================= */
#       /* (U,V) */
#       VectorDotProduct(&rmat[0], &rmat[3], &f_buff);
#       f_buff /= 2.;
# 
#       /* U = U - 0.5*V(U,V) */
#       for (i=0;i<3;i++) V_buff[i] = rmat[i];
#       for (i=0;i<3;i++) rmat[i]   -= rmat[3+i]*f_buff;
# 
#       /* V = V - 0.5*U(U,V) */
#       for (i=0;i<3;i++) rmat[3+i] -= V_buff[i]*f_buff;
# 
#       /* W = UxV */
#       VectorCross(&rmat[0], &rmat[3], &rmat[6]);
# 
#       /* U scaling */
#       VectorDotProduct(&rmat[0], &rmat[0], &f_buff);
#       f_buff = 1./sqrt(f_buff);
#       for (i=0;i<3;i++) rmat[i] = rmat[i]*f_buff;
# 
#       /* V scaling */
#       VectorDotProduct(&rmat[3], &rmat[3], &f_buff);
#       f_buff = 1./sqrt(f_buff);
#       for (i=0;i<3;i++) rmat[3+i] = rmat[3+i]*f_buff;
# 
#       /* W scaling */
#       VectorDotProduct(&rmat[6], &rmat[6], &f_buff);
#       f_buff = 1./sqrt(f_buff);
#       for (i=0;i<3;i++) rmat[6+i] = rmat[6+i]*f_buff;
# 
# 
#       /* Yaw correction part 1 */
#       /* ===================== */
#       /* ===================== */
#       /* Filtered x body over ground (projection of x body on earth xy plane) */
#       dirovergndHRmat[0] = rmat0filt[0];
#       dirovergndHRmat[1] = rmat3filt[0];
#       dirovergndHRmat[2] = 0.;
# 
#       /* Normalized GPS speed over ground vector */
#       dirovergndHGPS[0] = cos(u3[0]);
#       dirovergndHGPS[1] = sin(u3[0]);
#       dirovergndHGPS[2] = 0.;
# 
#       /* Set yaw correction flag */
#       flags[0] = 1;
# 
# 
#       /* SOG from GPS */
#       /* ============ */
#       /* ============ */
#       /* Read speed from GPS */
#       speedGPS[1] = speedGPS[0];
#       speedGPS[0] = u4[0];
#       accelGPS[0] = speedGPS[0] - speedGPS[1];
#    }
# 
# /*----------------------------------------------------------------------------*/
# //   /* Close debug file */
# //   fclose (deb_file_ptr);
# /*----------------------------------------------------------------------------*/
# } 
# #endif
# 
# /*----------------------------------------------------------------------------*/
# static void MatrixMultiply(float *M1, float *M2)
# /*----------------------------------------------------------------------------*/
# /* M1 = M1*M2                                                                 */
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    float  M_buff[9];
#    int    i;
# 
# 
#    for (i=0;i<9;i++) M_buff[i] = M1[i];
# 
#    M1[0] = M_buff[0]*M2[0] + M_buff[1]*M2[3] + M_buff[2]*M2[6];    
#    M1[3] = M_buff[3]*M2[0] + M_buff[4]*M2[3] + M_buff[5]*M2[6];    
#    M1[6] = M_buff[6]*M2[0] + M_buff[7]*M2[3] + M_buff[8]*M2[6];    
# 
#    M1[1] = M_buff[0]*M2[1] + M_buff[1]*M2[4] + M_buff[2]*M2[7];    
#    M1[4] = M_buff[3]*M2[1] + M_buff[4]*M2[4] + M_buff[5]*M2[7];    
#    M1[7] = M_buff[6]*M2[1] + M_buff[7]*M2[4] + M_buff[8]*M2[7];    
# 
#    M1[2] = M_buff[0]*M2[2] + M_buff[1]*M2[5] + M_buff[2]*M2[8];    
#    M1[5] = M_buff[3]*M2[2] + M_buff[4]*M2[5] + M_buff[5]*M2[8];    
#    M1[8] = M_buff[6]*M2[2] + M_buff[7]*M2[5] + M_buff[8]*M2[8];    
# }
# 
# /*----------------------------------------------------------------------------*/
# static void TransposeMatrix(float *M1, float *M2)
# /*----------------------------------------------------------------------------*/
# /* M2 = Transpose M1                                                          */
# /*----------------------------------------------------------------------------*/
# {
#    M2[0] = M1[0]; M2[1] = M1[3]; M2[2] = M1[6];
#    M2[3] = M1[1]; M2[4] = M1[4]; M2[5] = M1[7];
#    M2[6] = M1[2]; M2[7] = M1[5]; M2[8] = M1[8];
# }
# 
# /*----------------------------------------------------------------------------*/
# static void MatrixVector(float *M, float *V1, float *V2)
# /*----------------------------------------------------------------------------*/
# /* V2 = M*V1                                                                  */
# /*----------------------------------------------------------------------------*/
# {
#    V2[0] = M[0]*V1[0] + M[1]*V1[1] + M[2]*V1[2];
#    V2[1] = M[3]*V1[0] + M[4]*V1[1] + M[5]*V1[2];
#    V2[2] = M[6]*V1[0] + M[7]*V1[1] + M[8]*V1[2];
# }
# 
# 
# /*----------------------------------------------------------------------------*/
# static void VectorDotProduct(float *V1, float *V2, float *Scalar_Product)
# /*----------------------------------------------------------------------------*/
# /* Scalar_Product = V1.V2                                                     */
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    int    i;
# 
# 
#    *Scalar_Product = 0.;
#    for (i=0;i<3;i++) *Scalar_Product += V1[i]*V2[i];
# }
# 
# /*----------------------------------------------------------------------------*/
# static void VectorCross(float *V1, float *V2, float *V3)
# /*----------------------------------------------------------------------------*/
# /* V3 = V1xV2                                                                 */
# /*----------------------------------------------------------------------------*/
# {
#    V3[0] = V1[1]*V2[2] - V1[2]*V2[1];
#    V3[1] = V1[2]*V2[0] - V1[0]*V2[2];
#    V3[2] = V1[0]*V2[1] - V1[1]*V2[0];
# }
# 
# /*----------------------------------------------------------------------------*/
# static void VectorAdd(float *V1, float *V2, float *V3)
# /*----------------------------------------------------------------------------*/
# /* V3 = V1+V2                                                                 */
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    int    i;
# 
# 
#    for (i=0;i<3;i++) V3[i] = V1[i]+V2[i];
# }
# 
# /*----------------------------------------------------------------------------*/
# static void VectorScale(float *V1, float *K, float *V2)
# /*----------------------------------------------------------------------------*/
# /* V3 = V1+V2                                                                 */
# /*----------------------------------------------------------------------------*/
# {
#    /* Declarations */
#    int    i;
# 
# 
#    for (i=0;i<3;i++) V2[i] = V1[i] * (*K);
# }
# #undef MDL_DERIVATIVES
# #if defined(MDL_DERIVATIVES)
# /*----------------------------------------------------------------------------*/
# static void mdlDerivatives(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
# }
# #endif
# 
# #if defined(MATLAB_MEX_FILE) || defined(NRT)
# #undef MDL_ZERO_CROSSINGS
#   #if defined(MDL_DERIVATIVES)
# /*----------------------------------------------------------------------------*/
# static void mdlZeroCrossings(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
# }
#   #endif
# #endif
# 
# /*----------------------------------------------------------------------------*/
# static void mdlTerminate(SimStruct *S)
# /*----------------------------------------------------------------------------*/
# {
# }
# 
# #ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#   #include "simulink.c"    /* MEX-file interface mechanism */
# #else
#   #include "cg_sfun.h"     /* Code generation registration function */
# #endif
