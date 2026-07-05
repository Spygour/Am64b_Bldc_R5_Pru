/* Includes */
#include "CurrentCtlr.h"
#include "../Ads8688/Ads8688.h"
#include "../Igd/Igd.h"
#include "../PruDriver/PruDriver.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <math.h>
#include <string.h>

/* Definitions */
#define V_OFFSET_CALC_PHASE_MAX_CNT 400
#define R_CALC_PHASE_MAX_CNT 400

#define MAX_DUTY 120000

#define SQRT_3 1.73205

#define PHASE_U 0
#define PHASE_V 1
#define PHASE_W 2

#define THETA_FILTER_A 0.01

#define DEADTIME 72 /* DEADTIME 300 ns */

#define MAX_VOLTAGE (float)11

#define Kp 0.4
#define Ki 3

/* Types */
typedef enum {
  V_OFFSET_CALC_PHASE,
  R_CALC_PHASE,
  PI_CONTROL_PHASE
} CurrentCtlr_State_t;

/* static Variables */
static CurrentCtlr_MainData_t CurrentCtlr_MainData = {
    .theta = 0,
    .R = 0,
    .L = 0,
    .I = {0, 0, 0},
    .V = {0, 0, 0},
    .clarke = {.Ia = 0, .Ia_prev = 0, .Ib = 0, .Ib_prev = 0, .Va = 0, .Vb = 0},
    .park = {.Id = 0, .Iq = 0, .Vd = 0, .Vq = 0}};

static float CurrentCtlr_VrefOffset[ADS8688_CHANNELS];
static uint32_t CurrentCtlr_IsrCnt = 0;

static uint32_t CurrentCtrl_FreewheelTimeFirst = 0;

static uint32_t CurrentCtlr_Duty[CURRENTCTLR_PHASES] = {0, 0, 0};

static CurrentCtlr_State_t CurrentCtlr_State = V_OFFSET_CALC_PHASE;

/* Local functions */
static void CurrentCtlr_ClarkeTransform(void) {
  CurrentCtlr_MainData.clarke.Ia = CurrentCtlr_MainData.I[PHASE_U];
  CurrentCtlr_MainData.clarke.Ib =
      (CurrentCtlr_MainData.I[PHASE_U] + 2 * CurrentCtlr_MainData.I[PHASE_V]) /
      SQRT_3;
}

static void CurrentCtlr_CalcTheta(void) {
  float Ea = 0;
  float Eb = 0;
  /* Calculate the Ea */
  Ea = CurrentCtlr_MainData.clarke.Va -
       CurrentCtlr_MainData.R * CurrentCtlr_MainData.clarke.Ia;
  Ea -=
      (CurrentCtlr_MainData.L *
       (CurrentCtlr_MainData.clarke.Ia - CurrentCtlr_MainData.clarke.Ia_prev) *
       Ads8688_TimingData.DeltaTime);
  /* Calculate the Eb */
  Eb = CurrentCtlr_MainData.clarke.Vb -
       CurrentCtlr_MainData.R * CurrentCtlr_MainData.clarke.Ib;
  Eb -=
      (CurrentCtlr_MainData.L *
       (CurrentCtlr_MainData.clarke.Ib - CurrentCtlr_MainData.clarke.Ib_prev) *
       Ads8688_TimingData.DeltaTime);

  /* Filter to avoid too much noise on the theta */
  CurrentCtlr_MainData.theta =
      (1 - THETA_FILTER_A) * CurrentCtlr_MainData.theta +
      THETA_FILTER_A * atan2f(Eb, Ea);

  /* Update the previous values with the current */
  CurrentCtlr_MainData.clarke.Ia_prev = CurrentCtlr_MainData.clarke.Ia;
  CurrentCtlr_MainData.clarke.Ib_prev = CurrentCtlr_MainData.clarke.Ib;
}

static void CurrentCtlr_ParkTransform(void) {
  CurrentCtlr_MainData.park.Id =
      CurrentCtlr_MainData.clarke.Ia * cosf(CurrentCtlr_MainData.theta) +
      CurrentCtlr_MainData.clarke.Ib * sinf(CurrentCtlr_MainData.theta);
  CurrentCtlr_MainData.park.Iq =
      -CurrentCtlr_MainData.clarke.Ia * sinf(CurrentCtlr_MainData.theta) +
      CurrentCtlr_MainData.clarke.Ib * cosf(CurrentCtlr_MainData.theta);
}

static void CurrentCtlr_Observer(void) {
  /* Previous values */
  CurrentCtlr_MainData.park.Vd_prev = CurrentCtlr_MainData.park.Vd;
  CurrentCtlr_MainData.park.Vq_prev = CurrentCtlr_MainData.park.Vq;
  /* Vq error */
  CurrentCtlr_MainData.park.eq_prev = CurrentCtlr_MainData.park.eq;
  CurrentCtlr_MainData.park.eq =
      CurrentCtlr_MainData.park.Iq_desired -
      CurrentCtlr_MainData.park
          .Iq; /* TODO Iq_desired comes from the torque request */
  /* Vd error */
  CurrentCtlr_MainData.park.ed_prev = CurrentCtlr_MainData.park.ed;
  CurrentCtlr_MainData.park.ed = 0 - CurrentCtlr_MainData.park.Id;
  /* Actual calculation */
  CurrentCtlr_MainData.park.Vq =
      CurrentCtlr_MainData.park.Vq_prev +
      (CurrentCtlr_MainData.park.eq - CurrentCtlr_MainData.park.eq_prev) * Kp +
      CurrentCtlr_MainData.park.eq * Ki * Ads8688_TimingData.DeltaTime;
  CurrentCtlr_MainData.park.Vd =
      CurrentCtlr_MainData.park.Vd_prev +
      (CurrentCtlr_MainData.park.ed - CurrentCtlr_MainData.park.ed_prev) * Kp +
      CurrentCtlr_MainData.park.ed * Ki * Ads8688_TimingData.DeltaTime;
}

static void CurrentCtlr_InverseParkTrasform(void) {
  CurrentCtlr_MainData.clarke.Va =
      CurrentCtlr_MainData.park.Vd * cosf(CurrentCtlr_MainData.theta) -
      CurrentCtlr_MainData.park.Vq * sinf(CurrentCtlr_MainData.theta);
  CurrentCtlr_MainData.clarke.Vb =
      CurrentCtlr_MainData.park.Vd * sinf(CurrentCtlr_MainData.theta) +
      CurrentCtlr_MainData.park.Vq * cosf(CurrentCtlr_MainData.theta);
}

static void CurrentCtlr_InverseClarkeTransform(void) {
  float duty_tmp_float = 0;
  CurrentCtlr_MainData.V[PHASE_U] = CurrentCtlr_MainData.clarke.Va;
  CurrentCtlr_MainData.V[PHASE_V] =
      -CurrentCtlr_MainData.clarke.Va / 2 +
      CurrentCtlr_MainData.clarke.Vb * (SQRT_3 / 2);
  CurrentCtlr_MainData.V[PHASE_W] =
      -CurrentCtlr_MainData.clarke.Va / 2 -
      CurrentCtlr_MainData.clarke.Vb * (SQRT_3 / 2);

  for (uint8_t i = 0; i < CURRENTCTLR_PHASES; i++) {
    duty_tmp_float = CurrentCtlr_MainData.V[i] * MAX_DUTY / MAX_VOLTAGE;
    CurrentCtlr_Duty[i] = (uint32_t)duty_tmp_float;

    // Safety clamp to ensure it never breaches boundaries
    if (duty_tmp_float > MAX_DUTY)  duty_tmp_float = MAX_DUTY;
    if (duty_tmp_float < 0.0f)      duty_tmp_float = 0.0f;
    
    CurrentCtlr_Duty[i] = (uint32_t)duty_tmp_float;
  }
}

static void CurrentCtlr_Main(void) {
  switch (CurrentCtlr_State) {
  case V_OFFSET_CALC_PHASE: {
    if (CurrentCtlr_IsrCnt >= (V_OFFSET_CALC_PHASE_MAX_CNT - 1)) {
      CurrentCtlr_IsrCnt = 0;
      /* Calculate the offset values for the currents measurements */
      for (uint8_t i = 0; i < ADS8688_CHANNELS; i++) {
        CurrentCtlr_VrefOffset[i] += Ads8688_AdcVoltage[i];
        CurrentCtlr_VrefOffset[i] /= V_OFFSET_CALC_PHASE_MAX_CNT;
      }
      /* Update the vref offset for the gatedriver */
      Igd_UpdateVoltageOffset(CurrentCtlr_VrefOffset, 2);
      /* Activate the pwm for only phase u 10 per cent */
      CurrentCtlr_Duty[0] = 12000;
      CurrentCtlr_Duty[1] = 0;
      CurrentCtlr_Duty[2] = 0;
      Pwm_SetDutycycle(CurrentCtlr_Duty);
      /* Set the deadtime */
      Pwm_SetDeadTime(DEADTIME);
      /* Enable the pwm state machine */
      Pwm_EnableOutputs(true);
      CurrentCtlr_State = R_CALC_PHASE;
    } else {
      for (uint8_t i = 0; i < ADS8688_CHANNELS; i++) {
        CurrentCtlr_VrefOffset[i] += Ads8688_AdcVoltage[i];
      }
      CurrentCtlr_IsrCnt++;
    }
  } break;

  case R_CALC_PHASE: {
    if (CurrentCtlr_IsrCnt >= (R_CALC_PHASE_MAX_CNT - 1)) {
      CurrentCtlr_IsrCnt = 0;
      /* For now the theta is zero as initialized value */
      CurrentCtlr_MainData.theta = 0;
      /* Calculate the Main data v and i values */
      CurrentCtlr_MainData.I[0] += Ads8688_AdcVoltage[0];
      CurrentCtlr_MainData.I[0] /= R_CALC_PHASE_MAX_CNT;
      Igd_CalculateCurrent(CurrentCtlr_MainData.I, CurrentCtlr_MainData.I, 1);

      CurrentCtlr_MainData.V[0] = MAX_VOLTAGE * CurrentCtlr_Duty[0] /
                                  MAX_DUTY; /* Same voltage we are ok here */

      /* Calculate the actual Resistance */
      CurrentCtlr_MainData.R =
          CurrentCtlr_MainData.V[0] / CurrentCtlr_MainData.I[0];
      /* Reset the current and voltage */
      CurrentCtlr_MainData.I[0] = 0;
      CurrentCtlr_MainData.V[0] = 0;
      CurrentCtlr_State = PI_CONTROL_PHASE;
    } else {
      CurrentCtlr_MainData.I[0] += Ads8688_AdcVoltage[0];
      CurrentCtlr_IsrCnt++;
    }
  } break;

  case PI_CONTROL_PHASE: {
    /* Calculate the current for phases u and v */
    Igd_CalculateCurrent(CurrentCtlr_MainData.I, Ads8688_AdcVoltage,
                         ADS8688_CHANNELS);
    /* Kirchhoff law */
    CurrentCtlr_MainData.I[PHASE_W] =
        -CurrentCtlr_MainData.I[PHASE_U] - CurrentCtlr_MainData.I[PHASE_V];
    /* Clarke Transform */
    CurrentCtlr_ClarkeTransform();
    /* First run? theta is 0 */
    if (CurrentCtlr_IsrCnt == 0) {
      CurrentCtlr_MainData.theta = 0;
    } else {
      CurrentCtlr_CalcTheta();
    }
    /* Park Transform */
    CurrentCtlr_ParkTransform();
    /* Observer */
    CurrentCtlr_Observer();
    /* Inverse Park */
    CurrentCtlr_InverseParkTrasform();
    /* Inverse Clarke */
    CurrentCtlr_InverseClarkeTransform();
    /* Update Pwms */
    Pwm_SetDutycycle(CurrentCtlr_Duty);
    CurrentCtlr_IsrCnt++;
  } break;

  default:
    break;
  }
}

/* Global functions */
void CurrentCtlr_Init(void) {
  CurrentCtlr_State = V_OFFSET_CALC_PHASE;
  CurrentCtlr_IsrCnt = 0;
  CurrentCtrl_FreewheelTimeFirst = Ads8688_TimingData.CurrentTime;
  /* Initialize the clarke */
  CurrentCtlr_MainData.clarke.Ia = 0;
  CurrentCtlr_MainData.clarke.Ib = 0;
  CurrentCtlr_MainData.clarke.Ia_prev = 0;
  CurrentCtlr_MainData.clarke.Ib_prev = 0;
  CurrentCtlr_MainData.clarke.Va = 0;
  CurrentCtlr_MainData.clarke.Vb = 0;
  /* Initialize the park */
  CurrentCtlr_MainData.park.Id = 0;
  CurrentCtlr_MainData.park.Iq = 0;
  CurrentCtlr_MainData.park.Vd = 0;
  CurrentCtlr_MainData.park.Vq = 0;
  CurrentCtlr_MainData.park.Iq_desired = 0.95; /* 1000 RPM */
  CurrentCtlr_MainData.park.ed = 0;
  CurrentCtlr_MainData.park.eq = 0;
  CurrentCtlr_MainData.park.ed_prev = 0;
  CurrentCtlr_MainData.park.eq_prev = 0;
  CurrentCtlr_MainData.park.Vd_prev = 0;
  CurrentCtlr_MainData.park.Vq_prev = 0;
  /* Initialize the VRef offset and the voltages */
  for (uint8_t i = 0; i < ADS8688_CHANNELS; i++) {
    CurrentCtlr_MainData.V[i] = 0;
    CurrentCtlr_VrefOffset[i] = Ads8688_AdcVoltage[i];
  }
  /* Already one calculation */
  CurrentCtlr_IsrCnt++;
}

void CurrentCtlr_Isr(void) { CurrentCtlr_Main(); }
