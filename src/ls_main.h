/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

/* global defines for LeakSentinel firmware */

#ifndef _LS_MAIN_
#define _LS_MAIN_
const char * valve_state_str[] = { "VUNKNOWN", "VOPEN", "VCLOSED", "VOPENING", "VCLOSING" };
const char * led_state_str[] = { "GREEN", "RED", "OFF" };
/* const char* motor_state_str[] = { "MUNKNOWN", "MIDLE", "MCW", "MCCW", "MSTALLED", "MTIMEDOUT"}; */
/* const char* valve_type_str[] = { "TUNKNOWN", "TGATE", "TBALL", "TTEE", "TOTHER"}; */
/* const char* battery_state_simple_str[] = { "BUNKNOWN", "BCHARGED", "BDISCHARGING", "BCHARGING", "BLOW"}; */
/* const char* battery_state_str[] = { "BUNKNOWN", "BDIDLE", "BDTRICKLE", "BDPRE", "BDTAPER","BDTOPOFF","BDDONE","BDFAST","BDRESERVED"}; */
/* const char* source_state_str[] = { "SNO_INPUT", "SUSB_HOST_SDP", "SUSB_CDP_1P5A", "SUSB_DCP_3P0A","SPOORSRC","SUNKNOWN_ADAPTER","SNONSTANDARD_ADAPTER","SOTG","SUNKNOWN"}; */
/* const char* flow_state_str[] = { "FUNKNOWN", "FNOFLOW", "FFLOW"}; */
/* const char* wifi_state_str[] = { "WUNKNOWN", "WCONNECTED", "WUNCONNECTED", "WCONNECTING", "WDISCONNECTED"}; */

float wifi_rssi = -198.0;

uint16_t my_awake_time = 30;
uint16_t my_sleep_multiplier = 4;

uint8_t lsADC_NumSmpls = 32;

/* https://www.precisionmicrodrives.com/content/ab-022-pwm-frequency-for-linear-motion-control/ */
/* If our resistance is approx 1 Ohm and L = ?? */
/* We should operate at a minimum pulse width of 5 * tau where tau = L/R */
/* If we want the minimum PWM duty cycle to be 10% (as an example), then the */
/* PWM period should be 5 * tau * 10 (which is 1/(minimum duty cycle)) */
/* which is 50 * L in our case (with R ~ 1 Ohm) */
/* therefore, frequency is 1/(L * 50) */
/* uint16_t lsMTR_pwmFreq = 500; */

float lsMTR_maxCurrent = 6.0;

uint16_t lsMTR_currentLimHigh = 80; /* need to adjust limit per voltage supply @ 6v = 2.5A, @ 8.5v = 3.4A */

uint16_t lsMTR_delayStartMS = 250;

uint8_t lsMTR_maxTimeS = 3;
uint16_t lsMTR_turnBackmS = 500;

uint32_t lsMTR_runTimeMS = 0; /* in milliseconds */

#endif /* _LS_MAIN_ */
