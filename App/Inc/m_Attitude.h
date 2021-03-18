#ifndef  _ATTITUDE_H_
#define  _ATTITUDE_H_


extern void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
extern float LPF2pApply_1(float sample);

extern float Get_Pitch();

#endif 
