#ifndef LOWPASS_H
#define LOWPASS_H

/* 一阶低通 */
typedef struct {
    float out;
    float alpha;
} lowpass1d_t;

void  Lowpass_Init(lowpass1d_t *lp, float alpha);
float Lowpass_Update(lowpass1d_t *lp, float in);
void  Lowpass_Reset(lowpass1d_t *lp);

#endif
