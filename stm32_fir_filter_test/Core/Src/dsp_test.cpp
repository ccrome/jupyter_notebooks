/*
 * dsp_test.c
 *
 *  Created on: Nov 20, 2019
 *      Author: caleb
 */

using namespace std;
extern "C" {
#include "stm32h7xx_hal.h"
#include <arm_math.h>
#include "main.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "assert.h"
}
#define MAX_BLOCKSIZE 32
#define MAX_CHANNELS  16
#define MAX_COEFS  320

extern TIM_HandleTypeDef htim2;

template <class dtype>
class Array {
	dtype *data;
	uint32_t blocksize;
	uint32_t channels;

public:
	void init(void *data, uint32_t channels, uint32_t blocksize) {
		this->data = (dtype *)data;
		this->blocksize = blocksize;
		this->channels = channels;
	}
	dtype *get(int channel) {
		int offset = this->blocksize * channel;
		return &this->data[offset];
	}
	int get_blocksize() {
		return this->blocksize;
	}
	int get_channels() {
		return this->channels;
	}
};

template <typename dtype, typename fir_type>
void fir_init(fir_type instances[], int channels, Array<dtype> &coefs, Array<dtype> &states, int blocksize) {
	int channel;
	dtype *taps  = coefs.get(0);
	int ntaps = coefs.get_blocksize();

	for (channel = 0; channel < channels; channel++) {
		dtype *state = states.get(channel);
		if constexpr(std::is_same_v<dtype, float>)
			arm_fir_init_f32(&instances[channel], ntaps,  taps, state, blocksize);
		else if constexpr(std::is_same_v<dtype, int32_t>)
			arm_fir_init_q31(&instances[channel], ntaps,  taps, state, blocksize);
		else if constexpr(std::is_same_v<dtype, int16_t>)
			arm_fir_init_q15(&instances[channel], ntaps,  taps, state, blocksize);
		else {
			assert(0);
		}
	}
}

template <typename dtype, typename fir_type>
void fir_run(fir_type *instances, Array<dtype> &din, Array<dtype> &dout, const Array<dtype> &coefs, int is_fast) {
	int channels = din.get_channels();
	int blocksize = din.get_blocksize();
	int channel;
	for (channel = 0; channel < channels; channel++) {
		if constexpr(std::is_same_v<dtype, float>)
				arm_fir_f32(&instances[channel], din.get(channel), dout.get(channel), blocksize);
		if constexpr(std::is_same_v<dtype, int32_t>) {
				if (!is_fast)
					arm_fir_q31(&instances[channel], din.get(channel), dout.get(channel), blocksize);
				else
					arm_fir_fast_q31(&instances[channel], din.get(channel), dout.get(channel), blocksize);
		} if constexpr(std::is_same_v<dtype, int16_t>) {
				if (!is_fast)
					arm_fir_q15(&instances[channel], din.get(channel), dout.get(channel), blocksize);
				else
					arm_fir_fast_q15(&instances[channel], din.get(channel), dout.get(channel), blocksize);
		}
	}
}

extern "C" {
	int _write(int file, unsigned char *ptr, int len) {
		CDC_Transmit_HS(ptr, len);
		return len;
	}
}


float coefs_mem[MAX_COEFS];
float din_mem [MAX_CHANNELS*MAX_BLOCKSIZE];
float dout_mem[MAX_CHANNELS*MAX_BLOCKSIZE];
float states_mem[MAX_CHANNELS*(MAX_COEFS+MAX_BLOCKSIZE-1)];

template <typename dtype, typename fir_type>
void dsp_test_(int is_fast=0)
{
	int ntaps;
	int channels;
	int blocksize;
	unsigned long int start, end;
	double timer_clock_speed = 200e6;
	const char *label;
	const char *extra;
	extra = "";
	if constexpr(std::is_same_v<dtype, float>)
			label = "f32";
	else if constexpr(std::is_same_v<dtype, int32_t>)
			label= "q31";
	else if constexpr(std::is_same_v<dtype, int16_t>)
			label = "q15";
	else
			assert(0);
	if (is_fast)
		extra = "f";
	HAL_TIM_Base_Start(&htim2);
	fir_type instances[MAX_CHANNELS];
	for (channels = 1; channels <= MAX_CHANNELS; channels++) {
		for (blocksize=1; blocksize <= MAX_BLOCKSIZE; blocksize++) {
			for (ntaps = 10; ntaps <= MAX_COEFS; ntaps++) {
				unsigned long elapsed_sec;
				Array<dtype> din, dout, states, coefs;
				din.init(din_mem, channels, blocksize);
				dout.init(dout_mem, channels, blocksize);
				states.init(states_mem, channels, ntaps+blocksize-1);
				coefs.init(coefs_mem, 1, ntaps);
				fir_init(instances, channels, coefs, states, blocksize);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				start = TIM2->CNT;
				fir_run(instances, din, dout, coefs, is_fast);
				end = TIM2->CNT;
				elapsed_sec = (unsigned long)(((end-start) * 1.0 /timer_clock_speed)*1e9); // nanoseconds
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_Delay(1);
				printf("%s%s,%d,%d,%d,%lu\n", label, extra, channels, blocksize, ntaps, elapsed_sec);
			}
		}
	}
}

extern "C" void dsp_test()
{
	HAL_Delay(5000);
	int fast = 1;
	int slow = 0;
	printf("type,channels,blocksize,ntaps,time_ns\n");
	dsp_test_<int32_t, arm_fir_instance_q31>(slow);
	dsp_test_<int32_t, arm_fir_instance_q31>(fast);
	dsp_test_<int16_t, arm_fir_instance_q15>(slow);
	dsp_test_<int16_t, arm_fir_instance_q15>(fast);
	dsp_test_<float  , arm_fir_instance_f32>();
}
