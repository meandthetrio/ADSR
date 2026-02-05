#pragma once

#include "shared_messages.h"
#include <cmath>

// DSP helpers used by the audio engine and control-rate mapping.

class TapeSaturator
{
public:
	void Init(float sample_rate)
	{
		sample_rate_ = sample_rate;
		dc_pre_.Init(sample_rate, 20.0f);
		dc_post_.Init(sample_rate, 20.0f);
		low_bump_lp_.Init(sample_rate, 90.0f);
		high_emph_lp_.Init(sample_rate, 2000.0f);
		post_lp_.Init(sample_rate, 16000.0f);
		release_coeff_ = expf(-1.0f / (0.08f * sample_rate));
		smooth_coeff_ = 1.0f - expf(-1.0f / (0.03f * sample_rate));
		drive_target_ = 0.0f;
		bias_target_ = 0.0f;
		tone_target_ = 0.5f;
		bump_target_ = 0.0f;
		mix_target_ = 0.0f;
		output_gain_target_ = 1.0f;
		post_gain_target_ = 1.0f;
		drive_ = 0.0f;
		bias_ = 0.0f;
		tone_ = 0.5f;
		bump_ = 0.0f;
		mix_ = 0.0f;
		output_gain_ = 1.0f;
		post_gain_ = 1.0f;
		env_ = 0.0f;
		prev_y_ = 0.0f;
		UpdatePostCutoff(drive_);
	}

	float Process(float x)
	{
		drive_ += smooth_coeff_ * (drive_target_ - drive_);
		bias_ += smooth_coeff_ * (bias_target_ - bias_);
		tone_ += smooth_coeff_ * (tone_target_ - tone_);
		bump_ += smooth_coeff_ * (bump_target_ - bump_);
		mix_ += smooth_coeff_ * (mix_target_ - mix_);
		output_gain_ += smooth_coeff_ * (output_gain_target_ - output_gain_);
		post_gain_ += smooth_coeff_ * (post_gain_target_ - post_gain_);

		const float dry = x;
		x = dc_pre_.Process(x);

		const float pre_emph_boost = 1.0f + drive_;
		const float low_amt = (0.06f + 0.08f * (1.0f - tone_)) * pre_emph_boost;
		const float high_amt = (0.02f + 0.08f * tone_) * pre_emph_boost;
		const float low = low_bump_lp_.Process(x);
		const float high = x - high_emph_lp_.Process(x);
		const float bump_curve = bump_ * bump_;
		const float mid = x - low - high;
		const float bump_low = bump_curve * 0.25f;
		const float bump_mid = bump_curve * 1.6f;
		const float mid_res = bump_curve * 1.8f;
		const float mid_res_sample = (mid + (prev_y_ * 0.35f)) * mid_res;
		x = x + (low * (low_amt + bump_low))
			+ (mid * bump_mid)
			+ (mid_res_sample)
			+ (high * high_amt);

		const float abs_x = fabsf(x);
		if (abs_x > env_)
		{
			env_ = abs_x;
		}
		else
		{
			env_ *= release_coeff_;
		}
		const float comp_amt = 0.35f * drive_;
		const float eff_drive = drive_ / (1.0f + comp_amt * env_);
		const float drive_boost = (1.0f + drive_) * (1.0f + drive_);

		const float bias_amt = bias_ * 0.12f;
		const float xb = x * (1.0f + eff_drive * 2.5f * drive_boost) + bias_amt;
		float y = FastTanh(xb);
		const float mem_amt = 0.05f + 0.1f * drive_;
		y += mem_amt * prev_y_;
		prev_y_ = y;
		y = dc_post_.Process(y);

		y = post_lp_.Process(y);
		y *= post_gain_;

		float out = (mix_ * y) + ((1.0f - mix_) * dry);
		out *= output_gain_;
		return out;
	}

	void SetDrive(float d01)
	{
		drive_target_ = Clamp(d01, 0.0f, 1.0f);
		UpdatePostCutoff(drive_target_);
		post_gain_target_ = DbToGain(-6.0f * drive_target_);
	}

	void SetBias(float b11)
	{
		bias_target_ = Clamp(b11, -1.0f, 1.0f);
	}

	void SetTone(float t01)
	{
		tone_target_ = Clamp(t01, 0.0f, 1.0f);
	}

	void SetBump(float b01)
	{
		bump_target_ = Clamp(b01, 0.0f, 1.0f);
	}

	void SetMix(float m01)
	{
		mix_target_ = Clamp(m01, 0.0f, 1.0f);
	}

	void SetOutput(float o01)
	{
		const float out = Clamp(o01, 0.0f, 1.0f);
		const float gain_db = -12.0f + (18.0f * out);
		output_gain_target_ = powf(10.0f, gain_db / 20.0f);
	}

private:
	struct DcBlocker
	{
		float x1 = 0.0f;
		float y1 = 0.0f;
		float r = 0.0f;

		void Init(float sample_rate, float cutoff_hz)
		{
			r = expf(-2.0f * 3.14159265f * cutoff_hz / sample_rate);
			x1 = 0.0f;
			y1 = 0.0f;
		}

		float Process(float x)
		{
			const float y = x - x1 + (r * y1);
			x1 = x;
			y1 = y;
			return y;
		}
	};

	struct OnePoleLp
	{
		float a = 0.0f;
		float y = 0.0f;

		void Init(float sample_rate, float cutoff_hz)
		{
			SetFreq(sample_rate, cutoff_hz);
			y = 0.0f;
		}

		void SetFreq(float sample_rate, float cutoff_hz)
		{
			a = expf(-2.0f * 3.14159265f * cutoff_hz / sample_rate);
		}

		float Process(float x)
		{
			y = (1.0f - a) * x + (a * y);
			return y;
		}
	};

	static float Clamp(float v, float lo, float hi)
	{
		if (v < lo)
		{
			return lo;
		}
		if (v > hi)
		{
			return hi;
		}
		return v;
	}

	static float FastTanh(float x)
	{
		if (x > 3.0f)
		{
			x = 3.0f;
		}
		else if (x < -3.0f)
		{
			x = -3.0f;
		}
		const float x2 = x * x;
		return x * (27.0f + x2) / (27.0f + 9.0f * x2);
	}

	static float DbToGain(float db)
	{
		return powf(10.0f, db / 20.0f);
	}

	void UpdatePostCutoff(float drive)
	{
		float cutoff = 16000.0f - (drive * 8000.0f);
		if (cutoff < 8000.0f)
		{
			cutoff = 8000.0f;
		}
		post_lp_.SetFreq(sample_rate_, cutoff);
	}

	float sample_rate_ = 48000.0f;
	float drive_target_ = 0.0f;
	float bias_target_ = 0.0f;
	float tone_target_ = 0.5f;
	float bump_target_ = 0.0f;
	float mix_target_ = 0.0f;
	float output_gain_target_ = 1.0f;
	float post_gain_target_ = 1.0f;

	float drive_ = 0.0f;
	float bias_ = 0.0f;
	float tone_ = 0.5f;
	float bump_ = 0.0f;
	float mix_ = 0.0f;
	float output_gain_ = 1.0f;
	float post_gain_ = 1.0f;

	float env_ = 0.0f;
	float prev_y_ = 0.0f;
	float release_coeff_ = 0.0f;
	float smooth_coeff_ = 0.0f;

	DcBlocker dc_pre_;
	DcBlocker dc_post_;
	OnePoleLp low_bump_lp_;
	OnePoleLp high_emph_lp_;
	OnePoleLp post_lp_;
};

class BiquadLp
{
public:
	void Reset()
	{
		z1_ = 0.0f;
		z2_ = 0.0f;
	}

	void Set(float sample_rate, float freq, float q)
	{
		if (freq < 20.0f)
		{
			freq = 20.0f;
		}
		const float nyq = sample_rate * 0.49f;
		if (freq > nyq)
		{
			freq = nyq;
		}
		if (q < 0.001f)
		{
			q = 0.001f;
		}

		const float w0 = (2.0f * 3.14159265f * freq) / sample_rate;
		const float cos_w0 = cosf(w0);
		const float sin_w0 = sinf(w0);
		const float alpha = sin_w0 / (2.0f * q);

		const float b0 = (1.0f - cos_w0) * 0.5f;
		const float b1 = 1.0f - cos_w0;
		const float b2 = (1.0f - cos_w0) * 0.5f;
		const float a0 = 1.0f + alpha;
		const float a1 = -2.0f * cos_w0;
		const float a2 = 1.0f - alpha;

		a0_ = b0 / a0;
		a1_ = b1 / a0;
		a2_ = b2 / a0;
		b1_ = a1 / a0;
		b2_ = a2 / a0;
	}

	void SetCoeffs(const BiquadCoeffs& c)
	{
		a0_ = c.a0;
		a1_ = c.a1;
		a2_ = c.a2;
		b1_ = c.b1;
		b2_ = c.b2;
	}

	float Process(float x)
	{
		const float y = (a0_ * x) + z1_;
		z1_ = (a1_ * x) + z2_ - (b1_ * y);
		z2_ = (a2_ * x) - (b2_ * y);
		return y;
	}

private:
	float a0_ = 0.0f;
	float a1_ = 0.0f;
	float a2_ = 0.0f;
	float b1_ = 0.0f;
	float b2_ = 0.0f;
	float z1_ = 0.0f;
	float z2_ = 0.0f;
};

inline BiquadCoeffs ComputeBiquadLpCoeffs(float sample_rate, float freq, float q)
{
	if (freq < 20.0f)
	{
		freq = 20.0f;
	}
	const float nyq = sample_rate * 0.49f;
	if (freq > nyq)
	{
		freq = nyq;
	}
	if (q < 0.001f)
	{
		q = 0.001f;
	}
	const float w0 = (2.0f * 3.14159265f * freq) / sample_rate;
	const float cos_w0 = cosf(w0);
	const float sin_w0 = sinf(w0);
	const float alpha = sin_w0 / (2.0f * q);

	const float b0 = (1.0f - cos_w0) * 0.5f;
	const float b1 = 1.0f - cos_w0;
	const float b2 = (1.0f - cos_w0) * 0.5f;
	const float a0 = 1.0f + alpha;
	const float a1 = -2.0f * cos_w0;
	const float a2 = 1.0f - alpha;

	BiquadCoeffs c;
	c.a0 = b0 / a0;
	c.a1 = b1 / a0;
	c.a2 = b2 / a0;
	c.b1 = a1 / a0;
	c.b2 = a2 / a0;
	return c;
}

class OnePoleHp
{
public:
	void Init(float sample_rate, float cutoff_hz)
	{
		SetFreq(sample_rate, cutoff_hz);
		y_ = 0.0f;
		x1_ = 0.0f;
	}

	void SetFreq(float sample_rate, float cutoff_hz)
	{
		a_ = expf(-2.0f * 3.14159265f * cutoff_hz / sample_rate);
	}

	float Process(float x)
	{
		const float y = a_ * (y_ + x - x1_);
		x1_ = x;
		y_ = y;
		return y;
	}

	void Reset()
	{
		y_ = 0.0f;
		x1_ = 0.0f;
	}

private:
	float a_ = 0.0f;
	float y_ = 0.0f;
	float x1_ = 0.0f;
};

struct BitCrushState
{
	int hold = 0;
	float hold_l = 0.0f;
	float hold_r = 0.0f;
};
