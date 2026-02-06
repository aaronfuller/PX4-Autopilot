#include <px4_arch/io_timer_hw_description.h>

/* Timer allocation
 *
 * TIM1_CH1  T Motor 1               - PA8
 * TIM1_CH2  T Motor 2               - PE11
 *
 * TIM2_CH1  T Motor 3               - PA15
 * TIM2_CH3  T Motor 4               - PA2
 *
 * TIM3_CH2  T Motor 5               - PA7
 * TIM3_CH4  T Motor 6               - PB1
 *
 * TIM4_CH2  T Motor 7               - PB7
 * TIM4_CH3  T Motor 8               - PD14
 *
 * TIM5_CH4  T BUZZER_1              - PA3, driven by tone_alarm driver
 * TIM8_CH3  T HRT                   - High resolution timer
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer2, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer3, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin8}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin15}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortB, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
