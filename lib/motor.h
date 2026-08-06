// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Declarations for all built-in motor driver classes (Generic2, Generic1,
// BTS7960, ESC). Implementations are in motor.cpp - same .h/.cpp split used by
// lib/steering and lib/traxxas_remctl, instead of the header-only style
// default_motor.h used previously.

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"
#include "pwm.h"
#include "motor_interface.h"

// Motor drivers with 2 Direction Pins (INA, INB) and 1 PWM (ENABLE) pin,
// ie. L298, L293, VNH5019.
class Generic2: public MotorInterface
{
    public:
        Generic2(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_a_pin, int in_b_pin);

        void begin();
        void brake() override;

    protected:
        void forward(int pwm) override;
        void reverse(int pwm) override;

    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_pin_;
        int pwm_bits_;
        float pwm_frequency_;
};

// Motor drivers with 1 Direction Pin (INA) and 1 PWM (ENABLE) pin.
class Generic1: public MotorInterface
{
    public:
        Generic1(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_pin, int unused = -1);

        void begin();
        void brake() override;

    protected:
        void forward(int pwm) override;
        void reverse(int pwm) override;

    private:
        int in_pin_;
        int pwm_pin_;
        int pwm_bits_;
        float pwm_frequency_;
};

// BTS7970 motor driver.
class BTS7960: public MotorInterface
{
    public:
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int unused, int in_a_pin, int in_b_pin);
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int in_a_pin, int in_b_pin);

        void begin();
        void brake() override;

    protected:
        void forward(int pwm) override;
        void reverse(int pwm) override;

    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_bits_;
        int pwm_max_;
        float pwm_frequency_;
};

// Bidirectional (forward/reverse) ESC or standard hobby servo, driven by a
// single microsecond-pulse PWM signal (1500us = neutral/center). Used for both
// the drive-wheel ESC and, on ACKERMANN, the steering servo/ESC channel
// (MOTOR_STR_*) via lib/steering.
class ESC: public MotorInterface
{
    public:
        static const int PWM_NEUTRAL = 1500;

        ESC(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int unused = -1, int unused2 = -1);

        void begin();
        void brake() override;

    protected:
        void forward(int pwm) override;
        void reverse(int pwm) override;

    private:
        int pwm_pin_;
};

// now you can create a config constant that you can use in lino_base_config.h
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
    // pass your built in class to Motor macro
    #define Motor Generic2
#endif

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
    // pass your built in class to Motor macro
    #define Motor Generic1
#endif

#ifdef USE_BTS7960_MOTOR_DRIVER
    // pass your built in class to Motor macro
    #define Motor BTS7960
#endif

#ifdef USE_ESC_MOTOR_DRIVER
    // pass your built in class to Motor macro
    #define Motor ESC
#endif

#endif
