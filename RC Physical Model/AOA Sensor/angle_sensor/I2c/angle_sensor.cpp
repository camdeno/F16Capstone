/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_config.h>
#include <drivers/device/device.h>

#include <drivers/device/i2c.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/angle_sensor.h>
#include <uORB/topics/subsystem_info.h>

#include <drivers/distance_sensor/angle_source/angle.h>

Angle::Angle(int bus, int address, unsigned conversion_interval, const char *path) :
        I2C("Angle", path, bus, address, 100000),
    _sensor_ok(false),
    _last_published_sensor_ok(true), /* initialize differently to force publication */
    _measure_ticks(0),
    _collect_phase(false),
    _diff_pres_offset(0.0f),
    _angle_pub(nullptr),
    _angle_orb_class_instance(-1),
    _subsys_pub(nullptr),
    _class_instance(-1),
    _conversion_interval(conversion_interval),
    _sample_perf(perf_alloc(PC_ELAPSED, "aspd_read")),
    _comms_errors(perf_alloc(PC_COUNT, "aspd_com_err"))
{
    // enable debug() calls
    _debug_enabled = false;

    // work_cancel in the dtor will explode if we don't do this...
    memset(&_work, 0, sizeof(_work));
}

Angle::~Angle()
{
    /* make sure we are truly inactive */
    stop();

    if (_class_instance != -1) {
        unregister_class_devname(AIRSPEED_BASE_DEVICE_PATH, _class_instance);
    }

    orb_unadvertise(_angle_pub);

    // free perf counters
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int
Angle::init()
{
    /* do I2C init (and probe) first */
    if (I2C::init() != PX4_OK) {
        return PX4_ERROR;
    }

    /* register alternate interfaces if we have to */
    _class_instance = register_class_devname(AIRSPEED_BASE_DEVICE_PATH);

    /* advertise sensor topic, measure manually to initialize valid report */
    measure();
    angle_sensor_s data = {};

    /* measurement will have generated a report, publish */
        _angle_pub = orb_advertise_multi(ORB_ID(angle_sensor), &data,&_angle_orb_class_instance,
                                   ORB_PRIO_HIGH-_class_instance);

        if (_angle_pub == nullptr) {
        PX4_WARN("uORB started?");
    }

    return PX4_OK;
}

int
Angle::probe()
{
    /* on initial power up the device may need more than one retry
       for detection. Once it is running the number of retries can
       be reduced
    */
        _retries = 4;
    int ret = measure();

    // drop back to 2 retries once initialised
    _retries = 2;
    return ret;
}

int
Angle::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _measure_ticks = 0;
                return OK;

            /* external signaling (DRDY) not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
            case SENSOR_POLLRATE_DEFAULT: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* set interval for next measurement to minimum legal value */
                    _measure_ticks = USEC2TICK(_conversion_interval);

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* convert hz to tick interval via microseconds */
                    unsigned ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(_conversion_interval)) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    _measure_ticks = ticks;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }
        break;

    case SENSORIOCGPOLLRATE:
        if (_measure_ticks == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return (1000 / _measure_ticks);

    case SENSORIOCRESET:
        /* XXX implement this */
        return -EINVAL;

    case AIRSPEEDIOCSSCALE: {
            struct airspeed_scale *s = (struct airspeed_scale *)arg;
            _diff_pres_offset = s->offset_pa;
            return OK;
        }

    case AIRSPEEDIOCGSCALE: {
            struct airspeed_scale *s = (struct airspeed_scale *)arg;
            s->offset_pa = _diff_pres_offset;
            s->scale = 1.0f;
            return OK;
        }

    default:
        /* give it to the superclass */
        return I2C::ioctl(filp, cmd, arg);
    }
}

void
Angle::start()
{
    /* reset the report ring and state machine */
    _collect_phase = false;

    /* schedule a cycle to start things */
        work_queue(HPWORK, &_work, (worker_t)&Angle::cycle_trampoline, this, 1);
}

void
Angle::stop()
{
    work_cancel(HPWORK, &_work);
}

void
Angle::update_status()
{
    if (_sensor_ok != _last_published_sensor_ok) {
        /* notify about state change */
        struct subsystem_info_s info = {};
        info.present = true;
        info.enabled = true;
        info.ok = _sensor_ok;
        info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_DIFFPRESSURE;

        if (_subsys_pub != nullptr) {
            orb_publish(ORB_ID(subsystem_info), _subsys_pub, &info);

        } else {
            _subsys_pub = orb_advertise(ORB_ID(subsystem_info), &info);
        }

        _last_published_sensor_ok = _sensor_ok;
    }
}

void
Angle::cycle_trampoline(void *arg)
{
    Angle *dev = (Angle *)arg;
    dev->cycle();

    dev->update_status();
}
