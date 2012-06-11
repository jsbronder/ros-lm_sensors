/*
 * Copyright (c) 2012, Justin Bronder
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the organization nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstring>
#include <cstdlib>

#include "LMSensor.hpp"

namespace lm_sensors {

LMSensor::LMSensor(const sensors_chip_name *name, const sensors_feature *feature) :
    m_label(""),
    m_value(0.0),
    m_alarm(false)
{
    std::memcpy(&m_name, name, sizeof(sensors_chip_name));
    std::memcpy(&m_feature, feature, sizeof(sensors_feature));

    int r;
    char * label = sensors_get_label(&m_name, &m_feature);
    if (label) {
        m_label.assign(label);
        std::free(label);
    } else {
        err("Failed to parse sensor feature name\n");
    }

    update();
}

LMSensor::~LMSensor()
{}

const std::string &LMSensor::label() const
{
    return m_label;
}

bool LMSensor::alarm() const
{
    return m_alarm;
}

double LMSensor::value() const
{
    return m_value;
}

void LMSensor::update()
{
    switch (m_feature.type) {
        case SENSORS_FEATURE_IN:
            _update_feature_in();
            break;

        case SENSORS_FEATURE_FAN:
            _update_feature_fan();
            break;

        case SENSORS_FEATURE_TEMP:
            _update_feature_temp();
            break;

        case SENSORS_FEATURE_POWER:
        case SENSORS_FEATURE_ENERGY:
        case SENSORS_FEATURE_CURR:
        case SENSORS_FEATURE_VID:
        case SENSORS_FEATURE_BEEP_ENABLE:
            err("Unimplemented sensor feature type");
            break;

        default:
            err("Unknown sensor feature type");
            break;
    }
}

std::string LMSensor::to_string() const
{
    char buf[128];

    switch (m_feature.type) {
        case SENSORS_FEATURE_IN:
            snprintf(buf, 127, "%6.1f V", m_value);
            break;

        case SENSORS_FEATURE_TEMP:
            snprintf(buf, 127, "%6.1f C", m_value);
            break;

        case SENSORS_FEATURE_FAN:
            snprintf(buf, 127, "%6.1f RPM", m_value);
            break;

        default:
            snprintf(buf, 127, "%6.1f", m_value);
            break;
    }
    return std::string(buf);
}

void LMSensor::ros_update(diagnostic_updater::DiagnosticStatusWrapper &dsw)
{
    update();

    if (alarm())
        dsw.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Alarm active");
    else
        dsw.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");

    dsw.add(label(), to_string());
}

void LMSensor::_update_feature_in()
{
    const sensors_subfeature *sf;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_IN_INPUT);
    m_value = sf ? _get_value(sf) : 0.0;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_IN_ALARM);
    m_alarm = sf && _get_value(sf);

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_IN_MIN_ALARM);
    m_alarm |= sf && _get_value(sf);

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_IN_MAX_ALARM);
    m_alarm |= sf && _get_value(sf);
}

void LMSensor::_update_feature_fan()
{
    const sensors_subfeature *sf;

    /*
     * If the sensor is reporting a fault, we can't trust anything else
     * it might tell us.  Keep previous/default values.
     */
    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_FAN_FAULT);
    if (sf && _get_value(sf))
        return;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_FAN_INPUT);
    m_value = sf ? _get_value(sf) : 0.0;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_FAN_ALARM);
    m_alarm = sf && _get_value(sf);
}

void LMSensor::_update_feature_temp()
{
    const sensors_subfeature *sf;

    /*
     * If the sensor is reporting a fault, we can't trust anything else
     * it might tell us.  Keep previous/default values.
     */
    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_TEMP_FAULT);
    if (sf && _get_value(sf))
        return;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_TEMP_INPUT);
    m_value = sf ? _get_value(sf) : 0.0;

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_TEMP_ALARM);
    m_alarm = sf && _get_value(sf);

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_TEMP_MAX_ALARM);
    m_alarm |= sf && _get_value(sf);

    sf = sensors_get_subfeature(&m_name, &m_feature, SENSORS_SUBFEATURE_TEMP_MIN_ALARM);
    m_alarm |= sf && _get_value(sf);
}

double LMSensor::_get_value(const sensors_subfeature *sf)
{
    double ret = 0.0;

    int r = sensors_get_value(&m_name, sf->number, &ret);
    if (r)
    {
        ret = 0.0;
        err("Failed to read value of subfeature %s\n", sf->name);
    }

    return ret;
}

std::ostream &operator<<(std::ostream &stream, const LMSensor &sensor)
{
    stream << sensor.m_label << ":  " << sensor.to_string();
    if (sensor.m_alarm)
        stream << " ALARM";

    return stream;
}

} // namespace lm_sensors



