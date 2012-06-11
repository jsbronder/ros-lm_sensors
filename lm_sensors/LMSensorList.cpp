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

#include "LMSensorList.hpp"

namespace lm_sensors {

LMSensorList::LMSensorList()
{
    sensors_init(NULL);
    int sensor_i = 0;
    const sensors_chip_name *sc;

    while ((sc = sensors_get_detected_chips(NULL, &sensor_i))) {
        int feature_i = 0;
        const sensors_feature *sf;
        while ((sf = sensors_get_features(sc, &feature_i)))
        {
            LMSensor *lms = new LMSensor(sc, sf);
            m_sensors.push_back(lms);
        }
    }
}

LMSensorList::~LMSensorList()
{
    for (LMSensorList::iterator it = m_sensors.begin(); it != m_sensors.end(); ++it)
        delete *it;
    sensors_cleanup();
}

void LMSensorList::update()
{
    for (LMSensorList::iterator it = m_sensors.begin(); it != m_sensors.end(); ++it)
        (*it)->update();
} 

LMSensorList::const_iterator LMSensorList::begin() const
{
    return m_sensors.begin();
}

LMSensorList::const_iterator LMSensorList::end() const
{
    return m_sensors.end();
}

} // namespace lm_sensors
