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

#include <string>
#include <iostream>

#include <sensors/sensors.h>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

namespace lm_sensors {

#define err(fmt...) fprintf(stderr, fmt);

class LMSensor {
    public:
        /*
         * Constructor
         *
         * @param name  -   Sensor chip name as returned from libsensors.  For
         *                  instance, see the return value of
         *                  sensors_get_detected_chips().
         * @param feature - Sensor chip feature as returned from libsensors.
         *                  For instance see the return value of
         *                  sensors_get_features().
         */
        explicit LMSensor(const sensors_chip_name *name, const sensors_feature *feature);
        ~LMSensor();

        /*
         * @return - Name of the sensor
         */
        const std::string &label() const;

        /*
         * @return - True if the sensor is currently signaling an
         * alarm.
         */
        bool alarm() const;

        /*
         * @return - The reported value from the chip.
         */
        double value() const;

        /*
         * Read the latest value from the sensor.
         */
        void update();

        /*
         * @return - a string representation of the sensor value.
         */
        std::string to_string() const;

        /*
         * Update the ROS diagnostics.
         */
        void ros_update(diagnostic_updater::DiagnosticStatusWrapper &dsw);

    private:
        sensors_chip_name   m_name;
        sensors_feature     m_feature;
        std::string         m_label;
        double              m_value;
        bool                m_alarm;

        void    _update_feature_in();
        void    _update_feature_fan();
        void    _update_feature_temp();
        double  _get_value(const sensors_subfeature *sf);

        friend std::ostream &operator<<(std::ostream &stream, const LMSensor &sensor);
};

} // namespace lm_sensors

