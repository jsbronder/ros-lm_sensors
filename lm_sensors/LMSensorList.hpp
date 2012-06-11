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

#include "LMSensor.hpp"
#include <vector>

namespace lm_sensors {

class LMSensorList {
    public:
        typedef std::vector<LMSensor *>::const_iterator const_iterator;
        typedef std::vector<LMSensor *>::iterator iterator;;

        /*
         * Constructor.  Calls sensors_init() and create a LMSensor
         * for each detected sensor on the system.
         */
        explicit LMSensorList();

        /*
         * Destructor.  Calls sensors_cleanup() once all created
         * LMSensors are destroyed.
         */
        ~LMSensorList();

        /* 
         * Read latest values from all sensors
         */
        void update();

        /*
         * @returns -   a const iterator to the beginning of the sensor list
         */
        const_iterator begin() const;

        /*
         * @returns -   a const iterator to the end of the sensor list
         */
        const_iterator end() const;

    private:
        std::vector<LMSensor *>  m_sensors;
};

} // namespace lm_sensors
