/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(AUTO_REPORT_POSITION)

#include "../gcode.h"
#include "../../module/motion.h"

/**
 * M154: Set position auto-report interval.
 *   S<seconds>      - Interval in seconds (supports decimals, e.g. S0.5 for 500 ms; 0 = off)
 *   P<milliseconds> - Interval in milliseconds (e.g. P500 for 500 ms; takes precedence over S)
 */
void GcodeSuite::M154() {

  if (parser.seenval('P')) {
    const uint16_t ms = parser.value_ushort();
    position_auto_reporter.set_interval_ms(ms ? _MAX(ms, (uint16_t)100) : 0);
  }
  else if (parser.seenval('S')) {
    const float sec = parser.value_float();
    if (sec <= 0.0f)
      position_auto_reporter.set_interval_ms(0);
    else {
      const uint32_t ms = (uint32_t)(sec * 1000.0f + 0.5f);
      position_auto_reporter.set_interval_ms((uint16_t)_MIN(_MAX(ms, 100UL), 60000UL));
    }
  }

}

#endif // AUTO_REPORT_POSITION
