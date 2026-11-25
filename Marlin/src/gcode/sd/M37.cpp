/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * M37: Stream a pre-filtered, decimated 3D toolpath preview from onboard SD
 *      to a connected TFT touchscreen without disturbing active print state.
 */

#include "../../inc/MarlinConfig.h"

#if HAS_MEDIA

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../HAL/shared/Delay.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint16_t M37_MAX_PREVIEW_VERTS = 4400;
constexpr uint16_t M37_HARD_CAP_VERTS    = 4560;
constexpr uint8_t  M37_MAX_Z_LEVELS      = 32;
constexpr float    M37_ARC_STEP_RAD      = 0.25f;
constexpr int      M37_MAX_ARC_SEGS      = 32;
constexpr float    M37_PI                = 3.14159265f;
constexpr uint8_t  M37_VERTS_PER_LINE    = 8;

enum M37SegType : uint8_t {
  M37_SEG_START  = 0,
  M37_SEG_TRAVEL = 1,
  M37_SEG_PLUNGE = 2,
  M37_SEG_CUT    = 3
};

struct M37Vec3 {
  float x, y, z;
};

struct M37Stats {
  bool hasBounds;
  float minX, maxX, minY, maxY, minZ, maxZ;
  float cutDistMm;
  float travelDistMm;
  float estTimeSec;
  float zLevels[M37_MAX_Z_LEVELS];
  uint16_t zLevelCount;
  uint32_t moveLineCount;
  uint32_t totalVerts;
  M37Vec3 startPt;
  M37Vec3 endPt;
};

struct M37Emitter {
  bool emit;
  uint32_t stride;
  uint32_t totalVerts;
  uint32_t vertIdx;
  uint16_t emittedCount;
  uint8_t lastEmittedType;
  uint8_t batchCount;

  void flushLine() {
    if (batchCount > 0) {
      SERIAL_EOL();
      batchCount = 0;
      hal.watchdog_refresh();
    }
  }

  void pushVertex(const M37Vec3 &pt, const uint8_t segType) {
    const uint32_t idx = vertIdx++;
    if (!emit) return;
    if (emittedCount >= M37_HARD_CAP_VERTS) return;

    const bool isFirst      = (idx == 0);
    const bool isLast       = (totalVerts > 0 && idx + 1 >= totalVerts);
    const bool typeChanged  = (!isFirst && segType != lastEmittedType);
    const bool onStride     = (stride <= 1) || ((idx % stride) == 0);
    const bool allowType    = typeChanged && (emittedCount < M37_MAX_PREVIEW_VERTS);

    if (!(isFirst || isLast || onStride || allowType)) return;

    if (batchCount == 0) {
      SERIAL_ECHOPGM("V ");
    } else {
      SERIAL_CHAR(';');
    }

    const int32_t xi = (int32_t)lroundf(pt.x * 100.0f);
    const int32_t yi = (int32_t)lroundf(pt.y * 100.0f);
    const int32_t zi = (int32_t)lroundf(pt.z * 100.0f);

    SERIAL_ECHO((int)segType);
    SERIAL_CHAR(',');
    SERIAL_ECHO(xi);
    SERIAL_CHAR(',');
    SERIAL_ECHO(yi);
    SERIAL_CHAR(',');
    SERIAL_ECHO(zi);

    emittedCount++;
    lastEmittedType = segType;
    batchCount++;
    if (batchCount >= M37_VERTS_PER_LINE) {
      flushLine();
    }
  }
};

static void m37RecordBounds(M37Stats &st, const M37Vec3 &pt) {
  if (!st.hasBounds) {
    st.minX = st.maxX = pt.x;
    st.minY = st.maxY = pt.y;
    st.minZ = st.maxZ = pt.z;
    st.hasBounds = true;
  } else {
    if (pt.x < st.minX) st.minX = pt.x;
    if (pt.x > st.maxX) st.maxX = pt.x;
    if (pt.y < st.minY) st.minY = pt.y;
    if (pt.y > st.maxY) st.maxY = pt.y;
    if (pt.z < st.minZ) st.minZ = pt.z;
    if (pt.z > st.maxZ) st.maxZ = pt.z;
  }
}

static void m37RecordZLevel(M37Stats &st, const float z) {
  for (uint16_t i = 0; i < st.zLevelCount; ++i) {
    if (fabsf(st.zLevels[i] - z) < 0.05f) return;
  }
  if (st.zLevelCount < M37_MAX_Z_LEVELS) {
    st.zLevels[st.zLevelCount++] = z;
  }
}

struct M37ParserState {
  M37Vec3 curPos;
  float curFeed;
  float unitScale;
  int8_t modalG;
  bool relMode;
  bool firstPoint;
};

static void m37ProcessLine(char *line, M37ParserState &ps, M37Stats *st, M37Emitter &em) {
  char *p = line;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0') return;

  // Skip optional line number N<digits>
  if ((*p == 'N' || *p == 'n') && p[1] >= '0' && p[1] <= '9') {
    ++p;
    while (*p >= '0' && *p <= '9') ++p;
    while (*p == ' ' || *p == '\t') ++p;
  }

  int8_t lineG = -1;
  bool hasX = false, hasY = false, hasZ = false, hasI = false, hasJ = false;
  float valX = 0.0f, valY = 0.0f, valZ = 0.0f, valI = 0.0f, valJ = 0.0f;

  while (*p != '\0') {
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') break;
    char code = *p++;
    if (code >= 'a' && code <= 'z') code -= 32;

    char *endPtr = nullptr;
    if (code == 'G') {
      const long gnum = strtol(p, &endPtr, 10);
      if (endPtr != p) {
        if (gnum >= 0 && gnum <= 3) {
          lineG = (int8_t)gnum;
          ps.modalG = lineG;
        } else if (gnum == 20) {
          ps.unitScale = 25.4f;
        } else if (gnum == 21) {
          ps.unitScale = 1.0f;
        } else if (gnum == 90) {
          ps.relMode = false;
        } else if (gnum == 91) {
          ps.relMode = true;
        } else if (gnum == 92) {
          // Ignore coordinate offset lines as motion
          return;
        }
        p = endPtr;
      }
    } else if (code == 'M' || code == 'T') {
      // Non-motion command on this line
      return;
    } else if (code == 'X' || code == 'Y' || code == 'Z' || code == 'I' || code == 'J' || code == 'F') {
      const float val = strtof(p, &endPtr);
      if (endPtr != p) {
        if (code == 'X')      { hasX = true; valX = val * ps.unitScale; }
        else if (code == 'Y') { hasY = true; valY = val * ps.unitScale; }
        else if (code == 'Z') { hasZ = true; valZ = val * ps.unitScale; }
        else if (code == 'I') { hasI = true; valI = val * ps.unitScale; }
        else if (code == 'J') { hasJ = true; valJ = val * ps.unitScale; }
        else if (code == 'F') { const float f = val * ps.unitScale; if (f > 1.0f) ps.curFeed = f; }
        p = endPtr;
      }
    } else {
      // Skip unknown word
      while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
    }
  }

  if (!(hasX || hasY || hasZ || hasI || hasJ)) return;

  const int8_t gcode = (lineG >= 0) ? lineG : ps.modalG;
  if (gcode < 0 || gcode > 3) return;

  const M37Vec3 prevPos = ps.curPos;
  M37Vec3 newPos = ps.curPos;
  if (hasX) newPos.x = ps.relMode ? (ps.curPos.x + valX) : valX;
  if (hasY) newPos.y = ps.relMode ? (ps.curPos.y + valY) : valY;
  if (hasZ) newPos.z = ps.relMode ? (ps.curPos.z + valZ) : valZ;
  ps.curPos = newPos;

  if (st != nullptr) st->moveLineCount++;

  if (ps.firstPoint) {
    ps.firstPoint = false;
    if (st != nullptr) {
      st->startPt = newPos;
      st->endPt   = newPos;
    }
    em.pushVertex(newPos, M37_SEG_START);
    return;
  }

  if (st != nullptr) st->endPt = newPos;

  const bool isTravel = (gcode == 0) || (prevPos.z > 0.01f && newPos.z > 0.01f);
  const bool isPlunge = !isTravel && (newPos.z < prevPos.z - 0.001f);
  const uint8_t segType = isTravel ? M37_SEG_TRAVEL : (isPlunge ? M37_SEG_PLUNGE : M37_SEG_CUT);

  if (st != nullptr) {
    if (segType == M37_SEG_CUT && newPos.z <= 0.01f) {
      m37RecordZLevel(*st, newPos.z);
    }
    if (!isTravel) {
      m37RecordBounds(*st, prevPos);
      m37RecordBounds(*st, newPos);
    }
  }

  if (gcode == 0 || gcode == 1) {
    if (st != nullptr) {
      const float dx = newPos.x - prevPos.x;
      const float dy = newPos.y - prevPos.y;
      const float dz = newPos.z - prevPos.z;
      const float segLen = sqrtf(dx * dx + dy * dy + dz * dz);
      if (isTravel) {
        st->travelDistMm += segLen;
        const float rapidFeed = (ps.curFeed > 100.0f) ? (ps.curFeed * 1.5f) : 1500.0f;
        st->estTimeSec += (segLen / rapidFeed) * 60.0f;
      } else {
        st->cutDistMm += segLen;
        const float feed = (ps.curFeed > 1.0f) ? ps.curFeed : 600.0f;
        st->estTimeSec += (segLen / feed) * 60.0f;
      }
    }
    em.pushVertex(newPos, segType);
    return;
  }

  // G2 / G3 Arc tessellation
  const float radius = sqrtf(valI * valI + valJ * valJ);
  if (radius <= 1e-6f) {
    em.pushVertex(newPos, segType);
    return;
  }
  const float cx = prevPos.x + valI;
  const float cy = prevPos.y + valJ;
  const float startAngle = atan2f(prevPos.y - cy, prevPos.x - cx);
  const float endAngle   = atan2f(newPos.y - cy, newPos.x - cx);
  float sweep = endAngle - startAngle;
  if (gcode == 2) {
    if (sweep >= 0.0f) sweep -= 2.0f * M37_PI;
  } else {
    if (sweep <= 0.0f) sweep += 2.0f * M37_PI;
  }

  if (st != nullptr) {
    const float arcLen = radius * fabsf(sweep);
    const float dz = newPos.z - prevPos.z;
    const float segLen = sqrtf(arcLen * arcLen + dz * dz);
    if (isTravel) {
      st->travelDistMm += segLen;
      const float rapidFeed = (ps.curFeed > 100.0f) ? (ps.curFeed * 1.5f) : 1500.0f;
      st->estTimeSec += (segLen / rapidFeed) * 60.0f;
    } else {
      st->cutDistMm += segLen;
      const float feed = (ps.curFeed > 1.0f) ? ps.curFeed : 600.0f;
      st->estTimeSec += (segLen / feed) * 60.0f;
    }
  }

  int nSegs = (int)(fabsf(sweep) / M37_ARC_STEP_RAD) + 1;
  if (nSegs > M37_MAX_ARC_SEGS) nSegs = M37_MAX_ARC_SEGS;
  const float angleStep = sweep / (float)nSegs;

  for (int k = 0; k < nSegs; ++k) {
    M37Vec3 pt;
    if (k == nSegs - 1) {
      pt = newPos;
    } else {
      const float ang = startAngle + (float)(k + 1) * angleStep;
      pt.x = cx + radius * cosf(ang);
      pt.y = cy + radius * sinf(ang);
      pt.z = prevPos.z + (newPos.z - prevPos.z) * (float)(k + 1) / (float)nSegs;
    }
    if (st != nullptr && !isTravel) {
      m37RecordBounds(*st, pt);
    }
    em.pushVertex(pt, segType);
  }
}

static void m37RunPass(MediaFile &file, M37Stats *st, M37Emitter &em, const bool sendHeartbeat) {
  file.seekSet(0);

  M37ParserState ps{};
  ps.curPos     = {0.0f, 0.0f, 0.0f};
  ps.curFeed    = 600.0f;
  ps.unitScale  = 1.0f;
  ps.modalG     = -1;
  ps.relMode    = false;
  ps.firstPoint = true;

  uint8_t readBuf[512];
  char line[128];
  uint8_t lineLen = 0;
  bool inSemiComment = false;
  uint8_t parenDepth = 0;
  uint32_t bytesSinceWdt = 0;
  millis_t nextHeartbeatMs = millis() + 500;

  while (true) {
    const int16_t br = file.read(readBuf, sizeof(readBuf));
    if (br <= 0) break;

    bytesSinceWdt += (uint32_t)br;
    if (bytesSinceWdt >= 16384U) {
      bytesSinceWdt = 0;
      hal.watchdog_refresh();
      if (sendHeartbeat && ELAPSED(millis(), nextHeartbeatMs)) {
        nextHeartbeatMs = millis() + 500;
        SERIAL_ECHOLNPGM("#P");
      }
    }

    for (int16_t i = 0; i < br; ++i) {
      const char c = (char)readBuf[i];
      if (c == '\n') {
        line[lineLen] = '\0';
        if (lineLen > 0) m37ProcessLine(line, ps, st, em);
        lineLen = 0;
        inSemiComment = false;
        parenDepth = 0;
      } else if (c == '\r') {
        // ignore CR
      } else if (c == ';') {
        inSemiComment = true;
      } else if (!inSemiComment) {
        if (c == '(') {
          parenDepth++;
        } else if (c == ')') {
          if (parenDepth > 0) parenDepth--;
        } else if (parenDepth == 0 && lineLen < sizeof(line) - 1) {
          line[lineLen++] = c;
        }
      }
    }
  }

  if (lineLen > 0) {
    line[lineLen] = '\0';
    m37ProcessLine(line, ps, st, em);
  }
  em.flushLine();
}

} // anonymous namespace

/**
 * M37 <filepath>: Stream compact 3D toolpath preview from onboard SD card
 */
void GcodeSuite::M37() {
  if (card.isPrinting()) {
    SERIAL_ECHOLNPGM("Preview error: busy");
    return;
  }

  char *path = parser.string_arg;
  if (!path || !*path) {
    SERIAL_ECHOLNPGM("Preview error: no file");
    return;
  }
  while (*path == ' ') ++path;
  // Strip trailing spaces / comments
  for (char *p = path; *p; ++p) {
    if (*p == ' ' || *p == '\r' || *p == '\n') {
      *p = '\0';
      break;
    }
  }

  MediaFile previewFile;
  if (!card.openFileReadSilent(previewFile, path)) {
    SERIAL_ECHOLNPGM("Preview error: open failed");
    return;
  }

  SERIAL_ECHOLNPGM("Begin preview");

  // Pass 1: Collect full-resolution stats & total vertex count
  M37Stats stats{};
  M37Emitter countEmitter{};
  countEmitter.emit = false;
  m37RunPass(previewFile, &stats, countEmitter, true);

  if (!stats.hasBounds && stats.moveLineCount > 0) {
    m37RecordBounds(stats, stats.startPt);
    m37RecordBounds(stats, stats.endPt);
  }
  stats.totalVerts = countEmitter.vertIdx;

  // Send stats summary line
  SERIAL_ECHOPGM("S ");
  SERIAL_ECHO(stats.hasBounds ? 1 : 0);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.minX);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.maxX);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.minY);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.maxY);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.minZ);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.maxZ);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.cutDistMm);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.travelDistMm);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.estTimeSec);
  SERIAL_CHAR(' '); SERIAL_ECHO((int)stats.zLevelCount);
  SERIAL_CHAR(' '); SERIAL_ECHO(stats.moveLineCount);
  SERIAL_EOL();

  // Pass 2: Emit compact vertices (auto-decimated to fit TFT CCMRAM cache)
  if (stats.totalVerts > 0) {
    M37Emitter streamEmitter{};
    streamEmitter.emit = true;
    streamEmitter.totalVerts = stats.totalVerts;
    streamEmitter.stride = (stats.totalVerts > M37_MAX_PREVIEW_VERTS)
                             ? ((stats.totalVerts + M37_MAX_PREVIEW_VERTS - 1U) / M37_MAX_PREVIEW_VERTS)
                             : 1U;
    streamEmitter.lastEmittedType = 0xFF;
    m37RunPass(previewFile, nullptr, streamEmitter, false);
  }

  previewFile.close();
  SERIAL_ECHOLNPGM("End preview");
}

#endif // HAS_MEDIA
