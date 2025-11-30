/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_TOFSENSE

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_tofsense.h"

#define TOFSENSE_FRAME_LENGTH    16
#define TOFSENSE_HEADER_BYTE     0x57
#define TOFSENSE_FUNCTION_MARK   0x00
#define TOFSENSE_TIMEOUT_MS      200
#define TOFSENSE_TASK_PERIOD_MS  20 // 50Hz default update rate

#define TOFSENSE_BAUDRATE        115200

static serialPort_t *tofsenseSerialPort = NULL;

typedef enum {
    TOFSENSE_FRAME_STATE_WAIT_HEADER,
    TOFSENSE_FRAME_STATE_WAIT_MARK,
    TOFSENSE_FRAME_STATE_READING_PAYLOAD,
    TOFSENSE_FRAME_STATE_WAIT_CKSUM,
} tofsenseFrameState_e;

static tofsenseFrameState_e tofsenseFrameState;
static uint8_t tofsenseFrame[TOFSENSE_FRAME_LENGTH];
static uint8_t tofsenseReceivePosition;

static int32_t tofsenseDistance = RANGEFINDER_OUT_OF_RANGE;

static void tofsenseInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
    tofsenseFrameState = TOFSENSE_FRAME_STATE_WAIT_HEADER;
    tofsenseReceivePosition = 0;
    tofsenseDistance = RANGEFINDER_OUT_OF_RANGE;
}

static void tofsenseUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);
    static timeMs_t lastFrameReceivedMs = 0;
    const timeMs_t timeNowMs = millis();

    if (tofsenseSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(tofsenseSerialPort)) {
        uint8_t c = serialRead(tofsenseSerialPort);
        
        switch (tofsenseFrameState) {
        case TOFSENSE_FRAME_STATE_WAIT_HEADER:
            if (c == TOFSENSE_HEADER_BYTE) {
                tofsenseFrame[0] = c;
                tofsenseFrameState = TOFSENSE_FRAME_STATE_WAIT_MARK;
            }
            break;

        case TOFSENSE_FRAME_STATE_WAIT_MARK:
            if (c == TOFSENSE_FUNCTION_MARK) {
                tofsenseFrame[1] = c;
                tofsenseReceivePosition = 2;
                tofsenseFrameState = TOFSENSE_FRAME_STATE_READING_PAYLOAD;
            } else {
                tofsenseFrameState = TOFSENSE_FRAME_STATE_WAIT_HEADER;
            }
            break;

        case TOFSENSE_FRAME_STATE_READING_PAYLOAD:
            tofsenseFrame[tofsenseReceivePosition++] = c;
            if (tofsenseReceivePosition == TOFSENSE_FRAME_LENGTH - 1) {
                tofsenseFrameState = TOFSENSE_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case TOFSENSE_FRAME_STATE_WAIT_CKSUM: {
            uint8_t cksum = 0;
            for (int i = 0; i < TOFSENSE_FRAME_LENGTH - 1; i++) {
                cksum += tofsenseFrame[i];
            }

            if (c == cksum) {
                // Distance: 3 bytes at offset 8, 9, 10 (Little Endian 24-bit)
                // Value is in mm.
                uint32_t distMm = tofsenseFrame[8] | (tofsenseFrame[9] << 8) | (tofsenseFrame[10] << 16);
                uint8_t status = tofsenseFrame[11];

                // Status 0 indicates valid measurement
                if (status == 1) {
                    int32_t distCm = distMm / 10;
                    //  if (distCm > 0 && distCm <= dev->maxRangeCm) {
                    tofsenseDistance = distCm;
                    //  } else {
                    //      tofsenseDistance = RANGEFINDER_OUT_OF_RANGE;
                    //  }
                } else {
                    tofsenseDistance = RANGEFINDER_OUT_OF_RANGE;
                }
                
                lastFrameReceivedMs = timeNowMs;
            }
            
            tofsenseFrameState = TOFSENSE_FRAME_STATE_WAIT_HEADER;
            tofsenseReceivePosition = 0;
            break;
        }
        }
    }

    if (cmp32(timeNowMs, lastFrameReceivedMs) > TOFSENSE_TIMEOUT_MS) {
        tofsenseDistance = RANGEFINDER_OUT_OF_RANGE;
    }
}

static int32_t tofsenseGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return tofsenseDistance;
}

bool tofsenseDetect(rangefinderDev_t *dev, rangefinderType_e rfType)
{
    if (rfType != RANGEFINDER_TOFSENSE) {
        return false;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
    if (!portConfig) {
        return false;
    }

    tofsenseSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, TOFSENSE_BAUDRATE, MODE_RXTX, 0);
    if (tofsenseSerialPort == NULL) {
        return false;
    }

    dev->delayMs = TOFSENSE_TASK_PERIOD_MS;
    dev->maxRangeCm = 780; // TOFSense-F2 Mini max range is 7.8m.

    dev->detectionConeDeciDegrees = 900; 
    dev->detectionConeExtendedDeciDegrees = 900;

    dev->init = &tofsenseInit;
    dev->update = &tofsenseUpdate;
    dev->read = &tofsenseGetDistance;

    return true;
}

#endif
