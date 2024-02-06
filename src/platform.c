#include "platform.h"

#include <string.h>
#include <nrf.h>

static const char *defaultDeviceType = "0;CF20;R=D";

static char *deviceTypeStringLocation = (void*)PLATFORM_DEVICE_DATA_FLASH_POS;

static bool has_rfx2411n = false;

void platformGetDeviceTypeString(char *deviceTypeString)
{
  if (deviceTypeStringLocation[0] == 0xffu) {
    strncpy(deviceTypeString, defaultDeviceType, 32);
    deviceTypeString[32] = 0;
  } else {
    strncpy(deviceTypeString, deviceTypeStringLocation, 32);
    deviceTypeString[32] = 0;
  }
}

/**
 * Parse deviceType string to extract the deviceType
 *
 * Ignores the key=value sections.
 *
 * \param [in] deviceTypeString deviceTypeString extracted from the hardware
 * \param [out] deviceType Buffer of at least PLATFORM_DEVICE_TYPE_MAX_LEN
 *                         bytes where the device type will be stored
 * \return 0 in case of success, 1 in case of failure.
 */
static int platformParseDeviceTypeString(char* deviceTypeString, char* deviceType) {
  // char *state;

  memcpy(deviceType, deviceTypeString+2, 4);
  deviceType[4]  = '\0';
  
  return 0;
}

// This function configures the platform in runtime based on the device type.
// The main reason to not move it into the platform files is that if a user
// flashes the wrong binary in the NRF we still want it to start up.
int platformInitByDeviceType() {
  static char deviceTypeString[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];
  static char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];

  platformGetDeviceTypeString(deviceTypeString);
  if (platformParseDeviceTypeString(deviceTypeString, deviceType) != 0) {
    return 1;
  }

  if (0 == strcmp(deviceType, "CF20")) {
    has_rfx2411n = false;

  } else if (0 == strcmp(deviceType, "CF21")) {
    has_rfx2411n = true;

  } else if (0 == strcmp(deviceType, "RR10")) {
    has_rfx2411n = true;

  } else if ((0 == strcmp(deviceType, "RZ10")) ||
             (0 == strcmp(deviceType, "CB10")) ||
             (0 == strcmp(deviceType, "CB11"))) {
    has_rfx2411n = true;

  } else {
    has_rfx2411n = false;
  }

  return 0;
}

bool platformHasRfx2411n() {
  return has_rfx2411n;
}
