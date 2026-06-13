#ifndef BLE_CFG_H
#define BLE_CFG_H

#define LAMP_GATT_SERVICE       0x1815
#define BIG_LAMP_GATT_CHAR      0x0000
#define SMALL_LAMP_GATT_CHAR    0x0001
#define ENABLE_RANGING_CHAR     0x0002

#define DEVICE_MODEL_SERVICE    (0xFFFF)
#define DEVICE_MODEL_CHAR       0x0000

/* Device model */
#define LAMP_NAME               "BedroomLamp"

#define DEVICE_TYPE_SWITCH  "switch"
#define DEVICE_TYPE_TOGGLE  "toggle"

#define BIG_LAMP_NAME           "BigLamp"
#define BIG_LAMP_TYPE           DEVICE_TYPE_SWITCH

#define SMALL_LAMP_NAME         "SmallLamp"
#define SMALL_LAMP_TYPE         DEVICE_TYPE_SWITCH

#define ENABLE_RANGING_NAME     "EnableRanging"
#define ENABLE_RANGING_TYPE     DEVICE_TYPE_TOGGLE

/* Stringify helpers */
#define _STR(x)   #x
#define STR(x)    _STR(x)

#define DEVICE_MODEL_JSON \
"{\"device\":\"" LAMP_NAME "\",\"version\":1,\"service\":\"" STR(LAMP_GATT_SERVICE) "\",\"characteristics\":[" \
"{\"char\":\"" STR(BIG_LAMP_GATT_CHAR) "\",\"name\":\"" BIG_LAMP_NAME "\",\"format\":\"bool\",\"control\":\"switch\"}," \
"{\"char\":\"" STR(SMALL_LAMP_GATT_CHAR) "\",\"name\":\"" SMALL_LAMP_NAME "\",\"format\":\"bool\",\"control\":\"switch\"}," \
"{\"char\":\"" STR(ENABLE_RANGING_CHAR) "\",\"name\":\"" ENABLE_RANGING_NAME "\",\"format\":\"bitfield\",\"control\":\"bitfield\",\"bits\":[" \
"{\"bit\":0,\"name\":\"" BIG_LAMP_NAME "\"}," \
"{\"bit\":1,\"name\":\"" SMALL_LAMP_NAME "\"}" \
"]}" \
"]}"

#endif