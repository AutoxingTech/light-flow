int16 flow_x_integral
int16 flow_y_integral
uint16 integration_timespan
uint16 ground_distance
uint8 valid
uint8 version

# struct __attribute__((packed)) OpticalFlowPack
# {
#     BottomSensorPacket header;
#     int16_t flow_x_integral;
#     int16_t flow_y_integral;
#     uint16_t integration_timespan; // unit: us
#     uint16_t ground_distance;
#     uint8_t valid; // 0x00 invalid; 0xf5 valid
#     uint8_t version;
# };