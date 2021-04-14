// C++ implementation of the Blue Robotics 'Ping' binary message protocol

//~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!
// THIS IS AN AUTOGENERATED FILE
// DO NOT EDIT
//~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!

#pragma once

#include "pingmessage.h"

class ping_msg_ping1D_ack : public PingMessage
{
public:
    ping_msg_ping1D_ack(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_ack(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_ack()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t acked_id() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_acked_id(const uint16_t acked_id) { memcpy((payload_data(0)), &acked_id, 2.0);};
};

class ping_msg_ping1D_ascii_text : public PingMessage
{
public:
    ping_msg_ping1D_ascii_text(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_ascii_text(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_ascii_text(uint16_t ascii_message_length)
        : PingMessage { static_cast<uint16_t>(10 + ascii_message_length) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 0 + ascii_message_length; // payload size
        (uint16_t&)msgData[4] = 3; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    char* ascii_message() const { return (char*)(payload_data(0)); }
    void set_ascii_message_at(const uint16_t i, const char data) { memcpy((payload_data(0 + i)), &data, 1); }
};

class ping_msg_ping1D_continuous_start : public PingMessage
{
public:
    ping_msg_ping1D_continuous_start(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_continuous_start(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_continuous_start()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1400; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t id() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_id(const uint16_t id) { memcpy((payload_data(0)), &id, 2.0);};
};

class ping_msg_ping1D_continuous_stop : public PingMessage
{
public:
    ping_msg_ping1D_continuous_stop(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_continuous_stop(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_continuous_stop()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1401; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t id() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_id(const uint16_t id) { memcpy((payload_data(0)), &id, 2.0);};
};

class ping_msg_ping1D_device_id : public PingMessage
{
public:
    ping_msg_ping1D_device_id(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_device_id(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_device_id()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1201; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t device_id() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_device_id(const uint8_t device_id) { memcpy((payload_data(0)), &device_id, 1.0);};
};

class ping_msg_ping1D_distance : public PingMessage
{
public:
    ping_msg_ping1D_distance(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_distance(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_distance()
        : PingMessage { static_cast<uint16_t>(34) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 24; // payload size
        (uint16_t&)msgData[4] = 1212; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t distance() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_distance(const uint32_t distance) { memcpy((payload_data(0)), &distance, 4.0);};
    uint16_t confidence() const { uint16_t d; memcpy(&d, (payload_data(4.0)), 2.0); return d; };
    void set_confidence(const uint16_t confidence) { memcpy((payload_data(4.0)), &confidence, 2.0);};
    uint16_t pulse_duration() const { uint16_t d; memcpy(&d, (payload_data(6.0)), 2.0); return d; };
    void set_pulse_duration(const uint16_t pulse_duration) { memcpy((payload_data(6.0)), &pulse_duration, 2.0);};
    uint32_t ping_number() const { uint32_t d; memcpy(&d, (payload_data(8.0)), 4.0); return d; };
    void set_ping_number(const uint32_t ping_number) { memcpy((payload_data(8.0)), &ping_number, 4.0);};
    uint32_t scan_start() const { uint32_t d; memcpy(&d, (payload_data(12.0)), 4.0); return d; };
    void set_scan_start(const uint32_t scan_start) { memcpy((payload_data(12.0)), &scan_start, 4.0);};
    uint32_t scan_length() const { uint32_t d; memcpy(&d, (payload_data(16.0)), 4.0); return d; };
    void set_scan_length(const uint32_t scan_length) { memcpy((payload_data(16.0)), &scan_length, 4.0);};
    uint32_t gain_index() const { uint32_t d; memcpy(&d, (payload_data(20.0)), 4.0); return d; };
    void set_gain_index(const uint32_t gain_index) { memcpy((payload_data(20.0)), &gain_index, 4.0);};
};

class ping_msg_ping1D_distance_simple : public PingMessage
{
public:
    ping_msg_ping1D_distance_simple(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_distance_simple(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_distance_simple()
        : PingMessage { static_cast<uint16_t>(15) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 5; // payload size
        (uint16_t&)msgData[4] = 1211; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t distance() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_distance(const uint32_t distance) { memcpy((payload_data(0)), &distance, 4.0);};
    uint8_t confidence() const { uint8_t d; memcpy(&d, (payload_data(4.0)), 1.0); return d; };
    void set_confidence(const uint8_t confidence) { memcpy((payload_data(4.0)), &confidence, 1.0);};
};

class ping_msg_ping1D_firmware_version : public PingMessage
{
public:
    ping_msg_ping1D_firmware_version(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_firmware_version(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_firmware_version()
        : PingMessage { static_cast<uint16_t>(16) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 6; // payload size
        (uint16_t&)msgData[4] = 1200; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t device_type() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_device_type(const uint8_t device_type) { memcpy((payload_data(0)), &device_type, 1.0);};
    uint8_t device_model() const { uint8_t d; memcpy(&d, (payload_data(1.0)), 1.0); return d; };
    void set_device_model(const uint8_t device_model) { memcpy((payload_data(1.0)), &device_model, 1.0);};
    uint16_t firmware_version_major() const { uint16_t d; memcpy(&d, (payload_data(2.0)), 2.0); return d; };
    void set_firmware_version_major(const uint16_t firmware_version_major) { memcpy((payload_data(2.0)), &firmware_version_major, 2.0);};
    uint16_t firmware_version_minor() const { uint16_t d; memcpy(&d, (payload_data(4.0)), 2.0); return d; };
    void set_firmware_version_minor(const uint16_t firmware_version_minor) { memcpy((payload_data(4.0)), &firmware_version_minor, 2.0);};
};

class ping_msg_ping1D_gain_index : public PingMessage
{
public:
    ping_msg_ping1D_gain_index(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_gain_index(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_gain_index()
        : PingMessage { static_cast<uint16_t>(14) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 4; // payload size
        (uint16_t&)msgData[4] = 1207; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t gain_index() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_gain_index(const uint32_t gain_index) { memcpy((payload_data(0)), &gain_index, 4.0);};
};

class ping_msg_ping1D_general_info : public PingMessage
{
public:
    ping_msg_ping1D_general_info(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_general_info(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_general_info()
        : PingMessage { static_cast<uint16_t>(20) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 10; // payload size
        (uint16_t&)msgData[4] = 1210; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t firmware_version_major() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_firmware_version_major(const uint16_t firmware_version_major) { memcpy((payload_data(0)), &firmware_version_major, 2.0);};
    uint16_t firmware_version_minor() const { uint16_t d; memcpy(&d, (payload_data(2.0)), 2.0); return d; };
    void set_firmware_version_minor(const uint16_t firmware_version_minor) { memcpy((payload_data(2.0)), &firmware_version_minor, 2.0);};
    uint16_t voltage_5() const { uint16_t d; memcpy(&d, (payload_data(4.0)), 2.0); return d; };
    void set_voltage_5(const uint16_t voltage_5) { memcpy((payload_data(4.0)), &voltage_5, 2.0);};
    uint16_t ping_interval() const { uint16_t d; memcpy(&d, (payload_data(6.0)), 2.0); return d; };
    void set_ping_interval(const uint16_t ping_interval) { memcpy((payload_data(6.0)), &ping_interval, 2.0);};
    uint8_t gain_index() const { uint8_t d; memcpy(&d, (payload_data(8.0)), 1.0); return d; };
    void set_gain_index(const uint8_t gain_index) { memcpy((payload_data(8.0)), &gain_index, 1.0);};
    uint8_t mode_auto() const { uint8_t d; memcpy(&d, (payload_data(9.0)), 1.0); return d; };
    void set_mode_auto(const uint8_t mode_auto) { memcpy((payload_data(9.0)), &mode_auto, 1.0);};
};

class ping_msg_ping1D_goto_bootloader : public PingMessage
{
public:
    ping_msg_ping1D_goto_bootloader(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_goto_bootloader(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_goto_bootloader()
        : PingMessage { static_cast<uint16_t>(10) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 0; // payload size
        (uint16_t&)msgData[4] = 1100; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

};

class ping_msg_ping1D_mode_auto : public PingMessage
{
public:
    ping_msg_ping1D_mode_auto(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_mode_auto(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_mode_auto()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1205; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t mode_auto() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_mode_auto(const uint8_t mode_auto) { memcpy((payload_data(0)), &mode_auto, 1.0);};
};

class ping_msg_ping1D_nack : public PingMessage
{
public:
    ping_msg_ping1D_nack(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_nack(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_nack(uint16_t nack_message_length)
        : PingMessage { static_cast<uint16_t>(12 + nack_message_length) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2 + nack_message_length; // payload size
        (uint16_t&)msgData[4] = 2; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t nacked_id() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_nacked_id(const uint16_t nacked_id) { memcpy((payload_data(0)), &nacked_id, 2.0);};
    char* nack_message() const { return (char*)(payload_data(2.0)); }
    void set_nack_message_at(const uint16_t i, const char data) { memcpy((payload_data(2.0 + i)), &data, 1); }
};

class ping_msg_ping1D_pcb_temperature : public PingMessage
{
public:
    ping_msg_ping1D_pcb_temperature(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_pcb_temperature(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_pcb_temperature()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1214; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t pcb_temperature() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_pcb_temperature(const uint16_t pcb_temperature) { memcpy((payload_data(0)), &pcb_temperature, 2.0);};
};

class ping_msg_ping1D_ping_enable : public PingMessage
{
public:
    ping_msg_ping1D_ping_enable(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_ping_enable(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_ping_enable()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1215; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t ping_enabled() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_ping_enabled(const uint8_t ping_enabled) { memcpy((payload_data(0)), &ping_enabled, 1.0);};
};

class ping_msg_ping1D_ping_interval : public PingMessage
{
public:
    ping_msg_ping1D_ping_interval(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_ping_interval(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_ping_interval()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1206; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t ping_interval() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_ping_interval(const uint16_t ping_interval) { memcpy((payload_data(0)), &ping_interval, 2.0);};
};

class ping_msg_ping1D_processor_temperature : public PingMessage
{
public:
    ping_msg_ping1D_processor_temperature(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_processor_temperature(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_processor_temperature()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1213; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t processor_temperature() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_processor_temperature(const uint16_t processor_temperature) { memcpy((payload_data(0)), &processor_temperature, 2.0);};
};

class ping_msg_ping1D_profile : public PingMessage
{
public:
    ping_msg_ping1D_profile(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_profile(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_profile(uint16_t profile_data_length
)
        : PingMessage { static_cast<uint16_t>(36 + profile_data_length) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 26 + profile_data_length; // payload size
        (uint16_t&)msgData[4] = 1300; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t distance() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_distance(const uint32_t distance) { memcpy((payload_data(0)), &distance, 4.0);};
    uint16_t confidence() const { uint16_t d; memcpy(&d, (payload_data(4.0)), 2.0); return d; };
    void set_confidence(const uint16_t confidence) { memcpy((payload_data(4.0)), &confidence, 2.0);};
    uint16_t pulse_duration() const { uint16_t d; memcpy(&d, (payload_data(6.0)), 2.0); return d; };
    void set_pulse_duration(const uint16_t pulse_duration) { memcpy((payload_data(6.0)), &pulse_duration, 2.0);};
    uint32_t ping_number() const { uint32_t d; memcpy(&d, (payload_data(8.0)), 4.0); return d; };
    void set_ping_number(const uint32_t ping_number) { memcpy((payload_data(8.0)), &ping_number, 4.0);};
    uint32_t scan_start() const { uint32_t d; memcpy(&d, (payload_data(12.0)), 4.0); return d; };
    void set_scan_start(const uint32_t scan_start) { memcpy((payload_data(12.0)), &scan_start, 4.0);};
    uint32_t scan_length() const { uint32_t d; memcpy(&d, (payload_data(16.0)), 4.0); return d; };
    void set_scan_length(const uint32_t scan_length) { memcpy((payload_data(16.0)), &scan_length, 4.0);};
    uint32_t gain_index() const { uint32_t d; memcpy(&d, (payload_data(20.0)), 4.0); return d; };
    void set_gain_index(const uint32_t gain_index) { memcpy((payload_data(20.0)), &gain_index, 4.0);};
    uint16_t profile_data_length() const { uint16_t d; memcpy(&d, (payload_data(24.0)), 2.0); return d; };
    //TODO do this in constructor (const)
    void set_profile_data_length(const uint16_t profile_data_length) { memcpy((payload_data(24.0)), &profile_data_length, 2.0);};
    uint8_t* profile_data() const { return (uint8_t*)(payload_data(26.0)); }
    void set_profile_data_at(const uint16_t i, const uint8_t data) { memcpy((payload_data(26.0 + i)), &data, 1); }
};

class ping_msg_ping1D_protocol_version : public PingMessage
{
public:
    ping_msg_ping1D_protocol_version(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_protocol_version(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_protocol_version()
        : PingMessage { static_cast<uint16_t>(14) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 4; // payload size
        (uint16_t&)msgData[4] = 5; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t protocol_version() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_protocol_version(const uint32_t protocol_version) { memcpy((payload_data(0)), &protocol_version, 4.0);};
};

class ping_msg_ping1D_pulse_duration : public PingMessage
{
public:
    ping_msg_ping1D_pulse_duration(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_pulse_duration(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_pulse_duration()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1208; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t pulse_duration() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_pulse_duration(const uint16_t pulse_duration) { memcpy((payload_data(0)), &pulse_duration, 2.0);};
};

class ping_msg_ping1D_range : public PingMessage
{
public:
    ping_msg_ping1D_range(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_range(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_range()
        : PingMessage { static_cast<uint16_t>(18) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 8; // payload size
        (uint16_t&)msgData[4] = 1204; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t scan_start() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_scan_start(const uint32_t scan_start) { memcpy((payload_data(0)), &scan_start, 4.0);};
    uint32_t scan_length() const { uint32_t d; memcpy(&d, (payload_data(4.0)), 4.0); return d; };
    void set_scan_length(const uint32_t scan_length) { memcpy((payload_data(4.0)), &scan_length, 4.0);};
};

class ping_msg_ping1D_set_device_id : public PingMessage
{
public:
    ping_msg_ping1D_set_device_id(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_device_id(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_device_id()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1000; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t device_id() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_device_id(const uint8_t device_id) { memcpy((payload_data(0)), &device_id, 1.0);};
};

class ping_msg_ping1D_set_gain_index : public PingMessage
{
public:
    ping_msg_ping1D_set_gain_index(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_gain_index(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_gain_index()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1005; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t gain_index() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_gain_index(const uint8_t gain_index) { memcpy((payload_data(0)), &gain_index, 1.0);};
};

class ping_msg_ping1D_set_mode_auto : public PingMessage
{
public:
    ping_msg_ping1D_set_mode_auto(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_mode_auto(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_mode_auto()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1003; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t mode_auto() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_mode_auto(const uint8_t mode_auto) { memcpy((payload_data(0)), &mode_auto, 1.0);};
};

class ping_msg_ping1D_set_ping_enable : public PingMessage
{
public:
    ping_msg_ping1D_set_ping_enable(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_ping_enable(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_ping_enable()
        : PingMessage { static_cast<uint16_t>(11) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 1; // payload size
        (uint16_t&)msgData[4] = 1006; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t ping_enabled() const { uint8_t d; memcpy(&d, (payload_data(0)), 1.0); return d; };
    void set_ping_enabled(const uint8_t ping_enabled) { memcpy((payload_data(0)), &ping_enabled, 1.0);};
};

class ping_msg_ping1D_set_ping_interval : public PingMessage
{
public:
    ping_msg_ping1D_set_ping_interval(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_ping_interval(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_ping_interval()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1004; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t ping_interval() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_ping_interval(const uint16_t ping_interval) { memcpy((payload_data(0)), &ping_interval, 2.0);};
};

class ping_msg_ping1D_set_range : public PingMessage
{
public:
    ping_msg_ping1D_set_range(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_range(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_range()
        : PingMessage { static_cast<uint16_t>(18) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 8; // payload size
        (uint16_t&)msgData[4] = 1001; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t scan_start() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_scan_start(const uint32_t scan_start) { memcpy((payload_data(0)), &scan_start, 4.0);};
    uint32_t scan_length() const { uint32_t d; memcpy(&d, (payload_data(4.0)), 4.0); return d; };
    void set_scan_length(const uint32_t scan_length) { memcpy((payload_data(4.0)), &scan_length, 4.0);};
};

class ping_msg_ping1D_set_speed_of_sound : public PingMessage
{
public:
    ping_msg_ping1D_set_speed_of_sound(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_set_speed_of_sound(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_set_speed_of_sound()
        : PingMessage { static_cast<uint16_t>(14) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 4; // payload size
        (uint16_t&)msgData[4] = 1002; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t speed_of_sound() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_speed_of_sound(const uint32_t speed_of_sound) { memcpy((payload_data(0)), &speed_of_sound, 4.0);};
};

class ping_msg_ping1D_speed_of_sound : public PingMessage
{
public:
    ping_msg_ping1D_speed_of_sound(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_speed_of_sound(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_speed_of_sound()
        : PingMessage { static_cast<uint16_t>(14) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 4; // payload size
        (uint16_t&)msgData[4] = 1203; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint32_t speed_of_sound() const { uint32_t d; memcpy(&d, (payload_data(0)), 4.0); return d; };
    void set_speed_of_sound(const uint32_t speed_of_sound) { memcpy((payload_data(0)), &speed_of_sound, 4.0);};
};

class ping_msg_ping1D_undefined : public PingMessage
{
public:
    ping_msg_ping1D_undefined(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_undefined(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_undefined()
        : PingMessage { static_cast<uint16_t>(10) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 0; // payload size
        (uint16_t&)msgData[4] = 0; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

};

class ping_msg_ping1D_voltage_5 : public PingMessage
{
public:
    ping_msg_ping1D_voltage_5(const PingMessage& msg) : PingMessage { msg } {}
    ping_msg_ping1D_voltage_5(const uint8_t* buf, const uint16_t length) : PingMessage { buf, length } {}
    ping_msg_ping1D_voltage_5()
        : PingMessage { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = 2; // payload size
        (uint16_t&)msgData[4] = 1202; // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t voltage_5() const { uint16_t d; memcpy(&d, (payload_data(0)), 2.0); return d; };
    void set_voltage_5(const uint16_t voltage_5) { memcpy((payload_data(0)), &voltage_5, 2.0);};
};

