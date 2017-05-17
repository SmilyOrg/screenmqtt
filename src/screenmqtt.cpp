#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <conio.h>

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <bitset>
#include <algorithm>
#include <string>
#include <functional>
#include <sstream>
#include <thread>
#include <condition_variable>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winnt.h>
#include <tchar.h>
#include "PhysicalMonitorEnumerationAPI.h"
#include "LowLevelMonitorConfigurationAPI.h"
#include "HighLevelMonitorConfigurationAPI.h"

#include "MQTTClient.h"
#include "yaml-cpp/yaml.h"

const std::string name = "screenmqtt 0.3";



#define CHECK(call) \
    if (!call) { \
        std::cerr << "Call to " #call " failed" << std::endl; \
        return true; \
    } \

#define MCHECK(call) { \
    int rc = call; \
    if (rc != MQTTCLIENT_SUCCESS) { \
        std::cerr << "Call to " #call " failed (" << rc << ")" << std::endl; \
    } \
}

std::mutex mutex;
std::condition_variable cv;
bool done = false;
std::string topic_all_power_state;
std::string topic_all_power_command;
HWND window;


int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void mqtt_delivered(void *context, MQTTClient_deliveryToken dt);
void mqtt_disconnected(void *context, char *cause);
void receive_monitors(const std::string &topic, const std::string &payload);

class MessageQueue {

    MQTTClient client = nullptr;
    int qos = 0;
    int timeout = 10000;

public:

    std::string topic_prefix;

    ~MessageQueue() {
        disconnect();
    }

    void connect(
        const std::string uri,
        const std::string client_id,
        const std::string topic_prefix,
        MQTTClient_connectOptions opts = MQTTClient_connectOptions_initializer
    ) {
        opts.cleansession = true;

        this->topic_prefix = topic_prefix;

        MCHECK(MQTTClient_create(&client, uri.c_str(), client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr));
        MCHECK(MQTTClient_setCallbacks(client, this, mqtt_disconnected, mqtt_received, mqtt_delivered));
        MCHECK(MQTTClient_connect(client, &opts));

        printf("Topic prefix: %s\nClient: %s\nQoS: %d\n\nPress q or Ctrl-C to quit\n\n",
            topic_prefix.c_str(), client_id.c_str(), qos);
    }

    void disconnect() {
        if (client) MCHECK(MQTTClient_disconnect(this->client, timeout));
        MQTTClient_destroy(&client);
    }

    void subscribe(std::string topic) {
        MCHECK(MQTTClient_subscribe(client, topic.c_str(), qos));
    }

    void publish(std::string topic, std::string payload) {
        MCHECK(MQTTClient_publish(
            client,
            topic.c_str(),
            payload.size(),
            &payload[0],
            qos,
            true,
            nullptr
        ));
    }

    int received(char *topicName, int topicLen, MQTTClient_message *message)
    {
        std::string topic;

        if (topicLen > 0) {
            topic.resize(topicLen);
            memcpy(&topic[0], topicName, topicLen);
        }
        else {
            topic = topicName;
        }

        std::string payload;
        payload.resize(message->payloadlen);
        memcpy(&payload[0], message->payload, message->payloadlen);
        printf("> %s: %s\n", topic.c_str(), payload.c_str());
        
        receive_monitors(topic, payload);
        
        MQTTClient_freeMessage(&message);
        MQTTClient_free(topicName);
        return 1;
    }

    void delivered(void *context, MQTTClient_deliveryToken dt)
    {
        fprintf(stderr, "Delivered: %d\n", dt);
    }

    void disconnected(void *context, char *cause)
    {
        fprintf(stderr, "\nConnection lost: %s\n", cause);
    }

} queue;

int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    auto queue = static_cast<MessageQueue*>(context);
    return queue->received(topicName, topicLen, message);
}

void mqtt_delivered(void *context, MQTTClient_deliveryToken dt)
{
    auto queue = static_cast<MessageQueue*>(context);
    fprintf(stderr, "Delivered: %d\n", dt);
}

void mqtt_disconnected(void *context, char *cause)
{
    auto queue = static_cast<MessageQueue*>(context);
    fprintf(stderr, "\nConnection lost: %s\n", cause);
}



void receive_monitors_all(const std::string &payload) {
    if (payload == "ON") {
        // -1 doesn't seem to work, so send mouse move event instead
        INPUT input = { INPUT_MOUSE };
        input.mi.dwFlags = MOUSEEVENTF_MOVE;
        SendInput(1, &input, sizeof(INPUT));
    }
    else if (payload == "OFF") {
        SendMessage(window, WM_SYSCOMMAND, SC_MONITORPOWER, 2); // -1: on, 1: low power, 2: off 
    }
    else {
        std::cerr << "Unrecognized power mode " << payload << std::endl;
        return;
    }
}

void publish_monitors_all(const std::string &payload) {
    queue.publish(topic_all_power_state, payload);
}



enum VCPCode {
    None = 0,
    PowerMode = 0xD6
};

struct ParamSource {
    DWORD flag;
    VCPCode vcp;
    std::vector<uint8_t> vcp_domain;
};

static ParamSource PFlag(DWORD flag) {
    return ParamSource{ flag, VCPCode::None };
}
static ParamSource PVCP(VCPCode vcp) {
    return ParamSource{ 0, vcp };
}

struct Param {
    std::string name;
    ParamSource source;
    std::function<std::string(HANDLE)> get_state;
    std::function<void(HANDLE, std::string)> set_state;

    HANDLE monitor;
    std::string topic_state;
    std::string topic_command;
};


std::vector<Param> available_params{
    { "power", PVCP(VCPCode::PowerMode),
        [](HANDLE monitor) {
            MC_VCP_CODE_TYPE vcp_type;
            DWORD cur;
            DWORD max;
            if (!GetVCPFeatureAndVCPFeatureReply(
                monitor,
                VCPCode::PowerMode,
                &vcp_type,
                &cur,
                &max
            )) return std::string("OFF");
            return std::string(cur == 0x01 ? "ON" : "OFF");
        },
        [](HANDLE monitor, std::string payload) {
            uint8_t power;
            if (payload == "ON") power = 0x01;
            else if (payload == "OFF") power = 0x04;
            else {
                std::cerr << "Unrecognized power mode " << payload << std::endl;
                return;
            }

            if (!SetVCPFeature(
                monitor,
                VCPCode::PowerMode,
                power
            )) std::cerr << "Unable to set power mode to " << (int)power << std::endl;
        }
    },
    { "brightness", PFlag(MC_CAPS_BRIGHTNESS),
        [](HANDLE monitor) {
            DWORD min;
            DWORD cur;
            DWORD max;
            if (!GetMonitorBrightness(
                monitor, &min, &cur, &max 
            )) return std::string("-1");
            std::ostringstream stream;
            stream << cur*0xFF/100;
            return stream.str();
        },
        [](HANDLE monitor, std::string payload) {
            int brightness = std::stoi(payload)*100/0xFF;
            if (!SetMonitorBrightness(monitor, brightness)) {
                std::cerr << "Unable to set brightness to " << brightness << std::endl;
            }
        },
    },
    { "temperature", PFlag(MC_CAPS_COLOR_TEMPERATURE),
        [](HANDLE monitor) {
            MC_COLOR_TEMPERATURE temp;
            if (!GetMonitorColorTemperature(
                monitor, &temp
            )) return std::string("-3");
            
            int kelvin = -2;

            switch (temp) {
            case MC_COLOR_TEMPERATURE_UNKNOWN: kelvin = -1; break;
            case MC_COLOR_TEMPERATURE_4000K: kelvin = 4000; break;
            case MC_COLOR_TEMPERATURE_5000K: kelvin = 5000; break;
            case MC_COLOR_TEMPERATURE_6500K: kelvin = 6500; break;
            case MC_COLOR_TEMPERATURE_7500K: kelvin = 7500; break;
            case MC_COLOR_TEMPERATURE_8200K: kelvin = 8200; break;
            case MC_COLOR_TEMPERATURE_9300K: kelvin = 9300; break;
            case MC_COLOR_TEMPERATURE_10000K: kelvin = 10000; break;
            case MC_COLOR_TEMPERATURE_11500K: kelvin = 11500; break;
            }

            int mired = 1000000 / kelvin;

            std::ostringstream stream;
            stream << mired;
            return stream.str();
        },
        [](HANDLE monitor, std::string payload) {
            int mired = std::stoi(payload);
            int kelvin = 1000000 / mired;

            MC_COLOR_TEMPERATURE temp;
            
            if (kelvin < 0) temp = MC_COLOR_TEMPERATURE_UNKNOWN;
            else if (kelvin < 4500) temp = MC_COLOR_TEMPERATURE_4000K;
            else if (kelvin < 5750) temp = MC_COLOR_TEMPERATURE_5000K;
            else if (kelvin < 7000) temp = MC_COLOR_TEMPERATURE_6500K;
            else if (kelvin < 7850) temp = MC_COLOR_TEMPERATURE_7500K;
            else if (kelvin < 8750) temp = MC_COLOR_TEMPERATURE_8200K;
            else if (kelvin < 9300) temp = MC_COLOR_TEMPERATURE_9300K;
            else if (kelvin < 9650) temp = MC_COLOR_TEMPERATURE_10000K;
            else temp = MC_COLOR_TEMPERATURE_11500K;

            if (temp == MC_COLOR_TEMPERATURE_UNKNOWN) return;

            if (!SetMonitorColorTemperature(monitor, temp)) {
                std::cerr << "Unable to set color temperature to " << temp << std::endl;
            }
        }
    },
    { "contrast", PFlag(MC_CAPS_CONTRAST),
        [](HANDLE monitor) {
            DWORD min;
            DWORD cur;
            DWORD max;
            if (!GetMonitorContrast(
                monitor, &min, &cur, &max
            )) return std::string("-1");
            std::ostringstream stream;
            stream << cur * 0xFF / 100;
            return stream.str();
        },
            [](HANDLE monitor, std::string payload) {
            int contrast = std::stoi(payload) * 100 / 0xFF;
            if (!SetMonitorContrast(monitor, contrast)) {
                std::cerr << "Unable to set contrast to " << contrast << std::endl;
            }
        }
    },
    { "degauss", PFlag(MC_CAPS_DEGAUSS) },
    { "technology", PFlag(MC_CAPS_MONITOR_TECHNOLOGY_TYPE),
        [](HANDLE monitor) {
            MC_DISPLAY_TECHNOLOGY_TYPE type;

            if (!GetMonitorTechnologyType(
                monitor, &type
            )) return std::string("N/A");

            switch (type)
            {

            case MC_SHADOW_MASK_CATHODE_RAY_TUBE:
                return std::string("Shadow Mask Cathode Ray Tube (CRT)");

            case MC_APERTURE_GRILL_CATHODE_RAY_TUBE:
                return std::string("Aperture Grill Cathode Ray Tube (CRT)");

            case MC_THIN_FILM_TRANSISTOR:
                return std::string("Thin Film Transistor (TFT)");

            case MC_LIQUID_CRYSTAL_ON_SILICON:
                return std::string("Liquid Crystal on Silicon (LCoS)");

            case MC_PLASMA:
                return std::string("Plasma");

            case MC_ORGANIC_LIGHT_EMITTING_DIODE:
                return std::string("Organic Light Emitting Diode (OLED)");

            case MC_ELECTROLUMINESCENT:
                return std::string("Electroluminescent (ELD)");

            case MC_MICROELECTROMECHANICAL:
                return std::string("Microelectromechanical");

            case MC_FIELD_EMISSION_DEVICE:
                return std::string("Field Emission Display (FED)");

            }
            return std::string("N/A");
        }
    },
    { "rgb-drive", PFlag(MC_CAPS_RED_GREEN_BLUE_DRIVE) },
    { "rgb-gain", PFlag(MC_CAPS_RED_GREEN_BLUE_GAIN) }
};

struct Cap {
    std::string name;
    std::vector<Cap> list;
};

static bool is_whitespace(const std::string &str)
{
    for (std::string::const_iterator it = str.begin(); it != str.end(); ++it)
    {
        if (*it != ' ' &&
            *it != '\t' &&
            *it != '\r' &&
            *it != '\n') return false;
    }
    return true;
}

static Cap parse_capstring(const char **pos) {
    Cap cap;
    std::string str;
    while (**pos > 0) {
        switch (**pos) {
            case '(':
                {
                    (*pos)++;
                    Cap sub = parse_capstring(pos);
                    if (**pos != ')') goto break_loop;
                    sub.name = str;
                    str.clear();
                    cap.list.push_back(sub);
                }
                break;
            case ')':
                cap.list.push_back(Cap{str});
                goto break_loop;
            case ' ':
            case '\t':
            case '\r':
            case '\n':
                if (!is_whitespace(str)) {
                    cap.list.push_back(Cap{str});
                }
                str.clear();
                break;
            default:
                str += **pos;
        }
        (*pos)++;
    }
break_loop:
    return cap;
}

static void dump_cap(const Cap &cap, int level = 0) {
    for (size_t i = 0; i < cap.list.size(); i++) {
        std::cout << std::string(level*2, ' ') << i << ": " << cap.list[i].name << std::endl;
        dump_cap(cap.list[i], level + 1);
    }
}

static Cap parse_capabilities(const char *capabilities) {
    const char *pos = capabilities;
    if (*pos != '(') return Cap{};
    pos++;
    Cap cap = parse_capstring(&pos);
    if (*pos != ')') return Cap{};
    return cap;
}

struct Screen {
    int index;
    HANDLE monitor;
    std::wstring desc;

    DWORD cap_flags;
    DWORD cap_color_temp;
    Cap capabilities;
    std::string model;
    std::vector<Param> supported_params;

    MessageQueue *queue;
    std::string topic_prefix;
    int qos;

    Screen(int index, HANDLE monitor, std::wstring desc) : index(index), monitor(monitor), desc(desc) {
        updateInfo();
    };
    ~Screen() {
        DestroyPhysicalMonitor(monitor);
        monitor = nullptr;
    }

    void printError(const char *msg) {
        std::wcerr << index << ": " << desc << ": " << msg << std::endl;
    }

    void updateInfo() {
        std::wcout << index << ": " << desc << " (" << monitor << ")" << std::endl;

        {
            printf("  flags\n");
            if (!GetMonitorCapabilities(
                monitor,
                &cap_flags,
                &cap_color_temp
            )) {
                printError("error updating flags");
                return;
            }

            for (auto param : available_params) {
                if (cap_flags & param.source.flag) {
                    std::cout << "    " << param.name << std::endl;
                    supported_params.push_back(param);
                }
            }
        }

        {
            printf("  low level\n");
            printf("    length\n");
            DWORD cap_len;
            if (!GetCapabilitiesStringLength(monitor, &cap_len)) {
                printError("error getting length");
                return;
            }
            std::string cap;
            cap.resize(cap_len);

            printf("    request\n");
            if (!CapabilitiesRequestAndCapabilitiesReply(
                monitor,
                (LPSTR)&cap[0],
                cap_len
            )) {
                printError("request failed");
                return;
            }

            printf("    parse\n");
            capabilities = parse_capabilities(&cap[0]);

            for (Cap cap_main : capabilities.list) {
                if (cap_main.name == "model") {
                    if (cap_main.list.size() > 0) {
                        model = cap_main.list[0].name;
                    }
                }
            }
            printf("    model\n");
            std::cout << "      " << model << std::endl;

            for (Cap cap_main : capabilities.list) {
                if (cap_main.name == "vcp") {
                    for (Cap vcp : cap_main.list) {
                        if (vcp.name.empty()) continue;
                        uint8_t vcp_code = std::stoi(vcp.name, 0, 16);
                        for (auto param : available_params) {
                            if (param.source.vcp == vcp_code) {
                                std::cout << "    " << param.name << std::endl;
                                supported_params.push_back(param);
                                Param &added = supported_params.back();
                                for (auto cap_domain : vcp.list) {
                                    uint8_t dom_code = std::stoi(cap_domain.name, 0, 16);
                                    std::cout << "      " << (int)dom_code << std::endl;
                                    param.source.vcp_domain.push_back(dom_code);
                                }
                            }
                        }
                    }
                }
            }

            // Uncomment to print all the capabilities
            //dump_cap(capabilities);
        }

        printf("  done\n\n");
    }

    void publish(const Param &param, std::string state_override = "")
    {
        std::string state = state_override.empty() ? param.get_state(monitor) : state_override;
        printf("< %s: %s\n", param.topic_state.c_str(), state.c_str());
        queue->publish(param.topic_state, state);
    }

    void publishAll()
    {
        for (auto &param : supported_params) {
            if (!param.topic_state.empty()) {
                publish(param);
            }
        }
    }

    bool receive(const std::string &topic, const std::string &payload)
    {
        for (auto &param : supported_params) {
            if (param.topic_command == topic) {
                param.set_state(monitor, payload);
                publish(param, payload);
                // Maybe update the real state in a bit?
                //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                //publish(param);
                return true;
            }
        }
        return false;
    }

    void connect(MessageQueue *queue)
    {
        if (queue == nullptr) return;

        this->queue = queue;
        this->topic_prefix = queue->topic_prefix + model + "/";
        
        for (auto &param : supported_params) {
            if (param.get_state) {
                param.topic_state = topic_prefix + param.name + "/state";
                std::cout << "PUB " << param.topic_state << std::endl;
            }
            if (param.set_state) {
                param.topic_command = topic_prefix + param.name + "/command";
                queue->subscribe(param.topic_command);
                std::cout << "SUB " << param.topic_command << std::endl;
            }
        }
        publishAll();
    }

};

std::vector<std::shared_ptr<Screen>> screens;

// Might be useful, but not right now.
/*
bool get_monitor_device(int index, DISPLAY_DEVICE &device) {
    bool valid = true;

    ZeroMemory(&device, sizeof(DISPLAY_DEVICE));
    device.cb = sizeof(DISPLAY_DEVICE);

    if (EnumDisplayDevices(NULL, index, &device, 0)) {

        CHAR deviceName[32];
        // DeviceName is first the gpu name
        lstrcpy(deviceName, device.DeviceName);

        std::cout << "device " << deviceName << std::endl;

        // DeviceName is now the monitor name
        EnumDisplayDevices(deviceName, 0, &device, 0);
    }
    else {
        valid = false;
    }

    return valid;
}
//*/

BOOL CALLBACK EnumDisplayCallback(
    _In_ HMONITOR hMonitor,
    _In_ HDC      hdcMonitor,
    _In_ LPRECT   lprcMonitor,
    _In_ LPARAM   dwData
) {
    DWORD physical_num;

    if (!GetNumberOfPhysicalMonitorsFromHMONITOR(hMonitor, &physical_num)) {
        std::cerr << "Unable to get number of physical monitors" << std::endl;
        return true;
    }

    std::vector<PHYSICAL_MONITOR> monitors;

    monitors.resize(physical_num);

    CHECK(GetPhysicalMonitorsFromHMONITOR(hMonitor, physical_num, &monitors[0]));

    for (DWORD i = 0; i < physical_num; i++) {
        PHYSICAL_MONITOR *mon = &monitors[i];
        
        std::shared_ptr<Screen> ptr = std::make_shared<Screen>(
            screens.size(),
            mon->hPhysicalMonitor,
            std::wstring(mon->szPhysicalMonitorDescription)
        );
        Screen &screen = *ptr;
        screens.push_back(ptr);
    }

    return true;
}




void receive_monitors(const std::string &topic, const std::string &payload) {
    if (topic == topic_all_power_command) {
        receive_monitors_all(payload);
        return;
    }
    for (auto ptr : screens) {
        Screen &screen = *ptr;
        bool handled = screen.receive(topic, payload);
        if (handled) break;
    }
}

void connect_monitors() {
    for (auto ptr : screens) {
        Screen &screen = *ptr;
        screen.connect(&queue);
    }
}

void update_monitors() {

    /* Might be useful in a bit.
    DISPLAY_DEVICE monitor;
    for (int i = 0; get_monitor_device(i, monitor); i++) {
        std::cout << i << ":" << std::endl;
        std::cout << monitor.DeviceID << std::endl;
        std::cout << monitor.DeviceKey << std::endl;
        std::cout << monitor.DeviceName << std::endl;
        std::cout << monitor.DeviceString << std::endl;
        std::cout << monitor.StateFlags << std::endl;
    }
    */

    screens.clear();
    if (!EnumDisplayMonitors(NULL, NULL, EnumDisplayCallback, 0)) {
        std::cerr << "Unable to enumerate display monitors" << std::endl;
    }
    connect_monitors();
}

long __stdcall WindowProcedure(HWND window, unsigned int msg, WPARAM wp, LPARAM lp)
{
    switch (msg)
    {
    case WM_DISPLAYCHANGE:
        std::cout << "Detected display change " << wp << " " << lp << std::endl;
        update_monitors();
        return 0L;
    case WM_POWERBROADCAST:
        switch (wp) {
        case PBT_POWERSETTINGCHANGE:
            auto setting = (POWERBROADCAST_SETTING*)lp;
            if (setting->PowerSetting == GUID_MONITOR_POWER_ON) {
                DWORD state = *(DWORD*)&setting->Data[0];
                publish_monitors_all(state == 1 ? "ON" : "OFF");
            }
            // This is the newer API to use, but that just seems to make it less compatible?
            /* else if (setting->PowerSetting == GUID_CONSOLE_DISPLAY_STATE) {
                DWORD state = *(DWORD*)&setting->Data[0];
                std::cout << "GUID_CONSOLE_DISPLAY_STATE " << state << std::endl;
            }*/
            break;
        }

        return 0L;
    case WM_DESTROY:
        std::cout << "\nDestroying window\n";
        PostQuitMessage(0);
        return 0L;
    default:
        return DefWindowProc(window, msg, wp, lp);
    }
}

void windows_run() {
    update_monitors();
    WNDCLASSEX wndclass = {
        sizeof(WNDCLASSEX), 0, WindowProcedure,
        0, 0, GetModuleHandle(0), nullptr,
        nullptr, nullptr,
        0, name.c_str(), nullptr };
    if (RegisterClassEx(&wndclass))
    {
        window = CreateWindowEx(0, name.c_str(), "Spooky hidden ghost title!",
            WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
            CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, GetModuleHandle(0), 0);
        if (window)
        {
            MSG msg;
            RegisterPowerSettingNotification(window, &GUID_MONITOR_POWER_ON, 0);
            // Alternative newer API
            //RegisterPowerSettingNotification(window, &GUID_CONSOLE_DISPLAY_STATE, 0);
            while (GetMessage(&msg, 0, 0, 0)) DispatchMessage(&msg);
        }
    }
}

std::string get_default_topic_prefix() {
    std::string topic_prefix = "pc/";
    char comp_name[MAX_COMPUTERNAME_LENGTH];
    DWORD comp_name_len = MAX_COMPUTERNAME_LENGTH;
    if (GetComputerName(comp_name, &comp_name_len) != 0) {
        topic_prefix.resize(comp_name_len);
        memcpy(&topic_prefix[0], comp_name, comp_name_len);
        std::transform(topic_prefix.begin(), topic_prefix.end(), topic_prefix.begin(), ::tolower);
        topic_prefix += "/";
    }
    topic_prefix += "monitor/";
    return topic_prefix;
}



int main(int argc, char *argv[]) {

    std::cout << name << "\n\n";

    std::string uri;
    std::string user;
    std::string pass;
    std::string topic_prefix = get_default_topic_prefix();

    try {
        YAML::Node config = YAML::LoadFile("config.yaml");

        MQTTClient_connectOptions opts = MQTTClient_connectOptions_initializer;

        auto broker = config["broker"];
        auto username = config["username"];
        auto password = config["password"];
        auto keepalive = config["keepalive"];
        auto prefix = config["prefix"];

        if (!broker) throw "'broker' not found";
        uri = broker.as<std::string>();
        std::cout << "Broker: " << uri << std::endl;

        if (username) {
            user = username.as<std::string>();
            opts.username = user.c_str();
            std::cout << "Username: " << opts.username << std::endl;
        }
        if (password) {
            pass = password.as<std::string>();
            opts.password = pass.c_str();
            std::cout << "Password: *****" << std::endl;
        }
        if (keepalive) {
            opts.keepAliveInterval = keepalive.as<int>();
            std::cout << "Keep alive: " << opts.keepAliveInterval << std::endl;
        }
        if (prefix) {
            topic_prefix = prefix.as<std::string>();
        }

        std::cout << std::endl;

        queue.connect(
            uri,
            name,
            topic_prefix,
            opts
        );

    } catch (YAML::BadFile badFile) {
        std::cerr << "Error: config.yaml not found (" << badFile.msg << ")" << std::endl;
        return 1;
    } catch (const char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 2;
    }

    auto topic_prefix_all = queue.topic_prefix + "monitor/all/";
    topic_all_power_state = topic_prefix_all + "power/state";
    topic_all_power_command = topic_prefix_all + "power/command";
    queue.subscribe(topic_all_power_command);
    
    std::thread windows_thread(windows_run);

    int ch;
    do
    {
        ch = _getch();
    } while (ch != 'Q' && ch != 'q' && ch != 3 /* Ctrl-C */);

    {
        std::lock_guard<std::mutex> lock(mutex);
        done = true;
    }
    cv.notify_all();
    
    windows_thread.detach();

    queue.disconnect();

    return 0;
}