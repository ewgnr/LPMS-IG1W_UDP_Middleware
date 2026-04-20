#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <vector>
#include <queue>
#include <mutex>
#include <cstdint>
#include <chrono>
#include <thread>
#include <cstring>
#include <conio.h>

#pragma comment(lib, "ws2_32.lib")

#pragma pack(push, 1)
struct ImuPacket
{
    float qw, qx, qy, qz;

    float wx, wy, wz;   // angular velocity
    float ax, ay, az;   // linear acc

    uint32_t timestamp;
};
#pragma pack(pop)

SOCKET udpSock;
sockaddr_in udpAddr;
bool udpEnabled = true;

uint16_t lastSentCmd = 0;

constexpr char HOST[] = "10.10.0.1";
constexpr int PORT = 65006;

bool gotInfo = false;
bool gotConfig = false;
bool gotStreamAck = false;
bool printEnabled = false;

constexpr uint8_t BYTE_START = 0x3A;
constexpr uint8_t BYTE_END0  = 0x0D;
constexpr uint8_t BYTE_END1  = 0x0A;

constexpr size_t MAX_PAYLOAD = 4096;
constexpr uint16_t SENSOR_ID = 0x0001;


uint16_t calcChecksumIG1(const std::vector<uint8_t>& frame)
{
    uint16_t cs = 0;

    // skip START byte
    for (size_t i = 1; i < frame.size(); i++)
    {
        // exclude checksum + end bytes (last 3 bytes)
        if (i >= frame.size() - 3)
            break;

        cs += frame[i];
    }

    return cs;
}

struct Command
{
    uint16_t cmd = 0;
    std::vector<uint8_t> data;

    bool sent = false;
    bool processed = false;
    int retry = 0;

    std::chrono::steady_clock::time_point lastSend;
};

struct Packet
{
    uint16_t address = 0;
    uint16_t cmd = 0;
    uint16_t len = 0;
    std::vector<uint8_t> payload;
    uint16_t checksum = 0;
};

enum State
{
    ST_START,
    ST_ADDR0,
    ST_ADDR1,
    ST_CMD0,
    ST_CMD1,
    ST_LEN0,
    ST_LEN1,
    ST_PAYLOAD,
    ST_CS0,
    ST_CS1,
    ST_END0,
    ST_END1
};

struct Parser
{
    State state = ST_START;
    Packet pkt;
    uint16_t cs = 0;
    size_t index = 0;
};

std::queue<Command> cmdQueue;
std::mutex cmdMutex;

void pushCommand(Command c)
{
    std::lock_guard<std::mutex> lock(cmdMutex);
    cmdQueue.push(c);
}

void sendCommand(SOCKET sock, Command& c)
{
    std::vector<uint8_t> out;

    out.push_back(BYTE_START);

    // sensor ID
    out.push_back(SENSOR_ID & 0xFF);
    out.push_back((SENSOR_ID >> 8) & 0xFF);

    // command
    out.push_back(c.cmd & 0xFF);
    out.push_back((c.cmd >> 8) & 0xFF);

    // length
    uint16_t len = (uint16_t)c.data.size();
    out.push_back(len & 0xFF);
    out.push_back(len >> 8);

    // payload
    out.insert(out.end(), c.data.begin(), c.data.end());

    // checksum (REAL IG1 FIX)
    uint16_t cs = calcChecksumIG1(out);
    out.push_back(cs & 0xFF);
    out.push_back(cs >> 8);

    // end
    out.push_back(BYTE_END0);
    out.push_back(BYTE_END1);

    // DEBUG FRAME (IMPORTANT)
    std::cout << "FRAME:";
    for (auto b : out)
        std::cout << " " << std::hex << (int)b;
    std::cout << std::dec << "\n";

    send(sock, (char*)out.data(), (int)out.size(), 0);

    lastSentCmd = c.cmd;

    std::cout << "[SEND] cmd=0x" << std::hex << c.cmd << std::dec << "\n";

    c.sent = true;
    c.lastSend = std::chrono::steady_clock::now();
}

enum InitState
{
    INIT_IDLE,

    INIT_STARTUP,

    INIT_GET_INFO,
    INIT_WAIT_INFO,

    INIT_GET_CONFIG,
    INIT_WAIT_CONFIG,

    INIT_SET_CONFIG,
    INIT_WAIT_SET,

    INIT_START_STREAM,
    INIT_WAIT_STREAM,

    INIT_DONE
};

InitState initState = INIT_STARTUP;
bool infoReceived = false;
bool configReceived = false;

void processInit()
{
    if (!cmdQueue.empty())
        return;

    switch (initState)
    {
        case INIT_IDLE:
            std::cout << "[INIT] Request sensor info\n";
            pushCommand({0x0007, {}});
            initState = INIT_WAIT_INFO;
            break;

        case INIT_WAIT_INFO:
            if (gotInfo)
            {
                std::cout << "[INIT] Info OK → request config\n";
                pushCommand({0x0005, {}});
                initState = INIT_WAIT_CONFIG;
                gotInfo = false;
            }
            break;

        case INIT_WAIT_CONFIG:
            if (gotConfig)
            {
                std::cout << "[INIT] Config OK → set config\n";
                pushCommand({0x0006, {0x0F, 0x00}});
                initState = INIT_WAIT_SET;
                gotConfig = false;
            }
            break;

        case INIT_WAIT_SET:
            if (gotStreamAck)
            {
                std::cout << "[INIT] Start stream\n";
                pushCommand({0x0001, {}});
                initState = INIT_WAIT_STREAM;
                gotStreamAck = false;
            }
            break;

        case INIT_WAIT_STREAM:
            std::cout << "✅ STREAM ACTIVE\n";
            initState = INIT_DONE;
            break;

        case INIT_DONE:
        default:
            break;
    }
}

void handlePacket(const Packet& p)
{
    const size_t n = p.payload.size() / 4;
    /*
    std::cout << "[RECV] cmd=0x" << std::hex << p.cmd
              << " floats=" << std::dec << n << "\n";
    */
    if (n != 45)
    {
        std::cout << "[ERROR] incomplete frame\n";
        return;
    }

    const float* f = reinterpret_cast<const float*>(p.payload.data());

    // =========================
    // CORE
    // =========================
    float timestamp = f[0];

    float acc_raw[3] = {f[1], f[2], f[3]};
    float acc_cal[3] = {f[4], f[5], f[6]};

    // =========================
    // GYRO1
    // =========================
    float gyro1_raw[3]   = {f[7], f[8], f[9]};
    float gyro1_bias[3]  = {f[10], f[11], f[12]};
    float gyro1_align[3] = {f[13], f[14], f[15]};

    // =========================
    // GYRO2
    // =========================
    float gyro2_raw[3]   = {f[16], f[17], f[18]};
    float gyro2_bias[3]  = {f[19], f[20], f[21]};
    float gyro2_align[3] = {f[22], f[23], f[24]};

    // =========================
    // MAG
    // =========================
    float mag_raw[3] = {f[25], f[26], f[27]};
    float mag_cal[3] = {f[28], f[29], f[30]};

    // =========================
    // ANGULAR VELOCITY
    // =========================
    float ang_vel[3] = {f[31], f[32], f[33]};

    // =========================
    // ORIENTATION
    // =========================
    float quat[4]  = {f[34], f[35], f[36], f[37]};
    float euler[3] = {f[38], f[39], f[40]};

    // =========================
    // LINEAR ACC + TEMP
    // =========================
    float lin_acc[3] = {f[41], f[42], f[43]};
    float temp       = f[44];

    // =========================
    // SANITY CHECKS
    // =========================
    auto ok = [](float v)
    {
        return std::isfinite(v) && std::fabs(v) < 1e6;
    };

    if (!ok(timestamp))
        std::cout << "[WARN] bad timestamp\n";

    if (printEnabled)
    {
        std::cout << "==============================\n";
        std::cout << "[LPMS IG1W FRAME]\n";
        std::cout << "==============================\n";

        // =========================
        // RAW SENSOR DEBUG ONLY
        // =========================
        std::cout << "\n[RAW SENSOR DEBUG]\n";

        std::cout << "gyro1_raw: "
            << gyro1_raw[0] << " "
            << gyro1_raw[1] << " "
            << gyro1_raw[2] << "\n";

        std::cout << "gyro2_raw: "
            << gyro2_raw[0] << " "
            << gyro2_raw[1] << " "
            << gyro2_raw[2] << "\n";


        // =========================
        // CALIBRATION DEBUG ONLY
        // =========================
        std::cout << "\n[CALIBRATION DEBUG]\n";

        std::cout << "gyro1_bias: "
            << gyro1_bias[0] << " "
            << gyro1_bias[1] << " "
            << gyro1_bias[2] << "\n";

        std::cout << "gyro2_bias: "
            << gyro2_bias[0] << " "
            << gyro2_bias[1] << " "
            << gyro2_bias[2] << "\n";

        std::cout << "gyro1_align: "
            << gyro1_align[0] << " "
            << gyro1_align[1] << " "
            << gyro1_align[2] << "\n";

        std::cout << "gyro2_align: "
            << gyro2_align[0] << " "
            << gyro2_align[1] << " "
            << gyro2_align[2] << "\n";


        // =========================
        // REAL IMU OUTPUT (USE THIS)
        // =========================
        std::cout << "\n[REAL IMU OUTPUT]\n";

        std::cout << "angular_velocity: "
            << ang_vel[0] << " "
            << ang_vel[1] << " "
            << ang_vel[2] << "\n";

        std::cout << "quaternion: "
            << quat[0] << " "
            << quat[1] << " "
            << quat[2] << " "
            << quat[3] << "\n";

        std::cout << "euler: "
            << euler[0] << " "
            << euler[1] << " "
            << euler[2] << "\n";

        std::cout << "linear_acc: "
            << lin_acc[0] << " "
            << lin_acc[1] << " "
            << lin_acc[2] << "\n";

        std::cout << "\n------------------------------\n";
    }

    if (udpEnabled)
    {
        ImuPacket pkt;

        pkt.qw = quat[0];
        pkt.qx = quat[1];
        pkt.qy = quat[2];
        pkt.qz = quat[3];

        pkt.wx = ang_vel[0];
        pkt.wy = ang_vel[1];
        pkt.wz = ang_vel[2];

        pkt.ax = lin_acc[0];
        pkt.ay = lin_acc[1];
        pkt.az = lin_acc[2];

        pkt.timestamp = (uint32_t)timestamp;

        sendto(udpSock,
               (char*)&pkt,
               sizeof(pkt),
               0,
               (sockaddr*)&udpAddr,
               sizeof(udpAddr));
    }
}

void processByte(Parser& ps, uint8_t b)
{
    switch (ps.state)
    {
        case ST_START:
            if (b == BYTE_START)
            {
                ps.cs = 0;
                ps.index = 0;
                ps.pkt = Packet{};
                ps.state = ST_ADDR0;
            }
            break;

        case ST_ADDR0:
            ps.pkt.address = b;
            ps.cs += b;
            ps.state = ST_ADDR1;
            break;

        case ST_ADDR1:
            ps.pkt.address |= (b << 8);
            ps.cs += b;
            ps.state = ST_CMD0;
            break;

        case ST_CMD0:
            ps.pkt.cmd = b;
            ps.cs += b;
            ps.state = ST_CMD1;
            break;

        case ST_CMD1:
            ps.pkt.cmd |= (b << 8);
            ps.cs += b;
            ps.state = ST_LEN0;
            break;

        case ST_LEN0:
            ps.pkt.len = b;
            ps.cs += b;
            ps.state = ST_LEN1;
            break;

        case ST_LEN1:
            ps.pkt.len |= (uint16_t(b) << 8);
            ps.cs += b;

            if (ps.pkt.len > MAX_PAYLOAD)
                ps.state = ST_START;
            else if (ps.pkt.len == 0)
                ps.state = ST_CS0;
            else
            {
                ps.pkt.payload.clear();
                ps.index = 0;
                ps.state = ST_PAYLOAD;
            }
            break;

        case ST_PAYLOAD:
            ps.pkt.payload.push_back(b);
            ps.cs += b;

            if (++ps.index >= ps.pkt.len)
                ps.state = ST_CS0;
            break;

        case ST_CS0:
            ps.pkt.checksum = b;
            ps.state = ST_CS1;
            break;

        case ST_CS1:
            ps.pkt.checksum |= (b << 8);
            ps.state = ST_END0;
            break;

        case ST_END0:
            ps.state = (b == BYTE_END0) ? ST_END1 : ST_START;
            break;

        case ST_END1:
            if (b == BYTE_END1)
            {
                handlePacket(ps.pkt);  // accept IG1W frames as-is
            }
            ps.state = ST_START;
            break;
            }
}

void processCommands(SOCKET sock)
{
    std::lock_guard<std::mutex> lock(cmdMutex);

    if (cmdQueue.empty())
        return;

    Command& c = cmdQueue.front();

    if (!c.sent)
    {
        sendCommand(sock, c);
        return;
    }

    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - c.lastSend).count() > 200)
    {
        if (c.retry < 5)
        {
            std::cout << "[RETRY]\n";
            sendCommand(sock, c);
            c.retry++;
        }
        else
        {
            std::cout << "[FAILED CMD]\n";
            c.processed = true;
        }
    }

    if (c.processed)
        cmdQueue.pop();
}

void processKeyboard()
{
    if (_kbhit())
    {
        char c = _getch();

        if (c == 'p')
        {
            printEnabled = !printEnabled;
            std::cout << "\n[TOGGLE] Printing = "
                      << (printEnabled ? "ON" : "OFF") << "\n";
        }
    }
}

int main()
{
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2), &wsa);

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    inet_pton(AF_INET, HOST, &addr.sin_addr);

    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) != 0)
    {
        std::cout << "connect failed\n";
        return -1;
    }

    std::cout << "CONNECTED\n";

    std::cout <<
    R"(

    ==============================
       LPMS IG1W CLIENT STARTED
    ==============================
    Controls:
      p  -> toggle printing ON/OFF
    ==============================

    )";

    udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    udpAddr = {};
    udpAddr.sin_family = AF_INET;
    udpAddr.sin_port = htons(9001);
    inet_pton(AF_INET, "127.0.0.1", &udpAddr.sin_addr);

    std::cout << "[UDP] sending to 127.0.0.1:9001\n";

    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);

    Parser parser;
    std::vector<uint8_t> buf(8192);

    while (true)
    {
        processKeyboard(); 
        processInit();
        processCommands(sock);

        int n = recv(sock, (char*)buf.data(), buf.size(), 0);

        if (n > 0)
        {
            for (int i = 0; i < n; i++)
                processByte(parser, buf[i]);
        }
        else if (n == 0)
        {
            std::cout << "Disconnected\n";
            break;
        }
        else
        {
            int err = WSAGetLastError();
            if (err != WSAEWOULDBLOCK)
            {
                std::cout << "recv failed: " << err << "\n";
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    closesocket(sock);
    WSACleanup();
}