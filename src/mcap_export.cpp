// mcap_export.cpp
// Pure MCAP writer — NO rclcpp dependency.
// Writes sensor_msgs/msg/PointCloud2 in CDR so Foxglove reads it natively.
// Compile: g++ -std=c++17 mcap_export.cpp -I<mcap_include> \
//              -DMCAP_COMPRESSION_NO_LZ4 -DMCAP_COMPRESSION_NO_ZSTD \
//              -o your_binary

#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <mcap/types.hpp>

#include <cstring>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <vector>
#include <string>

#include "points_structure.hpp"

// -----------------------------------------------------------------------
// Global writer state
// -----------------------------------------------------------------------
static mcap::McapWriter g_writer;
static mcap::Channel    g_channel;
static bool             g_initialized = false;

// -----------------------------------------------------------------------
// CDR serialization helpers  (little-endian)
// -----------------------------------------------------------------------
static void push_u8(std::vector<uint8_t>& b, uint8_t v) {
    b.push_back(v);
}
static void push_u32(std::vector<uint8_t>& b, uint32_t v) {
    b.push_back(uint8_t(v));
    b.push_back(uint8_t(v >> 8));
    b.push_back(uint8_t(v >> 16));
    b.push_back(uint8_t(v >> 24));
}
static void push_i32(std::vector<uint8_t>& b, int32_t v) {
    push_u32(b, static_cast<uint32_t>(v));
}
static void push_f32(std::vector<uint8_t>& b, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    push_u32(b, bits);
}
// CDR string: length (incl. NUL) + chars + NUL, padded to 4 bytes
static void push_str(std::vector<uint8_t>& b, const std::string& s) {
    push_u32(b, static_cast<uint32_t>(s.size()) + 1);
    b.insert(b.end(), s.begin(), s.end());
    b.push_back('\0');
    while (b.size() % 4) b.push_back('\0');
}

// -----------------------------------------------------------------------
// Build CDR buffer for sensor_msgs/msg/PointCloud2
// -----------------------------------------------------------------------
static std::vector<uint8_t> buildCDR(const std::vector<PointsXYZ>& pts,
                                      uint64_t stamp_ns)
{
    const uint32_t N  = static_cast<uint32_t>(pts.size());
    const uint32_t PS = 12u; // point_step: 3 x float32

    std::vector<uint8_t> b;
    b.reserve(N * PS + 512);

    // CDR encapsulation header: little-endian
    push_u8(b, 0x00); push_u8(b, 0x01); push_u8(b, 0x00); push_u8(b, 0x00);

    // std_msgs/Header — stamp
    push_i32(b, static_cast<int32_t>(stamp_ns / 1000000000ULL)); // sec
    push_u32(b, static_cast<uint32_t>(stamp_ns % 1000000000ULL));// nanosec
    push_str(b, "map");   // frame_id

    // height / width
    push_u32(b, 1); // height
    push_u32(b, N); // width

    // fields sequence (3 entries: x, y, z)
    push_u32(b, 3);
    const char* names[3] = {"x","y","z"};
    for (int i = 0; i < 3; ++i) {
        push_str(b, names[i]);
        push_u32(b, static_cast<uint32_t>(i * 4)); // offset
        push_u8 (b, 7);                             // datatype FLOAT32
        push_u8 (b, 0); push_u8(b, 0); push_u8(b, 0); // pad to 4
        push_u32(b, 1);                             // count
    }

    // is_bigendian + pad
    push_u8(b, 0); push_u8(b, 0); push_u8(b, 0); push_u8(b, 0);

    // point_step, row_step
    push_u32(b, PS);
    push_u32(b, N * PS);

    // data[]
    push_u32(b, N * PS);
    for (const auto& p : pts) {
        push_f32(b, p.X);
        push_f32(b, p.Y);
        push_f32(b, p.Z);
    }

    // is_dense
    push_u8(b, 1);

    return b;
}

// -----------------------------------------------------------------------
// Public API  (matches your .hpp)
// -----------------------------------------------------------------------
void savePointCloudToMCAP(const std::vector<PointsXYZ>& points)
{
    if (!g_initialized) {
        mcap::McapWriterOptions opts("");
        auto st = g_writer.open("output.mcap", opts);
        if (!st.ok()) {
            std::cerr << "[mcap] Cannot open output.mcap: " << st.message << "\n";
            return;
        }

        const std::string schema_text = R"(
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: sensor_msgs/PointField
string name
uint32 offset
uint8 datatype
uint32 count
)";
        mcap::Schema schema("sensor_msgs/msg/PointCloud2", "ros2msg", schema_text);
        g_writer.addSchema(schema);

        g_channel.topic           = "/points";
        g_channel.schemaId        = schema.id;
        g_channel.messageEncoding = "cdr";
        g_writer.addChannel(g_channel);

        g_initialized = true;
    }

    if (points.empty()) return;

    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    auto cdr = buildCDR(points, now_ns);

    mcap::Message msg;
    msg.channelId   = g_channel.id;
    msg.logTime     = now_ns;
    msg.publishTime = now_ns;
    msg.data        = reinterpret_cast<const std::byte*>(cdr.data());
    msg.dataSize    = cdr.size();

    auto st = g_writer.write(msg);
    if (!st.ok())
        std::cerr << "[mcap] Write failed: " << st.message << "\n";
}

void closePointCloudMCAP()
{
    if (g_initialized) {
        g_writer.close();
        std::cout << "[mcap] Saved output.mcap\n";
        g_initialized = false;
    }
}