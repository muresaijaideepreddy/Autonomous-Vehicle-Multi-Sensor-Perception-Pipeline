#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <mcap/types.hpp>

#include <cstring>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <vector>
#include <string>

#include "mcap_export.hpp"

static mcap::McapWriter g_writer;
static mcap::Channel    g_ch_all;
static mcap::Channel    g_ch_clusters;
static mcap::Channel    g_ch_boxes;
static bool             g_initialized = false;
static uint32_t         g_frame_index = 0;

static uint64_t frameStampNs()
{
    return uint64_t(g_frame_index) * 100000000ULL;
}

static void pu8 (std::vector<uint8_t>& b, uint8_t  v) { b.push_back(v); }
static void pu32(std::vector<uint8_t>& b, uint32_t v) {
    b.push_back(uint8_t(v));     b.push_back(uint8_t(v>>8));
    b.push_back(uint8_t(v>>16)); b.push_back(uint8_t(v>>24));
}
static void pi32(std::vector<uint8_t>& b, int32_t v) { pu32(b, uint32_t(v)); }
static void pf32(std::vector<uint8_t>& b, float v) {
    uint32_t bits; std::memcpy(&bits, &v, 4); pu32(b, bits);
}
static void pstr(std::vector<uint8_t>& b, const std::string& s) {
    pu32(b, uint32_t(s.size()) + 1);
    b.insert(b.end(), s.begin(), s.end());
    b.push_back('\0');
    while (b.size() % 4) b.push_back('\0');
}

static std::vector<uint8_t> buildXYZ_CDR(const std::vector<PointsXYZ>& pts,
                                          uint64_t stamp_ns)
{
    const uint32_t N  = uint32_t(pts.size());
    const uint32_t PS = 12u;
    std::vector<uint8_t> b;
    b.reserve(N * PS + 512);

    pu8(b,0x00); pu8(b,0x01); pu8(b,0x00); pu8(b,0x00);
    pi32(b, int32_t(stamp_ns / 1000000000ULL));
    pu32(b, uint32_t(stamp_ns % 1000000000ULL));
    pstr(b, "map");
    pu32(b, 1); pu32(b, N);

    pu32(b, 3);
    const char* names[3] = {"x","y","z"};
    for (int i = 0; i < 3; ++i) {
        pstr(b, names[i]);
        pu32(b, uint32_t(i * 4));
        pu8(b, 7); pu8(b,0); pu8(b,0); pu8(b,0);
        pu32(b, 1);
    }

    pu8(b,0); pu8(b,0); pu8(b,0); pu8(b,0);
    pu32(b, PS); pu32(b, N * PS);
    pu32(b, N * PS);
    for (const auto& p : pts) {
        pf32(b, p.X); pf32(b, p.Y); pf32(b, p.Z);
    }
    pu8(b, 1);
    return b;
}

static std::vector<uint8_t> buildXYZRGB_CDR(const std::vector<PointsXYZRGB>& pts,
                                              uint64_t stamp_ns)
{
    const uint32_t N  = uint32_t(pts.size());
    const uint32_t PS = 16u;
    std::vector<uint8_t> b;
    b.reserve(N * PS + 512);

    pu8(b,0x00); pu8(b,0x01); pu8(b,0x00); pu8(b,0x00);
    pi32(b, int32_t(stamp_ns / 1000000000ULL));
    pu32(b, uint32_t(stamp_ns % 1000000000ULL));
    pstr(b, "map");
    pu32(b, 1); pu32(b, N);

    pu32(b, 6);
    pstr(b,"x"); pu32(b,0);  pu8(b,7); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);
    pstr(b,"y"); pu32(b,4);  pu8(b,7); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);
    pstr(b,"z"); pu32(b,8);  pu8(b,7); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);
    pstr(b,"r"); pu32(b,12); pu8(b,2); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);
    pstr(b,"g"); pu32(b,13); pu8(b,2); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);
    pstr(b,"b"); pu32(b,14); pu8(b,2); pu8(b,0); pu8(b,0); pu8(b,0); pu32(b,1);

    pu8(b,0); pu8(b,0); pu8(b,0); pu8(b,0);
    pu32(b, PS); pu32(b, N * PS);
    pu32(b, N * PS);
    for (const auto& p : pts) {
        pf32(b, p.x); pf32(b, p.y); pf32(b, p.z);
        pu8(b, p.r);  pu8(b, p.g);  pu8(b, p.b);
        pu8(b, 0);
    }
    pu8(b, 1);
    return b;
}

// Foxglove SceneUpdate JSON — strict schema that Foxglove Studio expects
static std::string buildBoxJSON(const std::vector<ExportBox>& boxes,
                                 uint64_t stamp_ns)
{
    const uint64_t sec  = stamp_ns / 1000000000ULL;
    const uint64_t nsec = stamp_ns % 1000000000ULL;

    std::string j = "{";
    j += "\"deletions\":[],";
    j += "\"entities\":[";

    for (size_t i = 0; i < boxes.size(); ++i)
    {
        const auto& box = boxes[i];
        if (i > 0) j += ",";

        // Pick color by label
        std::string color;
        if      (box.label == "CAR")        color = "{\"r\":0.2,\"g\":0.5,\"b\":1.0,\"a\":0.8}";
        else if (box.label == "PEDESTRIAN") color = "{\"r\":1.0,\"g\":0.2,\"b\":0.2,\"a\":0.8}";
        else if (box.label == "CYCLIST")    color = "{\"r\":0.2,\"g\":1.0,\"b\":0.2,\"a\":0.8}";
        else if (box.label == "TRUCK")      color = "{\"r\":1.0,\"g\":0.6,\"b\":0.0,\"a\":0.8}";
        else                                color = "{\"r\":0.7,\"g\":0.7,\"b\":0.7,\"a\":0.5}";

        j += "{";
        j += "\"timestamp\":{\"sec\":" + std::to_string(sec) +
             ",\"nsec\":" + std::to_string(nsec) + "},";
        j += "\"frame_id\":\"map\",";
        j += "\"id\":\"" + box.label + "_" + std::to_string(i) + "\",";
        j += "\"lifetime\":{\"sec\":0,\"nsec\":100000000},";
        j += "\"frame_locked\":true,";
        j += "\"metadata\":[{\"key\":\"label\",\"value\":\"" + box.label + "\"}],";
        j += "\"arrows\":[],";
        j += "\"cubes\":[{";
        j +=   "\"pose\":{";
        j +=     "\"position\":{";
        j +=       "\"x\":" + std::to_string(box.cx) + ",";
        j +=       "\"y\":" + std::to_string(box.cy) + ",";
        j +=       "\"z\":" + std::to_string(box.cz);
        j +=     "},";
        j +=     "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}";
        j +=   "},";
        j +=   "\"size\":{";
        j +=     "\"x\":" + std::to_string(box.sx) + ",";
        j +=     "\"y\":" + std::to_string(box.sy) + ",";
        j +=     "\"z\":" + std::to_string(box.sz);
        j +=   "},";
        j +=   "\"color\":" + color;
        j += "}],";
        j += "\"cylinders\":[],";
        j += "\"lines\":[],";
        j += "\"triangles\":[],";
        j += "\"texts\":[{";
        j +=   "\"pose\":{";
        j +=     "\"position\":{";
        j +=       "\"x\":" + std::to_string(box.cx) + ",";
        j +=       "\"y\":" + std::to_string(box.cy) + ",";
        j +=       "\"z\":" + std::to_string(box.cz + box.sz * 0.5f + 0.3f);
        j +=     "},";
        j +=     "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}";
        j +=   "},";
        j +=   "\"billboard\":true,";
        j +=   "\"font_size\":0.3,";
        j +=   "\"scale_invariant\":false,";
        j +=   "\"color\":{\"r\":1.0,\"g\":1.0,\"b\":1.0,\"a\":1.0},";
        j +=   "\"text\":\"" + box.label + "\"";
        j += "}],";
        j += "\"models\":[],";
        j += "\"spheres\":[],";
        j += "\"points\":[]";
        j += "}";
    }

    j += "]}";
    return j;
}

static void initWriter()
{
    mcap::McapWriterOptions opts("");
    auto st = g_writer.open("output.mcap", opts);
    if (!st.ok()) {
        std::cerr << "[mcap] cannot open output.mcap: " << st.message << "\n";
        return;
    }

    const std::string pc_schema_text = R"(
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

    mcap::Schema pc_schema("sensor_msgs/msg/PointCloud2", "ros2msg", pc_schema_text);
    g_writer.addSchema(pc_schema);

    g_ch_all.topic           = "/points_all";
    g_ch_all.schemaId        = pc_schema.id;
    g_ch_all.messageEncoding = "cdr";
    g_writer.addChannel(g_ch_all);

    g_ch_clusters.topic           = "/points_clusters";
    g_ch_clusters.schemaId        = pc_schema.id;
    g_ch_clusters.messageEncoding = "cdr";
    g_writer.addChannel(g_ch_clusters);

    // Foxglove SceneUpdate — use the official Foxglove schema name
    // Foxglove Studio recognises "foxglove.SceneUpdate" natively
    const std::string box_schema_text = R"({
  "title": "foxglove.SceneUpdate",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "deletions": {"type": "array", "items": {"type": "object"}},
    "entities":  {"type": "array", "items": {"type": "object"}}
  }
})";

    mcap::Schema box_schema("foxglove.SceneUpdate", "jsonschema", box_schema_text);
    g_writer.addSchema(box_schema);

    g_ch_boxes.topic           = "/bounding_boxes";
    g_ch_boxes.schemaId        = box_schema.id;
    g_ch_boxes.messageEncoding = "json";
    g_writer.addChannel(g_ch_boxes);

    g_initialized = true;
    std::cerr << "[mcap] writer ready — 3 channels\n";
}

static void writeMsg(mcap::Channel& ch, uint64_t stamp,
                     const void* data, size_t size)
{
    mcap::Message msg;
    msg.channelId   = ch.id;
    msg.logTime     = stamp;
    msg.publishTime = stamp;
    msg.sequence    = g_frame_index;
    msg.data        = reinterpret_cast<const std::byte*>(data);
    msg.dataSize    = size;
    auto st = g_writer.write(msg);
    if (!st.ok())
        std::cerr << "[mcap] write error on " << ch.topic
                  << ": " << st.message << "\n";
}

void saveFrameToMCAP(const std::vector<PointsXYZ>&    points,
                     const std::vector<PointsXYZRGB>& cluster_points,
                     const std::vector<ExportBox>&    boxes)
{
    if (!g_initialized) initWriter();
    if (!g_initialized) return;

    const uint64_t stamp = frameStampNs();

    std::cerr << "[mcap] frame " << g_frame_index
              << " | all=" << points.size()
              << " clustered=" << cluster_points.size()
              << " boxes=" << boxes.size() << "\n";

    if (!points.empty()) {
        auto cdr = buildXYZ_CDR(points, stamp);
        writeMsg(g_ch_all, stamp, cdr.data(), cdr.size());
    }

    if (!cluster_points.empty()) {
        auto cdr = buildXYZRGB_CDR(cluster_points, stamp);
        writeMsg(g_ch_clusters, stamp, cdr.data(), cdr.size());
    }

    {
        std::string json = buildBoxJSON(boxes, stamp);
        writeMsg(g_ch_boxes, stamp, json.data(), json.size());
    }

    ++g_frame_index;
}

void closePointCloudMCAP()
{
    if (g_initialized) {
        g_writer.close();
        std::cout << "[mcap] saved output.mcap — "
                  << g_frame_index << " frames\n";
        g_initialized = false;
    }
}