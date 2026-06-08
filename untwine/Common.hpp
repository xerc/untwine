#pragma once

#include <stdint.h>

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

#include "FatalError.hpp"
#include "FileDimInfo.hpp"

namespace untwine
{

// Number of cells into which points are put for each octree voxel.
const int CellCount = 128;

using PointCount = uint64_t;
using StringList = std::vector<std::string>;

struct Options
{
    std::string outputName;
    StringList inputFiles;
    std::string tempDir;
    bool doCube;
    size_t fileLimit;
    int level;
    int progressFd;
    bool progressDebug;
    StringList dimNames;
    bool stats;
    std::string a_srs;
    bool no_srs;
    bool metadata;
    bool dummy;
};

template<typename T>
struct Xyz
{
    T x;
    T y;
    T z;
};

struct Transform
{
    bool valid() const
    {
        return (scale.x != 0 && scale.y != 0.0 && scale.z != 0 &&
            !std::isnan(offset.x) && !std::isnan(offset.y) && !std::isnan(offset.z));
    }

    Xyz<double> scale {};
    Xyz<double> offset { std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN() };
};

struct BaseInfo
{
public:
    BaseInfo()
    {};

    bool preserveHeaderFields() const
        {return opts.inputFiles.size() == 1; }

    Options opts;
    pdal::BOX3D bounds;
    size_t pointSize {0};
    std::string outputFile;
    DimInfoList dimInfo;
    pdal::SpatialReference srs;
    int pointFormatId {0};
    uint16_t globalEncoding {0};
    uint16_t creationYear {1};
    uint16_t creationDoy {1};
    uint16_t fileSourceId {0};
    std::string systemId;
    std::string generatingSoftware { "Untwine" };
    Transform xform { { 0.0, 0.0, 0.0 },                             // scale
                      { std::numeric_limits<double>::quiet_NaN(),    // offset
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN() } };
    uint64_t numPoints {0};
};

// We make a special dimension to store the bits (class flags, scanner channel, scan dir, eofl).
const std::string UntwineBitsDimName { "UntwineBitsUntwine" };
// That special dimension is a byte in size.
const pdal::Dimension::Type UntwineBitsType { pdal::Dimension::Type::Unsigned8 };

// PDAL explodes the class flags, leaving us with the following dimensions that map to the
// special "untwine bits" dimension.
inline bool isUntwineBitsDim(std::string_view s)
{
    // IMPORTANT: `lower_bound` needs alphabetically order!
    static const std::array<std::string_view, 8> bitsDims{
        "ClassFlags",
        "EdgeOfFlightLine",
        "KeyPoint",
        "Overlap",
        "ScanChannel",
        "ScanDirectionFlag",
        "Synthetic",
        "Withheld"
    };

    return std::binary_search(bitsDims.begin(), bitsDims.end(), s);
}
// The position is the bit position of a bit, or the number of bits to shift an integer
// value to get it to the right location in the UntwineBits dimension.
inline int getUntwineBitPos(std::string_view s)
{
    struct Entry {
        std::string_view key;
        int value;
    };

    // IMPORTANT: `lower_bound` needs alphabetically order!
    static const std::array<Entry, 8> positions{{
        {"ClassFlags", 0},
        {"EdgeOfFlightLine", 7},
        {"KeyPoint", 1},
        {"Overlap", 3},
        {"ScanChannel", 5},
        {"ScanDirectionFlag", 6},
        {"Synthetic", 0},
        {"Withheld", 2}
    }};

    struct Comp {
        bool operator()(const Entry& e, std::string_view k) const { return e.key < k; }
        bool operator()(std::string_view k, const Entry& e) const { return k < e.key; }
    };

    auto it = std::lower_bound(positions.begin(), positions.end(), s, Comp{});

    if (it != positions.end() && it->key == s)
        return it->value;

    return -1;
}

inline bool isExtraDim(const std::string& name)
{
    using namespace pdal;
    using D = Dimension::Id;

    static const std::array<Dimension::Id, 15> lasDims
    {
        D::X,
        D::Y,
        D::Z,
        D::Intensity,
        D::ReturnNumber,
        D::NumberOfReturns,
        D::Classification,
        D::UserData,
        D::ScanAngleRank,
        D::PointSourceId,
        D::GpsTime,
        D::Red,
        D::Green,
        D::Blue,
        D::Infrared
    };

    D id = Dimension::id(name);
    for (Dimension::Id lasId : lasDims)
        if (lasId == id)
            return false;
    return (name != UntwineBitsDimName);
}

} // namespace untwine
