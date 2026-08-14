#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
inline constexpr uint32_t crc32_table_entry(uint32_t index) {
    for (unsigned bit = 0; bit < 8; ++bit) {
        index = (index >> 1) ^ (0xEDB88320 & -(index & 1));
    }
    return index;
}

inline constexpr auto make_crc32_table() {
    std::array<uint32_t, 256> table{};
    for (uint32_t i = 0; i < table.size(); ++i) {
        table[i] = crc32_table_entry(i);
    }
    return table;
}

inline constexpr auto crc32_lookup_table = make_crc32_table();

inline uint32_t crc32_seeded(const uint8_t *data, size_t size, const uint32_t seed) {
    uint32_t crc = ~seed;

    while (size--) {
        crc = (crc >> 8) ^ crc32_lookup_table[(crc ^ *data++) & 0xff];
    }

    return ~crc;
}

inline uint32_t crc32(const uint8_t* data, size_t size) {
    return crc32_seeded(data, size, 0xEADA2D49); // 0xA2 seed
}
